// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Attitude estimation using gyro and accelerometer
// Altitude estimation using rangefinder and accelerometer

#include "quaternion.h"
#include "vector.h"
#include "filter.h"
#include "util.h"

Vector rates; // estimated angular rates, rad/s
Quaternion attitude; // estimated attitude
bool landed;

float accWeight = 0.003;
float levelWeight = 0.0002;
LowPassFilter<Vector> ratesFilter(0.2); // cutoff frequency ~ 40 Hz

float altitude = NAN; // estimated height above the surface, m
float verticalSpeed = NAN; // estimated vertical speed, positive up, m/s
float altWeight = 0.3; // altitude correction gain on each range measurement
float verticalSpeedWeight = 1.5; // vertical speed correction gain on each range measurement
float rangeTimeout = 0.2; // invalidate altitude if no valid range measurements, s
LowPassFilter<float> accUpFilter(0.1);

void estimate() {
	applyGyro();
	applyAcc();
	applyLevel();
	estimateAltitude();
}

void applyGyro() {
	// filter gyro to get angular rates
	rates = ratesFilter.update(gyro);

	// apply rates to attitude
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(rates * dt));
}

void applyAcc() {
	// test should we apply accelerometer gravity correction
	landed = !motorsActive() && abs(acc.norm() - ONE_G) < ONE_G * 0.1f;

	if (!landed) return;

	// calculate accelerometer correction
	Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
	Vector correction = Vector::rotationVectorBetween(acc, up) * accWeight;

	// apply correction
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(correction));
}

void applyLevel() {
	if (landed) return;
	if (thrustTarget < 0.1) return; // skip at idle thrust

	// assume the pilot keeps the drone more or less level in flight
	Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
	Vector correction = Vector::rotationVectorBetween(Vector(0, 0, 1), up) * levelWeight;
	attitude = Quaternion::rotate(attitude, Quaternion::fromRotationVector(correction));
}

void estimateAltitude() {
	static float lastRangeTime = NAN;

	if (invalid(rangeTime) || t - rangeTime > rangeTimeout) { // no fresh range measurements
		altitude = NAN;
		verticalSpeed = NAN;
		return;
	}

	Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude); // world up in body frame

	// predict using vertical acceleration
	if (valid(altitude)) {
		float accUp = accUpFilter.update(Vector::dot(acc, up) - ONE_G);
		verticalSpeed += accUp * dt;
		altitude += verticalSpeed * dt;
	}

	if (rangeTime == lastRangeTime) return; // no new measurement
	lastRangeTime = rangeTime;

	if (up.z < 0.5) return; // tilted too much for the measurement to be meaningful
	float measured = range * up.z; // vertical distance

	if (invalid(altitude)) { // initialize
		altitude = measured;
		verticalSpeed = 0;
		accUpFilter.reset();
		return;
	}

	// correct using the rangefinder
	float error = measured - altitude;
	altitude += altWeight * error;
	verticalSpeed += verticalSpeedWeight * error;
}
