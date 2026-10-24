// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Flight control

#include "vector.h"
#include "quaternion.h"
#include "pid.h"
#include "filter.h"
#include "util.h"

const int RAW = 0, ACRO = 1, STAB = 2, AUTO = 3, ALT = 4; // flight modes
int mode = STAB;
bool armed = false;

Quaternion attitudeTarget;
Vector ratesTarget;
Vector ratesExtra; // feedforward rates
Vector torqueTarget;
float thrustTarget;
float altitudeTarget = NAN;
float verticalSpeedTarget = NAN;
float hoverEstimate = NAN; // learned hover thrust in ALT mode

PID rollRatePID(0.05, 0.2, 0.001, 0.3, 0.2);
PID pitchRatePID(0.05, 0.2, 0.001, 0.3, 0.2);
PID yawRatePID(0.3, 0, 0, 0.3);
PID rollPID(6);
PID pitchPID(6);
PID yawPID(3);
PID altitudePID(2); // altitude error -> vertical speed
PID verticalSpeedPID(0.25, 0.5, 0, 0.4); // vertical speed error -> thrust
float hoverThrust = 0.4; // thrust to hover, used on takeoff in ALT mode, learned automatically on landing
float verticalSpeedMax = 0.5; // m/s
float altitudeGround = 0.05; // below this altitude the drone is considered on the ground, m
Vector maxRate(radians(360), radians(360), radians(360));
float tiltMax = radians(30);
int flightModes[] = {STAB, STAB, STAB}; // map for rc mode switch

extern const int MOT_RL, MOT_RR, MOT_FR, MOT_FL;
extern float controlRoll, controlPitch, controlThrottle, controlYaw, controlMode;
extern float rangeMax;

void control() {
	interpretControls();
	failsafe();
	controlAltitude();
	controlAttitude();
	controlRates();
	controlTorque();
}

void interpretControls() {
	if (controlMode < 0.25) mode = flightModes[0];
	else if (controlMode <= 0.75) mode = flightModes[1];
	else if (controlMode > 0.75) mode = flightModes[2];

	if (mode == AUTO) return; // pilot is not effective in AUTO mode

	if (controlThrottle < 0.05 && controlYaw > 0.95) armed = true; // arm gesture
	if (controlThrottle < 0.05 && controlYaw < -0.95) armed = false; // disarm gesture

	if (abs(controlYaw) < 0.1) controlYaw = 0; // yaw dead zone

	thrustTarget = controlThrottle;

	if (mode == STAB || mode == ALT) {
		float yawTarget = attitudeTarget.getYaw();
		if (!armed || invalid(yawTarget) || controlYaw != 0) yawTarget = attitude.getYaw(); // reset yaw target
		attitudeTarget = Quaternion::fromEuler(Vector(controlRoll * tiltMax, controlPitch * tiltMax, yawTarget));
		ratesExtra = Vector(0, 0, -controlYaw * maxRate.z); // positive yaw stick means clockwise rotation in FLU
	}

	if (mode == ACRO) {
		attitudeTarget.invalidate(); // skip attitude control
		ratesTarget.x = controlRoll * maxRate.x;
		ratesTarget.y = controlPitch * maxRate.y;
		ratesTarget.z = -controlYaw * maxRate.z; // positive yaw stick means clockwise rotation in FLU
	}

	if (mode == RAW) { // direct torque control
		attitudeTarget.invalidate(); // skip attitude control
		ratesTarget.invalidate(); // skip rate control
		torqueTarget = Vector(controlRoll, controlPitch, -controlYaw) * 0.1;
	}
}

void controlAltitude() {
	static float thrustBase = NAN; // hover thrust guess at the moment of engaging
	static float engageTime = NAN;

	if (mode != ALT || !armed) {
		altitudeTarget = NAN;
		verticalSpeedTarget = NAN;
		hoverEstimate = NAN;
		return;
	}

	// Throttle stick: center - hold altitude, up/down - climb/descend
	const float deadZone = 0.2;
	float stick = (controlThrottle - 0.5f) * 2;
	if (abs(stick) < deadZone) stick = 0;
	else stick = (stick - (stick > 0 ? deadZone : -deadZone)) / (1 - deadZone);
	stick = constrain(stick, -1, 1);
	float climbRate = stick * verticalSpeedMax;

	if (invalid(altitude)) {
		// no altitude data (out of range or sensor failure)
		altitudeTarget = NAN;
		verticalSpeedTarget = NAN;
		if (valid(hoverEstimate)) { // keep the hover thrust, let the pilot adjust it
			thrustTarget = constrain(hoverEstimate + stick * 0.1f, 0.1f, 1.0f);
		} // otherwise use the throttle stick directly like in STAB
		return;
	}

	// Stay on the ground until the pilot raises the throttle stick
	bool onGround = altitude < altitudeGround;
	if (onGround && stick <= 0 && (invalid(altitudeTarget) || altitudeTarget < altitudeGround)) {
		if (valid(hoverEstimate) && t - engageTime > 5) {
			hoverThrust = constrain(hoverEstimate, 0.1f, 0.9f); // remember the learned hover thrust for the next takeoff
		}
		altitudeTarget = NAN; // landed
		verticalSpeedTarget = NAN;
		hoverEstimate = NAN;
		thrustTarget = 0;
		return;
	}

	// Engage altitude hold
	if (invalid(altitudeTarget)) {
		altitudeTarget = altitude;
		if (valid(hoverEstimate)) thrustBase = hoverEstimate; // altitude data is back
		else if (onGround) thrustBase = hoverThrust; // takeoff
		else thrustBase = controlThrottle; // switched to ALT in flight
		if (invalid(hoverEstimate)) engageTime = t;
		altitudePID.reset();
		verticalSpeedPID.reset();
	}

	altitudeTarget += climbRate * dt;
	altitudeTarget = constrain(altitudeTarget, 0, rangeMax - 0.1f);

	verticalSpeedTarget = altitudePID.update(altitudeTarget - altitude) + climbRate;
	verticalSpeedTarget = constrain(verticalSpeedTarget, -verticalSpeedMax, verticalSpeedMax);

	float thrust = thrustBase + verticalSpeedPID.update(verticalSpeedTarget - verticalSpeed);
	hoverEstimate = thrustBase + constrain(verticalSpeedPID.i * verticalSpeedPID.integral, -verticalSpeedPID.windup, verticalSpeedPID.windup);

	// Compensate thrust loss when tilted
	Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
	thrustTarget = constrain(thrust / constrain(up.z, 0.7f, 1.0f), 0.1f, 1.0f);
}

void controlAttitude() {
	if (!armed || attitudeTarget.invalid() || thrustTarget < 0.1) return; // skip attitude control

	const Vector up(0, 0, 1);
	Vector upActual = Quaternion::rotateVector(up, attitude);
	Vector upTarget = Quaternion::rotateVector(up, attitudeTarget);

	Vector error = Vector::rotationVectorBetween(upTarget, upActual);

	ratesTarget.x = rollPID.update(error.x) + ratesExtra.x;
	ratesTarget.y = pitchPID.update(error.y) + ratesExtra.y;

	float yawError = wrapAngle(attitudeTarget.getYaw() - attitude.getYaw());
	ratesTarget.z = yawPID.update(yawError) + ratesExtra.z;
}


void controlRates() {
	if (!armed || ratesTarget.invalid() || thrustTarget < 0.1) return; // skip rates control

	Vector error = ratesTarget - rates;

	// Calculate desired torque, where 0 - no torque, 1 - maximum possible torque
	torqueTarget.x = rollRatePID.update(error.x);
	torqueTarget.y = pitchRatePID.update(error.y);
	torqueTarget.z = yawRatePID.update(error.z);
}

void controlTorque() {
	if (!torqueTarget.valid()) return; // skip torque control

	if (!armed) {
		for (float& m : motors) m = 0; // stop motors if disarmed
		return;
	}

	if (thrustTarget < 0.1) {
		for (float& m : motors) m = 0.1; // idle thrust
		return;
	}

	motors[MOT_FL] = thrustTarget + torqueTarget.x - torqueTarget.y + torqueTarget.z;
	motors[MOT_FR] = thrustTarget - torqueTarget.x - torqueTarget.y - torqueTarget.z;
	motors[MOT_RL] = thrustTarget + torqueTarget.x + torqueTarget.y - torqueTarget.z;
	motors[MOT_RR] = thrustTarget - torqueTarget.x + torqueTarget.y + torqueTarget.z;

	desaturate(); // prioritize angle control over thrust control
}

void desaturate() {
	float max_ = -INFINITY, min_ = INFINITY;
	float correction = 0;

	// Find maximum and minimum thrust
	for (float m : motors) {
		if (m > max_) max_ = m;
		if (m < min_) min_ = m;
	}

	if (max_ > 1) { // thrust is above maximum
		correction = max_ - 1;
	} else if (min_ < 0) { // thrust is below minimum
		correction = min_;
	}

	for (float& m : motors) {
		m -= correction;
	}
}

const char* getModeName() {
	switch (mode) {
		case RAW: return "RAW";
		case ACRO: return "ACRO";
		case STAB: return "STAB";
		case AUTO: return "AUTO";
		case ALT: return "ALT";
		default: return "UNKNOWN";
	}
}
