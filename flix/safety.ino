// Copyright (c) 2024 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Fail-safe functions

extern float controlTime;
extern float mavlinkTime;
extern float controlRoll, controlPitch, controlThrottle, controlYaw;

float rcLossTimeout = 1;
float descendTime = 10;
float disarmTilt = radians(120);
float linkLossTimeout = 3; // connection loss timeout in automatic flight, s

void failsafe() {
	rcLossFailsafe();
	linkLossFailsafe();
	autoFailsafe();
	tiltFailsafe();
}

// RC loss failsafe
void rcLossFailsafe() {
	if (!armed) return;
	if (t - controlTime > rcLossTimeout) {
		if (mode == ALT && valid(altitude)) {
			autoLand();
		} else {
			descend();
		}
	}
}

// Landing using altitude hold on RC lost
void autoLand() {
	controlRoll = 0;
	controlPitch = 0;
	controlYaw = 0;
	if (!(altitudeCommand < altitudeGround)) land(); // start automatic landing if not landing yet
}

// Land if the connection to the ground station is lost during automatic flight (e.g. controlled by a script)
void linkLossFailsafe() {
	if (!armed || mode != ALT) return;
	if (!(altitudeCommand >= altitudeGround)) return; // not in automatic flight or already landing
	if (t - mavlinkTime > linkLossTimeout) {
		print("Connection lost\n");
		autoLand();
	}
}

// Smooth descend on RC lost
void descend() {
	mode = AUTO;
	attitudeTarget = Quaternion();
	thrustTarget -= dt / descendTime;
	if (thrustTarget < 0) {
		thrustTarget = 0;
		armed = false;
	}
}

// Allow pilot to interrupt automatic flight
void autoFailsafe() {
	static float roll, pitch, yaw, throttle;
	if (abs(roll - controlRoll) > 0.05 || abs(pitch - controlPitch) > 0.05 || abs(yaw - controlYaw) > 0.05 || abs(throttle - controlThrottle) > 0.05) {
		// controls changed and mode switch is not configured
		if (mode == AUTO && invalid(controlMode)) mode = STAB; // regain control by the pilot
	}
	roll = controlRoll;
	pitch = controlPitch;
	yaw = controlYaw;
	throttle = controlThrottle;
}

// Disarm if tilted too much
void tiltFailsafe() {
	if (!armed) return;
	if (mode != STAB && mode != ALT) return;

	Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
	float tilt = acos(up.z);
	if (disarmTilt && tilt > disarmTilt) {
		armed = false;
	}
}
