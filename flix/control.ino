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
float altitudeCommand = NAN; // commanded altitude in automatic flight (takeoff/land), NAN - pilot controls the altitude
float commandThrottle = NAN; // throttle stick position at the moment of the command

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
float takeoffAltitude = 0.5; // default takeoff altitude, m
float landSpeed = 0.2; // automatic landing speed, m/s
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
	if (switchMode() >= 0) mode = switchMode();

	if (mode == AUTO) return; // pilot is not effective in AUTO mode

	if (controlThrottle < 0.05 && controlYaw > 0.95) armed = true; // arm gesture
	if (controlThrottle < 0.05 && controlYaw < -0.95) armed = false; // disarm gesture

	if (abs(controlYaw) < 0.1) controlYaw = 0; // yaw dead zone

	if (mode != ALT) thrustTarget = controlThrottle; // in ALT mode thrust is set by controlAltitude

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
	static float lastAltitude = NAN; // last valid altitude

	if (mode != ALT || !armed) {
		altitudeTarget = NAN;
		verticalSpeedTarget = NAN;
		hoverEstimate = NAN;
		lastAltitude = NAN;
		altitudeCommand = NAN;
		if (mode == ALT) thrustTarget = 0; // disarmed
		return;
	}

	// Throttle stick: center - hold altitude, up/down - climb/descend
	const float deadZone = 0.2;
	float stick = (controlThrottle - 0.5f) * 2;
	if (abs(stick) < deadZone) stick = 0;
	else stick = (stick - (stick > 0 ? deadZone : -deadZone)) / (1 - deadZone);
	stick = constrain(stick, -1, 1);

	// Automatic flight (takeoff/land commands): the pilot takes over by moving the throttle stick
	if (valid(altitudeCommand) && abs(controlThrottle - commandThrottle) > 0.1f) {
		altitudeCommand = NAN;
		print("Pilot took control\n");
	}
	bool automatic = valid(altitudeCommand);
	if (automatic) stick = 0; // throttle stick is ignored
	float climbRate = stick * verticalSpeedMax;

	if (invalid(altitude)) {
		// no altitude data (out of range or sensor failure)
		altitudeTarget = NAN;
		verticalSpeedTarget = NAN;
		if (invalid(hoverEstimate)) { // never engaged, use the throttle stick directly like in STAB
			thrustTarget = automatic ? 0 : controlThrottle;
			return;
		}
		bool aboveRange = valid(lastAltitude) && lastAltitude > rangeMax * 0.7f;
		bool landing = automatic && altitudeCommand < altitudeGround;
		if (aboveRange || landing) {
			hoverEstimate = max(hoverEstimate - 0.1f * dt, 0.1f); // descend until the range is back
		}
		thrustTarget = constrain(hoverEstimate + stick * 0.1f, 0.1f, 1.0f); // keep the hover thrust, let the pilot adjust it
		return;
	}
	lastAltitude = altitude;

	// Stay on the ground until the pilot raises the throttle stick or takeoff is commanded
	bool onGround = altitude < altitudeGround;
	bool down = automatic ? altitudeCommand < altitudeGround : stick <= 0;
	if (onGround && down && (invalid(altitudeTarget) || altitudeTarget < altitudeGround)) {
		if (valid(hoverEstimate) && t - engageTime > 5) {
			hoverThrust = constrain(hoverEstimate, 0.1f, 0.9f); // remember the learned hover thrust for the next takeoff
		}
		altitudeTarget = NAN; // landed
		verticalSpeedTarget = NAN;
		hoverEstimate = NAN;
		thrustTarget = 0;
		if (automatic) { // automatic landing complete
			altitudeCommand = NAN;
			armed = false;
			print("Landed\n");
		}
		return;
	}

	// Engage altitude hold
	if (invalid(altitudeTarget)) {
		altitudeTarget = altitude;
		if (valid(hoverEstimate)) thrustBase = hoverEstimate; // altitude data is back
		else if (onGround) thrustBase = hoverThrust * 0.85f; // takeoff, the integral term adds the rest
		else thrustBase = controlThrottle; // switched to ALT in flight
		if (invalid(hoverEstimate)) engageTime = t;
		altitudePID.reset();
		verticalSpeedPID.reset();
	}

	if (automatic) { // move the target towards the commanded altitude
		float speed = altitudeCommand < altitudeTarget ? landSpeed : verticalSpeedMax;
		float step = constrain(altitudeCommand - altitudeTarget, -speed * dt, speed * dt);
		climbRate = dt > 0 ? step / dt : 0;
	}
	altitudeTarget += climbRate * dt;
	altitudeTarget = constrain(altitudeTarget, 0, rangeMax - 0.1f);

	verticalSpeedTarget = altitudePID.update(altitudeTarget - altitude) + climbRate;
	verticalSpeedTarget = constrain(verticalSpeedTarget, -verticalSpeedMax, verticalSpeedMax);

	float thrust = thrustBase + verticalSpeedPID.update(verticalSpeedTarget - verticalSpeed);
	if (verticalSpeedPID.i > 0) { // anti-windup: don't accumulate the integral beyond its limit
		float limit = verticalSpeedPID.windup / verticalSpeedPID.i;
		verticalSpeedPID.integral = constrain(verticalSpeedPID.integral, -limit, limit);
	}
	hoverEstimate = thrustBase + constrain(verticalSpeedPID.i * verticalSpeedPID.integral, -verticalSpeedPID.windup, verticalSpeedPID.windup);

	// Compensate thrust loss when tilted
	Vector up = Quaternion::rotateVector(Vector(0, 0, 1), attitude);
	thrustTarget = constrain(thrust / constrain(up.z, 0.7f, 1.0f), 0.1f, 1.0f);
}

// Automatic takeoff to the given altitude (or change the altitude in flight), returns false if denied
bool takeoff(float alt) {
	if (invalid(alt) || alt <= 0) alt = takeoffAltitude;
	if (invalid(altitude)) {
		print("Takeoff denied: no altitude data, check the range sensor\n");
		return false;
	}
	if (switchMode() >= 0 && switchMode() != ALT) {
		print("Takeoff denied: set the mode switch to ALT\n");
		return false;
	}
	if (!armed && altitude >= altitudeGround) {
		print("Takeoff denied: disarmed in the air\n");
		return false;
	}
	alt = constrain(alt, altitudeGround * 2, rangeMax - 0.1f);
	mode = ALT;
	armed = true;
	altitudeCommand = alt;
	commandThrottle = controlThrottle;
	print("Takeoff to %.2f m\n", alt);
	return true;
}

// Automatic landing, returns false if denied
bool land() {
	if (!armed) {
		print("Land denied: disarmed\n");
		return false;
	}
	if (invalid(altitude) && (mode != ALT || invalid(hoverEstimate))) {
		print("Land denied: no altitude data\n");
		return false;
	}
	if (switchMode() >= 0 && switchMode() != ALT) {
		print("Land denied: set the mode switch to ALT\n");
		return false;
	}
	if (mode != ALT) altitudeTarget = NAN; // engage altitude hold from the current altitude
	mode = ALT;
	altitudeCommand = 0;
	commandThrottle = controlThrottle;
	print("Landing\n");
	return true;
}

// Flight mode selected by the RC mode switch, -1 if the switch is not used
int switchMode() {
	if (controlMode < 0.25) return flightModes[0];
	if (controlMode <= 0.75) return flightModes[1];
	if (controlMode > 0.75) return flightModes[2];
	return -1; // NAN
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
