// Copyright (c) 2023 Oleg Kalachev <okalachev@gmail.com>
// Repository: https://github.com/okalachev/flix

// Main firmware file

#include "vector.h"
#include "quaternion.h"
#include "util.h"

extern float t, dt;
extern float controlRoll, controlPitch, controlYaw, controlThrottle, controlMode;
extern Vector gyro, acc;
extern Vector rates;
extern Quaternion attitude;
extern bool landed;
extern float range, rangeTime;
extern float altitude, verticalSpeed;
extern float motors[4];

void setup() {
	setupConsole();
	print("Initializing Flix\n");
	setupParameters();
	setupPower();
	setupLED();
	setLED(true);
	setupMotors();
	setupWiFi();
	setupIMU();
	setupRange();
	setupRC();
	setLED(false);
	print("Initializing complete\n");
}

void loop() {
	readIMU();
	step();
	readRange();
	readRC();
	estimate();
	control();
	sendMotors();
	handleConsole();
	processMavlink();
	readVoltage();
	logData();
	syncParameters();
}
