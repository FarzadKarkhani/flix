// Repository: https://github.com/okalachev/flix

// Work with the downward-facing distance sensor (rangefinder)

#include <Wire.h>
#include <VL53L0X.h>
#include "util.h"

int rangeModel = -1; // -1 - disabled, 1 - VL53L0X
int rangeSdaPin = SDA, rangeSclPin = SCL;
int rangeBudget = 33000; // measurement timing budget, us (lower is faster but noisier)
float rangeMax = 1.2; // maximum valid distance, m
float rangeOffset = 0; // sensor reading when the drone is on the ground, m

float range = NAN; // measured distance to the surface along the sensor axis, m
float rangeTime = NAN; // time of the last valid measurement
int rangeStatus = -1; // last raw device status

VL53L0X vl53l0x;
bool rangeReady = false;

void setupRange() {
	if (rangeModel < 0) return;
	print("Setup range sensor\n");
	rangeReady = false;

	if (rangeModel != 1) {
		print("Unknown range sensor model: %d\n", rangeModel);
		return;
	}

	// Share the bus with the IMU if the pins are the same, otherwise use a separate I2C controller
	TwoWire *wire = &Wire;
	if (imuBus == 1 && rangeSdaPin == imuSdaPin && rangeSclPin == imuSclPin) {
		// the bus is already started by the IMU
	} else {
		if (imuBus == 1) wire = &Wire1; // Wire is busy with the IMU on other pins
		wire->begin(rangeSdaPin, rangeSclPin, 400000);
	}

	vl53l0x.setBus(wire);
	vl53l0x.setTimeout(500);
	if (!vl53l0x.init()) {
		print("Range sensor init failed, check wiring and RNG_PIN_SDA/RNG_PIN_SCL\n");
		return;
	}
	if (!vl53l0x.setMeasurementTimingBudget(rangeBudget)) {
		print("Invalid RNG_BUDGET: %d\n", rangeBudget);
	}
	vl53l0x.setTimeout(0); // reads in the loop never block
	vl53l0x.startContinuous();
	rangeReady = true;
}

void readRange() {
	if (!rangeReady) return;

	static Rate rate(100); // poll faster than the sensor measures
	if (!rate) return;

	if ((vl53l0x.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0) return; // no new measurement yet

	uint8_t buf[12];
	vl53l0x.readMulti(VL53L0X::RESULT_RANGE_STATUS, buf, sizeof(buf));
	vl53l0x.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);

	rangeStatus = (buf[0] >> 3) & 0x0F;
	uint16_t mm = (uint16_t)buf[10] << 8 | buf[11];
	float distance = mm / 1000.0f - rangeOffset;

	if (rangeStatus == 11 && mm < 8190 && distance <= rangeMax) { // valid measurement
		range = max(distance, 0.0f);
	} else if (rangeStatus == 8 || rangeStatus == 10) { // closer than the minimal distance, the drone is on the ground
		range = 0;
	} else { // out of range or no signal
		range = NAN;
		return;
	}
	rangeTime = t;
}

void printRangeInfo() {
	if (rangeModel < 0) {
		print("Range sensor disabled, set RNG_MODEL\n");
		return;
	}
	print("ready: %d\n", rangeReady);
	print("status: %d\n", rangeStatus);
	print("range: %.3f m\n", range);
	print("last valid: %.2f s ago\n", t - rangeTime);
	print("altitude: %.3f m\n", altitude);
	print("vertical speed: %.3f m/s\n", verticalSpeed);
	print("hover thrust: %.3f (learned %.3f)\n", hoverThrust, hoverEstimate);
}
