#include "TOF.h"

TOF& TOF::getInstance(uint8_t ball_radius) {
    static TOF _instance;
    if (_instanceTofCreated == false) {
        _instance.setBallRadius(ball_radius);
        _instanceTofCreated = true;
    }

    return _instance;
}

void TOF::setOffset(void) {

}

void TOF::startMeasurement(void) {
    sensor_obj.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
    sensor_obj.writeReg(0x80, 0x01);
    sensor_obj.writeReg(0xFF, 0x01);
    sensor_obj.writeReg(0x00, 0x00);
    sensor_obj.writeReg(0x91, _stop_variable);
    sensor_obj.writeReg(0x00, 0x01);
    sensor_obj.writeReg(0xFF, 0x00);
    sensor_obj.writeReg(0x80, 0x00);
    sensor_obj.writeReg(VL53L0X::SYSRANGE_START, 0x01);
}

void TOF::setBallRadius(uint8_t ball_radius) {
    _ball_radius = ball_radius;
}

uint8_t TOF::getBallRadius(void) const {
    return _ball_radius;
}

uint16_t TOF::getDistance(void) {
    static uint8_t i = 0;

    if (i < 40) {
        uint8_t interrupt_state = sensor_obj.readReg(VL53L0X::RESULT_INTERRUPT_STATUS);

        Serial.println(String(i * 10) + ": " + String(interrupt_state, BIN));


        if ((interrupt_state & 0x07) == 0x04) {     // interrupt_state & 0000 0111 (bitmask) -> compare with interrupt state bit 0000 0100
            uint16_t distance = sensor_obj.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);

            _distance = distance;
            Serial.println("Distance: " + String(_distance));

            startMeasurement();
        }
        i++;
    }
    else while (true) {}

    return _distance;
}

bool TOF::beginTof(void) {
    Wire.begin();
    Serial.println("start");
    //sensor_obj.setTimeout(500);

    if (sensor_obj.init()) {
        Serial.println("Start messung");
        sensor_obj.setMeasurementTimingBudget(49 * 1000U); // 49 ms timing budget

        // Stop var: 111011
        _stop_variable = sensor_obj.readReg(0x91);
        Serial.println(_stop_variable, BIN);
        stepper.setDegreeTarget(-20.0F);
        startMeasurement();

        while (true) {
            getDistance();
            delay(10);
        }

        //setOffset();
        _distance = 0;

        stepper.setDegreeTarget(0.0F);
        while(stepper.getState()) {} // wait until engine is running

        return true;
    }
    else {
        Serial.println("Init fehlgeschlagen!");
        while(1);
    }

    return false;
}

float TOF::getDistanceSimulink(void) {
    return static_cast<float>(getDistance() / 1000.0F);
}