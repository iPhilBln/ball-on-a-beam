#include "TOF.h"

TOF& TOF::getInstance(uint8_t ball_radius) {
    static TOF _instance;
    if (_instanceTofCreated == false) {
        _instance.setBallRadius(ball_radius);
        _instanceTofCreated = true;
    }

    return _instance;
}

bool TOF::setOffset(void) {
    uint16_t  counter      = 0;
    uint16_t  distance     = 0;
    uint16_t  distance_old = 0;

    uint16_t      timeout   = 10000U;
    unsigned long timestamp = millis();

    while (counter < 50U) {
        if (millis() - timestamp > timeout) {
            Serial.println("Initialization for ToF sensor failed!");
            return false;
        }

        unsigned long cycle_time = millis() + 60U; // 75ms per measurement cycle

        distance = getDistancePure();
        if (distance != 500) {
            (0U < distance && distance < 100U) && (distance_old - 2 <= distance || distance <= distance_old + 2) ? counter++ : counter = 0;
            distance_old = distance;
        }
        while (millis() < cycle_time) {}
    }

    _offset = static_cast<uint8_t>(distance);
    _distance = _ball_radius;

    Serial.println("Offset ToF sensor: " + String(_offset));
    return true;
}

void TOF::startMeasurement(void) {
    sensor.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01);
    sensor.writeReg(0x80, 0x01);
    sensor.writeReg(0xFF, 0x01);
    sensor.writeReg(0x00, 0x00);
    sensor.writeReg(0x91, _stop_variable);
    sensor.writeReg(0x00, 0x01);
    sensor.writeReg(0xFF, 0x00);
    sensor.writeReg(0x80, 0x00);
    sensor.writeReg(VL53L0X::SYSRANGE_START, 0x01);
}

/*      SETTER      */

void TOF::setBallRadius(uint8_t ball_radius) {
    _ball_radius = ball_radius;
}

/*      GETTER      */

uint8_t TOF::getBallRadius(void) const {
    return _ball_radius;
}

uint8_t TOF::getOffset(void) const {
    return _offset;
}

/*      FUNCTIONALITY       */

uint16_t TOF::getDistancePure() {
    uint8_t interrupt_state = sensor.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07; // interrupt_state & 0000 0111 (bitmask) -> compare with interrupt state bit 0000 0100
    if (interrupt_state == 0x04) {           
        uint16_t distance_local = sensor.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
        startMeasurement();
        return distance_local;
    } 

    return 500;
}

uint16_t TOF::getDistance(void) {
    uint16_t distance_local = getDistancePure();

    if (distance_local >= _offset) {
        distance_local   = _ball_radius + (distance_local - _offset);
        (_ball_radius <= distance_local && distance_local <= 450U - _ball_radius) ? _distance = distance_local : _distance;
    }
    else {
        return 500;
    }

    return _distance;
}

float TOF::getDistanceSimulink(void) {
    static uint8_t counter = 0;

    if (counter == 0U) {
        uint16_t distance_local = getDistancePure();

        if (distance_local == 500) {
            if (_measurement_failed < 20) {
                _measurement_failed++;
            }
            else return 0xFFFF;

            return _distance / 1000.0F;
        }

        _measurement_failed = 0;

        if (distance_local >= _offset) {
            distance_local = _ball_radius + (distance_local - _offset);
            (_ball_radius <= distance_local && distance_local <= 450U - _ball_radius) ? _distance = distance_local : _distance;
        }
    }

    counter < 5U ? counter++ : counter = 0;    // count 7 times -> 52.5ms measurement cycle
    return _distance / 1000.0F;
}

bool TOF::beginTof(void) {
    Wire.begin();
    Serial.println("Begin ToF sensor calibration.");
    
    uint16_t      timeout   = 2000U;
    unsigned long timestamp = millis();

    while (sensor.init() == false) {
        if (millis() - timestamp > timeout) {
            Serial.println("Initialization for ToF sensor failed!");
            return false;
        }
    }

    sensor.setMeasurementTimingBudget(49U * 1000U); // 50 ms timing budget
    _stop_variable = sensor.readReg(0x91);
    _measurement_failed = 0;

    startMeasurement();
    return setOffset();
}