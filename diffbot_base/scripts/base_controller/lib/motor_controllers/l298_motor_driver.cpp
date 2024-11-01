#include <Arduino.h>  // Required for Arduino functions
#include "motor_controller_interface.h"
#include "l298_motor_driver.h"

diffbot::L298MotorController::L298MotorController(uint8_t in1, uint8_t in2, uint8_t enable_pin)
    : in1_(in1), in2_(in2), enable_pin_(enable_pin) {}

void diffbot::L298MotorController::begin() {
    pinMode(in1_, OUTPUT);
    pinMode(in2_, OUTPUT);
    pinMode(enable_pin_, OUTPUT);
}

void diffbot::L298MotorController::setSpeed(int value) {
    if (value > 0) {
        digitalWrite(in1_, HIGH);
        digitalWrite(in2_, LOW);
        analogWrite(enable_pin_, value);
    } else if (value < 0) {
        digitalWrite(in1_, LOW);
        digitalWrite(in2_, HIGH);
        analogWrite(enable_pin_, -value); // Convert to positive value
    } else {
        digitalWrite(in1_, LOW);
        digitalWrite(in2_, LOW);
        analogWrite(enable_pin_, 0);
    }
}
