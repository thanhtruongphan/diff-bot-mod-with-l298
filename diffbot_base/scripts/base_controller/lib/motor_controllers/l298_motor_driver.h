// // #ifndef L298_MOTOR_CONTROLLER_H
// // #define L298_MOTOR_CONTROLLER_H

// // #include <Arduino.h>
// // #include "diffbot_base_config.h"
// // #include "motor_controller_interface.h"

// // namespace diffbot {
// //     class L298MotorController : public MotorControllerIntf<void> {
// //     public:

// //         L298MotorController(uint8_t in1, uint8_t in2, uint8_t enable_pin);

// //         void begin();
// //         void setSpeed(int value) override;

// //     private:
// //         uint8_t in1_;
// //         uint8_t in2_;
// //         uint8_t enable_pin_;
// //     };

// // }

// // #endif // L298_MOTOR_CONTROLLER_H


// #ifndef MOTOR_CONTROLLER_INTERFACE_H
// #define MOTOR_CONTROLLER_INTERFACE_H

// namespace diffbot {

//     class MotorControllerIntf {
//     public:
//         virtual ~MotorControllerIntf() = default;

//         /** \brief Virtual function to set motor speed
//          * 
//          * To be overridden by derived motor controllers.
//          * 
//          * \param value Speed value, where positive values set forward, negative sets reverse, and zero stops the motor.
//          */
//         virtual void setSpeed(int value) = 0;
//     };

// }

// #endif // MOTOR_CONTROLLER_INTERFACE_H


#ifndef L298_MOTOR_DRIVER_H
#define L298_MOTOR_DRIVER_H

#include <Arduino.h>  // Required for Arduino functions
#include "motor_controller_interface.h"

namespace diffbot {

class L298MotorController : public MotorControllerIntf {
public:
    L298MotorController(uint8_t in1, uint8_t in2, uint8_t enable_pin);
    void begin();
    void setSpeed(int value) override;

private:
    uint8_t in1_, in2_, enable_pin_;
};

} // namespace diffbot

#endif // L298_MOTOR_DRIVER_H
