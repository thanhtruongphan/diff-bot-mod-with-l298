#include <ros.h>

#include "diffbot_base_config.h"
#include "base_controller.h"
#include "l298_motor_driver.h"


ros::NodeHandle nh;

using namespace diffbot;

L298MotorController motor_controller_right = L298MotorController(MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_LEFT_EN);
L298MotorController motor_controller_left = L298MotorController(MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, MOTOR_RIGHT_EN);
BaseController<L298MotorController> base_controller(nh, &motor_controller_left, &motor_controller_right);


void setup()
{
    nh.loginfo("Initialize DiffBot Motor Controllers");
    motor_controller_left.begin();
    motor_controller_right.begin();
    nh.loginfo("Setup finished");
    
    base_controller.setup();
    base_controller.init();


}


void loop()
{
    static bool imu_is_initialized;

    // The main control loop for the base_conroller.
    // This block drives the robot based on a defined control rate
    ros::Duration command_dt = nh.now() - base_controller.lastUpdateTime().control;
    if (command_dt.toSec() >= ros::Duration(1.0 / base_controller.publishRate().control_, 0).toSec())
    {
        base_controller.read();
        base_controller.write();
        base_controller.lastUpdateTime().control = nh.now();
    }

    // This block stops the motor when no wheel command is received
    // from the high level hardware_interface::RobotHW
    command_dt = nh.now() - base_controller.lastUpdateTime().command_received;
    if (command_dt.toSec() >= ros::Duration(E_STOP_COMMAND_RECEIVED_DURATION, 0).toSec())
    {
        nh.logwarn("Emergency STOP");
        base_controller.eStop();
    }

    // This block publishes the IMU data based on a defined imu rate
    ros::Duration imu_dt = nh.now() - base_controller.lastUpdateTime().imu;
    if (imu_dt.toSec() >= base_controller.publishRate().period().imu_)
    {
        // Sanity check if the IMU is connected
        if (!imu_is_initialized)
        {
            //imu_is_initialized = initIMU();
            if(imu_is_initialized)
                nh.loginfo("IMU Initialized");
            else
                nh.logfatal("IMU failed to initialize. Check your IMU connection.");
        }
        else
        {
            //publishIMU();
        }
        base_controller.lastUpdateTime().imu = nh.now();
    }

    // This block displays the encoder readings. change DEBUG to 0 if you don't want to display
    if(base_controller.debug())
    {
        ros::Duration debug_dt = nh.now() - base_controller.lastUpdateTime().debug;
        if (debug_dt.toSec() >= base_controller.publishRate().period().debug_)
        {
            base_controller.printDebug();
            base_controller.lastUpdateTime().debug = nh.now();
        }
    }
    // Call all the callbacks waiting to be called
    nh.spinOnce();
}



// #include <Arduino.h>

// #include "diffbot_base_config.h"
// // #include "base_controller.h"
// #include "l298_motor_driver.h"


// using namespace diffbot;

// L298MotorController motor_controller_right = L298MotorController(MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, MOTOR_LEFT_EN);
// L298MotorController motor_controller_left = L298MotorController(MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, MOTOR_RIGHT_EN);

// // Define motor control pins for Motor 1
// const int motor1_in1 = 21;
// const int motor1_in2 = 20;
// const int motor1_en = 1; // PWM pin for Motor 1

// // Define motor control pins for Motor 2
// const int motor2_in3 = 17;
// const int motor2_in4 = 16;
// const int motor2_en = 2; // PWM pin for Motor 2

// void setup() {
//     // Set motor pins as outputs
//     pinMode(motor1_in1, OUTPUT);
//     pinMode(motor1_in2, OUTPUT);
//     pinMode(motor1_en, OUTPUT);
    
//     pinMode(motor2_in3, OUTPUT);
//     pinMode(motor2_in4, OUTPUT);
//     pinMode(motor2_en, OUTPUT);
// }
// // Function to control motor speed and direction
// // value range: -255 to 255 (negative for reverse, positive for forward)
// void setMotorSpeed(int in1, int in2, int en, int speed) {
//     if (speed > 0) {
//         digitalWrite(in1, HIGH);
//         digitalWrite(in2, LOW);
//         analogWrite(en, speed); // Set speed (0-255)
//     } else if (speed < 0) {
//         digitalWrite(in1, LOW);
//         digitalWrite(in2, HIGH);
//         analogWrite(en, -speed); // Set speed in reverse direction (0-255)
//     } else {
//         digitalWrite(in1, LOW);
//         digitalWrite(in2, LOW);
//         analogWrite(en, 0); // Stop motor
//     }
// }

// void loop() {
//     // // Example control: Spin both motors forward at 75% speed
//     // setMotorSpeed(motor1_in1, motor1_in2, motor1_en, 0); // 75% of 255
//     // setMotorSpeed(motor2_in3, motor2_in4, motor2_en, 0);
    
//     // delay(2000); // Run forward for 2 seconds

//     // // Stop both motors
//     // setMotorSpeed(motor1_in1, motor1_in2, motor1_en, 0);
//     // setMotorSpeed(motor2_in3, motor2_in4, motor2_en, 0);

//     // delay(1000); // Stop for 1 second

//     // // Spin both motors backward at 50% speed
//     // setMotorSpeed(motor1_in1, motor1_in2, motor1_en, 0); // -50% of 255
//     // setMotorSpeed(motor2_in3, motor2_in4, motor2_en, 0);
    
//     // delay(2000); // Run backward for 2 seconds

//     // // Stop both motors
//     // setMotorSpeed(motor1_in1, motor1_in2, motor1_en, 0);
//     // setMotorSpeed(motor2_in3, motor2_in4, motor2_en, 0);

//     // delay(10000); // Stop for 1 second

//     motor_controller_left.setSpeed(120);
//     motor_controller_right.setSpeed(100);
//     delay(2000);

//     motor_controller_left.setSpeed(0);
//     motor_controller_right.setSpeed(0);
//     delay(2000);
//     motor_controller_left.setSpeed(250);
//     motor_controller_right.setSpeed(250);
//     delay(2000);
//     motor_controller_left.setSpeed(0);
//     motor_controller_right.setSpeed(0);
//     delay(20000);
// }

