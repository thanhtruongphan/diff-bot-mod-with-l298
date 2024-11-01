#ifndef MOTOR_CONTROLLER_INTF_H
#define MOTOR_CONTROLLER_INTF_H

namespace diffbot {
    class MotorControllerIntf
    {
    public:

        virtual void setSpeed(int value) = 0;
    };
}

#endif // MOTOR_CONTROLLER_INTF_H