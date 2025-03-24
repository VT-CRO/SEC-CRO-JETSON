#ifndef CROBOT_DEAD_WHEEL_ODOM_HPP
#define CROBOT_DEAD_WHEEL_ODOM_HPP


class DeadWheelOdom
{
    public:

    double enc_left = 0;
    double enc_right = 0;
    double enc_center = 0;

    DeadWheelOdom() = default;

};


#endif // DIFFDRIVE_ARDUINO_WHEEL_HPP