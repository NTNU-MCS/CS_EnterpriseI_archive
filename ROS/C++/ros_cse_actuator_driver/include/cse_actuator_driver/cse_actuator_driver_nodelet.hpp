#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Joy.h>
#include <stdio.h>
// #include <PiPCA9685/PCA9685.h>
#include <vector>
#include <cmath>
#include "cse_actuator_driver/cse_actuator_driver.hpp"

#include <math.h>

namespace cse_actuator_driver
{

class CSEActuatorDriverNodelet : public nodelet::Nodelet
{
public:
    CSEActuatorDriverNodelet();
private:
    virtual void onInit();

    void inputMsgCb(const std_msgs::Float64MultiArray &msg);
    void joyMsgCb(const sensor_msgs::Joy &msg);
    void timerCb(const ros::TimerEvent &event);

    ros::NodeHandle nh_, private_nh_;
    ros::Subscriber pwm_sub_;
    ros::Subscriber joy_sub_;
    ros::Timer timer_;

    CSEActuatorDriver cse_actuator_driver_;
    std::vector<double> control_inputs_;

};

} // namespace cse_actuator_driver