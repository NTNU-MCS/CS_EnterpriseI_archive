#include "cse_actuator_driver/cse_actuator_driver_nodelet.hpp"
#include "std_msgs/Float64MultiArray.h"
#include <pluginlib/class_list_macros.h>


namespace cse_actuator_driver
{

    CSEActuatorDriverNodelet::CSEActuatorDriverNodelet()
        : control_inputs_{0, 0, 0, 0, 0},
        cse_actuator_driver_{}
    {}

    void CSEActuatorDriverNodelet::onInit()
    {
        NODELET_INFO_STREAM("Initializing CSE actuator driver nodelet...");
        nh_ = getNodeHandle();
        private_nh_ = getPrivateNodeHandle();
        pwm_sub_ = nh_.subscribe("CSEI/u", 1, &CSEActuatorDriverNodelet::inputMsgCb, this);
        joy_sub_ = nh_.subscribe("joy", 1, &CSEActuatorDriverNodelet::joyMsgCb, this);
        timer_ = nh_.createTimer(ros::Duration(0.01), &CSEActuatorDriverNodelet::timerCb, this);
        // timer_ = nh_.createTimer(ros::Duration(2), &CSEActuatorDriverNodelet::timerCb, this);
    }

    void CSEActuatorDriverNodelet::inputMsgCb(const std_msgs::Float64MultiArray &msg)
    {
        if (msg.data.size() >= 5)
        {
            for (int i = 0; i < 5; i++)
            {
                control_inputs_[i] = msg.data[i];
            }
            cse_actuator_driver_.process_control_inputs(control_inputs_);
        }
    }

    void CSEActuatorDriverNodelet::joyMsgCb(const sensor_msgs::Joy &msg)
    {
        if (msg.buttons[2] == 1 && cse_actuator_driver_.is_running_)
        {
            NODELET_INFO_STREAM("Disabling controller input..");
            cse_actuator_driver_.is_running_ = false;
        }
        if (msg.buttons[3] == 1 && !cse_actuator_driver_.is_running_)
        {
                NODELET_INFO_STREAM("Enabling controller input..");
                cse_actuator_driver_.is_running_ = true;
        }
    }

    void CSEActuatorDriverNodelet::timerCb(const ros::TimerEvent &event)
    {
        cse_actuator_driver_.set_pwm();
                // testing
        // std::vector<double> duty_cycle = cse_actuator_driver_.get_duty();
        // NODELET_INFO_STREAM("Duty cycle 1: [" << duty_cycle[0] << "] ");
        // NODELET_INFO_STREAM("Duty cycle 2: [" << duty_cycle[1] << "] ");
        // NODELET_INFO_STREAM("Duty cycle 3: [" << duty_cycle[2] << "] ");
        // NODELET_INFO_STREAM("Duty cycle 4: [" << duty_cycle[3] << "] ");
        // NODELET_INFO_STREAM("Duty cycle 5: [" << duty_cycle[4] << "] ");
        // NODELET_INFO_STREAM("Duty cycle 6: [" << duty_cycle[5] << "] ");
        // NODELET_INFO_STREAM("Duty cycle 7: [" << duty_cycle[6] << "] ");

    }
}
PLUGINLIB_EXPORT_CLASS(cse_actuator_driver::CSEActuatorDriverNodelet, nodelet::Nodelet);
