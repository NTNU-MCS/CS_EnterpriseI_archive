#include "cse_actuator_driver/cse_actuator_driver.hpp"

namespace cse_actuator_driver
{
    // CSEActuatorDriver::CSEActuatorDriver(const Matrix5d &F, const Vector5d &breakpoints) 
    CSEActuatorDriver::CSEActuatorDriver() 
    : frequency_{50.0},
    // initialize pwm 
      pwm_{0,0,0,0,0,0,0},
      saturated_inputs_{0,0,0,0,0},
      duty_cycle_{BT_ZERO_PWM, VSP_ZERO_PWM, VSP_ZERO_PWM, 4.4, 6.15, 5.0, 5.2},
      pca_(),
      interp_servo1_(),
      interp_servo2_(),
      interp_servo3_(),
      interp_servo4_(),
      is_running_{false}
    {
        pca_.set_pwm_freq(frequency_);

        Eigen::Matrix<double, 5, 5> F1;
        Eigen::Matrix<double, 5, 5> F2;
        Eigen::Matrix<double, 5, 5> F3;
        Eigen::Matrix<double, 5, 5> F4;
        Eigen::Matrix<double, 5, 1> breakpoints;
        breakpoints << -1, -sqrt(2)/2, 0, sqrt(2)/2, 1;

        // See CSEI handbook and Matlab code, this is the servo_mapping array derived from the measurements array
        F1 <<   3.6000,    3.8490,    4.4500,    4.9096,    5.1000,
                3.5414,    3.8343,    4.4354,    4.8950,    5.1586,
                3.4000,    3.6929,    4.4000,    5.0364,    5.3000,
                3.5626,    3.8555,    4.3646,    4.9303,    5.1939,
                3.6300,    3.8409,    4.3500,    4.9157,    5.1500;
        F2 <<   7.0000,    7.1611,    7.5500,    7.1611,    7.0000,
                6.7803,    6.7510,    7.1399,    6.7510,    6.8536,
                6.2500,    6.2207,    6.1500,    6.3975,    6.5000,
                5.5782,    5.5490,    5.5490,    5.9025,    6.0050,
                5.3000,    5.3000,    5.3000,    5.6536,    5.8000;
        F3 <<   5.8000,    5.8732,    6.0500,    5.7318,    5.6000,
                5.5364,    5.5657,    5.7425,    5.4243,    5.4243,
                4.9000,    4.9293,    5.0000,    5.0000,    5.0000,
                4.2990,    4.3282,    4.2222,    4.4697,    4.4697,
                4.0500,    4.0061,    3.9000,    4.1475,    4.2500;
        F4 <<   5.6500,    5.4596,    5.0000,    4.5757,    4.4000,
                5.8697,    5.5182,    5.0586,    4.6343,    4.3561,
                6.4000,    6.0485,    5.2000,    4.5282,    4.2500,
                6.2232,    5.8718,    5.4121,    4.9525,    4.6743,
                6.1500,    5.9596,    5.5000,    5.0404,    4.8500;

        
        interp_servo1_.init(F1, breakpoints, breakpoints);
        interp_servo2_.init(F2, breakpoints, breakpoints);
        interp_servo3_.init(F3, breakpoints, breakpoints);
        interp_servo4_.init(F4, breakpoints, breakpoints);

        for (int i = 0; i < 7; i++)
            {
                pwm_[i] = duty_cycle_to_pwm(duty_cycle_[i]);
            }
        set_pwm();
    }

    //converts from percentage to microseconds
    uint32_t CSEActuatorDriver::duty_cycle_to_pwm(double duty_cycle) {
        uint32_t pwm =  std::round(duty_cycle * 1e-2 / (1e-6*frequency_)); 
        return pwm;
    }

    void CSEActuatorDriver::set_pwm()
    {
        for (int i = 0; i < 7; i++)
        {
            if (pwm_[i] <= 0)
            {}
            else
            {
                pca_.set_pwm_us(i, pwm_[i]); // send pwm in microseconds
            }
        }
    }

    void CSEActuatorDriver::process_control_inputs(const std::vector<double> inputs)
    {
        if (is_running_)
        {
            // scale bow thruster input
            saturated_inputs_[0] = clip(BT_GAIN * inputs[0], BT_MIN, BT_MAX);
            // VSP inputs
            saturated_inputs_[1] = 0.95 * clip(VSP_U_GAIN * inputs[1] + VSP_U_OFFSET, VSP_MIN, VSP_MAX );
            saturated_inputs_[2] = 0.95 * clip(VSP_U_GAIN * inputs[2] + VSP_U_OFFSET, VSP_MIN, VSP_MAX );
            // VSP angles
            saturated_inputs_[3] = inputs[3];
            saturated_inputs_[4] = inputs[4];

            // bow thruster
            if (-saturated_inputs_[0] > 0.01)
            {
                duty_cycle_[0] = BT_POS_OFFSET - BT_POS_GAIN * saturated_inputs_[0];
            }
            else if (-saturated_inputs_[0] < -0.01)
            {
                duty_cycle_[0] = BT_NEG_OFFSET - BT_NEG_GAIN * saturated_inputs_[0];
            }
            else
            {
                duty_cycle_[0] = BT_ZERO_PWM;
            }
            
            // omega_vsp 
            duty_cycle_[1] = VSP_ZERO_PWM + VSP_OMEGA * VSP_OMEGA_GAIN;
            duty_cycle_[2] = VSP_ZERO_PWM + VSP_OMEGA * VSP_OMEGA_GAIN; 

            // VSP servos
            duty_cycle_[3] = interp_servo1_.interp( -saturated_inputs_[1] * cos(saturated_inputs_[3]), saturated_inputs_[1] * sin(saturated_inputs_[3]) );
            duty_cycle_[4] = interp_servo2_.interp( -saturated_inputs_[1] * cos(saturated_inputs_[3]), saturated_inputs_[1] * sin(saturated_inputs_[3]) );
            duty_cycle_[5] = interp_servo3_.interp( -saturated_inputs_[2] * cos(saturated_inputs_[4]), saturated_inputs_[2] * sin(saturated_inputs_[4]) );
            duty_cycle_[6] = interp_servo4_.interp( -saturated_inputs_[2] * cos(saturated_inputs_[4]), saturated_inputs_[2] * sin(saturated_inputs_[4]) );
        }        
        else
        {
            duty_cycle_ = {BT_ZERO_PWM, VSP_ZERO_PWM, VSP_ZERO_PWM, 4.4, 6.15, 5.0, 5.2};
        }
        
        for (int i = 0; i < 7; i++)
        {
            pwm_[i] = duty_cycle_to_pwm(duty_cycle_[i]);
        }

        // Eigen::Matrix<double, 7, 1> asdf;
        // asdf << inputs[0], VSP_ZERO_PWM + 0 * VSP_OMEGA_GAIN, VSP_ZERO_PWM + 0 * VSP_OMEGA_GAIN, inputs[1], inputs[2], inputs[3], inputs[4];

        // for (int i = 0; i < 7; i++)
        // {
        //     pwm_[i] = duty_cycle_to_pwm(asdf(i));
        // }
        

    }

    std::vector<double> CSEActuatorDriver::get_duty()
    {
        return duty_cycle_;
    }

    double CSEActuatorDriver::clip(double n, double lower, double upper) 
    {
        return std::max(lower, std::min(n, upper));
    }

}