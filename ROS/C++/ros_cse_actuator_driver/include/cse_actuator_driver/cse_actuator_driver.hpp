#include "cse_actuator_driver/PCA9685.h"
#include "cse_actuator_driver/interp.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

using Vector5d = Eigen::Matrix<double, 5, 1>;
using Matrix5d = Eigen::Matrix<double, 5, 5>;

constexpr double BT_MIN       = -1;
constexpr double BT_MAX       =  1;
constexpr double BT_GAIN      =  1.25;
constexpr double BT_NEG_OFFSET=  6.3;
constexpr double BT_NEG_GAIN  =  0.35;
constexpr double BT_POS_OFFSET=  6.8;
constexpr double BT_POS_GAIN  =  0.735;
constexpr double VSP_MAX      =  1;
constexpr double VSP_MIN      =  0;
constexpr double VSP_OMEGA_GAIN =  1.71;
constexpr double VSP_U_GAIN   =  0.801;
constexpr double VSP_U_OFFSET =  0.3;
constexpr double VSP_OMEGA    = 0.235;
constexpr double VSP_ZERO_PWM =  5.2;
constexpr double BT_ZERO_PWM  =  6.5;
 

namespace cse_actuator_driver
{

class CSEActuatorDriver
{
public:
    CSEActuatorDriver();
    ~CSEActuatorDriver() = default;

    bool is_running_;

    void set_pwm();
    void process_control_inputs(const std::vector<double> inputs);
    uint32_t duty_cycle_to_pwm(double duty_cycle);

    std::vector<double> get_duty();

    double clip(double n, double lower, double upper);

private:
    double frequency_;
    std::vector<uint32_t> pwm_;
    std::vector<double> saturated_inputs_;
    std::vector<double> duty_cycle_;
 
    PCA9685 pca_;
    BilinearInterpolator<5> interp_servo1_;
    BilinearInterpolator<5> interp_servo2_;
    BilinearInterpolator<5> interp_servo3_;
    BilinearInterpolator<5> interp_servo4_;


};




}