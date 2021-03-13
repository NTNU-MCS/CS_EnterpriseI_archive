#ifndef RPY_PWM_HAT_PCA9685_H
#define RPY_PWM_HAT_PCA9685_H
// #define PCA9685_PRESCALE_MIN 3   /**< minimum prescale value */
// #define PCA9685_PRESCALE_MAX 255 /**< maximum prescale value */
// #define FREQUENCY_OSCILLATOR 25000000 /**< Int. osc. frequency in datasheet */
// constexpr uint16_t PCA9685_PRESCALE_MIN = 3;
// constexpr uint16_t PCA9685_PRESCALE_MAX = 255;
// constexpr uint16_t FREQUENCY_OSCILLATOR = 25000000;

#include <string>

class PCA9685 {
public:
    explicit PCA9685(const std::string &device = "/dev/i2c-1", int address = 0x40);
    // explicit PCA9685(const std::string &device = "/dev/i2c-1", int address = 0x5F);

    void set_pwm_freq(double freq_hz);

    void set_pwm(int channel, uint16_t on, uint16_t off);

    void set_all_pwm(uint16_t on, uint16_t off);

    void set_pwm_ms(int channel, double ms);

    void set_pwm_us(int channel, double us);

private:
    int i2c_fd;
    
    double frequency = 50.0;
    uint16_t prescale_ = 121;

    void check_ret(int ret, std::string msg = "");

};

#endif //RPY_PWM_HAT_PCA9685_H
