// Hardware layer for the ME130 pendulum, shared by the ROS 2 nodes.
//
// Each class owns one piece of the BD65496MUV / MPU6050 / encoder rig, and the
// nodes are split so the pieces never overlap:
//     motor_node    -> HardwarePwm (pwm0) + MotorGpio (PS=24, DIR=25)
//     encoder_node  -> QuadratureEncoder  (A=17, B=27)
//     imu_node      -> Mpu6050            (/dev/i2c-1)
// Two processes may hold different GPIO lines on the same chip, so this split
// is safe; two processes holding the SAME line is not, which is why the motor
// only ever moves through motor_node.
#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

struct gpiod_chip;
struct gpiod_line;

namespace me130
{

// Wiring, matching the BD65496MUV in EN/IN mode.
inline constexpr int kGpioPs = 24;      // HIGH = driver active, LOW = coast
inline constexpr int kGpioDir = 25;     // INB
inline constexpr int kGpioEncA = 17;
inline constexpr int kGpioEncB = 27;
inline constexpr int kPwmChannel = 0;   // INA, GPIO18
inline constexpr int kPwmFreqHz = 20000;

inline constexpr double kGearRatio = 25.0;
inline constexpr double kEncoderCountsPerMotorRev = 20.0;
inline constexpr double kCountsPerOutputRev = kEncoderCountsPerMotorRev * kGearRatio;

// Hardware PWM through sysfs, holding the duty_cycle fd open so a 500 Hz loop
// does not reopen a file every tick.
class HardwarePwm
{
public:
    HardwarePwm(int channel, int frequency_hz);
    ~HardwarePwm();
    HardwarePwm(const HardwarePwm &) = delete;
    HardwarePwm &operator=(const HardwarePwm &) = delete;

    // duty in [0, 1]; magnitude only, direction comes from MotorGpio.
    void setDuty(double duty);

private:
    int channel_;
    int duty_fd_{-1};
    long long period_ns_;
    std::string pwm_path_;
};

// PS (enable/coast) and DIR (direction) lines.
class MotorGpio
{
public:
    MotorGpio(int ps_gpio, int dir_gpio);
    ~MotorGpio();
    MotorGpio(const MotorGpio &) = delete;
    MotorGpio &operator=(const MotorGpio &) = delete;

    void enableDriver();
    void coast();
    // dir > 0 drives the line LOW. Keep this mapping: the characterization
    // data and the controller sign convention are both built on it.
    void setDirection(int dir);

private:
    gpiod_chip *chip_{nullptr};
    gpiod_line *ps_line_{nullptr};
    gpiod_line *dir_line_{nullptr};
};

// Quadrature decode on its own thread, using the full state-transition table.
class QuadratureEncoder
{
public:
    QuadratureEncoder(int gpio_a, int gpio_b);
    ~QuadratureEncoder();
    QuadratureEncoder(const QuadratureEncoder &) = delete;
    QuadratureEncoder &operator=(const QuadratureEncoder &) = delete;

    long long counts() const { return counts_.load(); }

private:
    void run();

    gpiod_chip *chip_{nullptr};
    gpiod_line *a_line_{nullptr};
    gpiod_line *b_line_{nullptr};
    std::atomic<long long> counts_{0};
    std::atomic<bool> running_{true};
    std::thread thread_;
};

// GY-521 / MPU6050 over /dev/i2c-1.
class Mpu6050
{
public:
    explicit Mpu6050(const std::string &device = "/dev/i2c-1", int address = 0x68);
    ~Mpu6050();
    Mpu6050(const Mpu6050 &) = delete;
    Mpu6050 &operator=(const Mpu6050 &) = delete;

    // Raw accelerometer counts in the tilt plane, and gyro Z in deg/s.
    bool read(float &acc_x, float &acc_y, float &gyro_z_dps);

    // Mean gyro Z with the rod held still, in rad/s.
    double calibrateGyroBiasRadPerSec(int samples = 500);

private:
    void writeReg(std::uint8_t reg, std::uint8_t value);

    int fd_{-1};
    int address_;
};

}  // namespace me130
