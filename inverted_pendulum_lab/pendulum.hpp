#pragma once
/* ============================================================================
 * pendulum.hpp -- Hardware layer + assignment interface for the IMU Pendulum Lab
 * ============================================================================
 * Raspberry Pi 4B + GY-521/MPU6050 + BD65496MUV motor driver + 25:1 20D motor
 *
 * You should not need to read or modify this file to complete the assignment.
 * It has two parts:
 *   1. Hardware plumbing: talking to the IMU over I2C, driving the motor via
 *      hardware PWM + DIR, decoding the quadrature encoder, raw terminal
 *      keyboard input. Same hardware layer as the original single-file
 *      program, just factored out so the control loop is easier to read.
 *   2. The interface between controller.cpp (your PID code) and
 *      pendulum_runner.cpp (the harness that calls it) -- at the bottom of
 *      this file.
 * ==========================================================================*/
#include <gpiod.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <algorithm>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

// ---- config: Motor Driver BD65496MUV EN/IN mode, PWM + DIR ----
constexpr int GPIO_PS = 24, GPIO_DIR = 25, GPIO_ENC_A = 17, GPIO_ENC_B = 27;
constexpr int PWM_CHANNEL = 0, PWM_FREQ_HZ = 20000;
constexpr double GEAR_RATIO = 25.0, ENCODER_COUNTS_PER_MOTOR_REV = 20.0;
constexpr double COUNTS_PER_OUTPUT_REV = ENCODER_COUNTS_PER_MOTOR_REV * GEAR_RATIO;
constexpr double PI = 3.14159265358979323846;
constexpr double DEG2RAD = PI / 180.0, RAD2DEG = 180.0 / PI;
constexpr double RAD_PER_COUNT = 2.0 * PI / COUNTS_PER_OUTPUT_REV;

constexpr double LOOP_HZ = 500.0, DT = 1.0 / LOOP_HZ;

// it will arm within +- 5 degrees of the selected zero position
constexpr double ARM_WINDOW = 5.0 * DEG2RAD;

// it will automatically disarm if the pendulum goes 90 degrees off the zero
// position in either direction
constexpr double THETA_MAX = 90.0 * DEG2RAD;
constexpr int IMU_FAIL_MAX = 8;

inline std::atomic<long long> encoder_count{0};
inline std::atomic<bool> running{true};

inline void writeSysfs(const std::string& path, const std::string& value)
{
    std::ofstream f(path);
    if (!f) throw std::runtime_error("Could not open: " + path);
    f << value;
    if (!f) throw std::runtime_error("Could not write to: " + path);
}

// ---- GY-521 / MPU6050 over Raspberry Pi I2C (/dev/i2c-1) ----------------------
class MPU6050
{
public:
    explicit MPU6050(const std::string& dev = "/dev/i2c-1", int address = 0x68)
        : address_(address)
    {
        fd_ = open(dev.c_str(), O_RDWR);
        if (fd_ < 0)
            throw std::runtime_error("Could not open " + dev +
                ". Enable I2C and check that /dev/i2c-1 exists.");

        if (ioctl(fd_, I2C_SLAVE, address_) < 0)
            throw std::runtime_error("Could not select MPU6050 at I2C address 0x68.");

        writeReg(0x6B, 0x00);   // PWR_MGMT_1: wake
        writeReg(0x1B, 0x08);   // GYRO_CONFIG: +/-500 deg/s
        writeReg(0x1C, 0x08);   // ACCEL_CONFIG: +/-4 g
        std::this_thread::sleep_for(50ms);
    }

    ~MPU6050()
    {
        if (fd_ >= 0) close(fd_);
    }

    void writeReg(uint8_t reg, uint8_t value)
    {
        uint8_t buf[2] = {reg, value};
        if (write(fd_, buf, 2) != 2)
            throw std::runtime_error("MPU6050 register write failed.");
    }

    bool read(float& accX, float& accY, float& gyroZ_dps)
    {
        uint8_t reg = 0x3B; // ACCEL_XOUT_H
        if (write(fd_, &reg, 1) != 1) return false;

        uint8_t b[14];
        ssize_t n = ::read(fd_, b, sizeof(b));
        if (n != 14) return false;

        auto s16 = [](uint8_t hi, uint8_t lo) -> int16_t {
            return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
        };

        int16_t ax = s16(b[0], b[1]);
        int16_t ay = s16(b[2], b[3]);
        int16_t gz = s16(b[12], b[13]);

        accX = static_cast<float>(ax);
        accY = static_cast<float>(ay);
        gyroZ_dps = static_cast<float>(gz) / 65.5f; // +/-500 dps
        return true;
    }

    double calibrateGyroBiasRadPerSec(int samples = 500)
    {
        std::cout << "Calibrating gyro bias -- keep rod STILL...\n";
        double sum_dps = 0.0;
        int good = 0;

        for (int i = 0; i < samples; ++i)
        {
            float ax, ay, gz;
            if (read(ax, ay, gz))
            {
                sum_dps += gz;
                ++good;
            }
            std::this_thread::sleep_for(2ms);
        }

        if (good < samples / 2)
            throw std::runtime_error("Too many MPU6050 read failures during gyro calibration.");

        return (sum_dps / good) * DEG2RAD;
    }

private:
    int fd_{-1};
    int address_;
};

// ---- Hardware PWM via sysfs, with a PERSISTENT fd for duty_cycle so a 500Hz
// control loop isn't opening/closing a file 1000x/sec (that adds latency/jitter)
class HardwarePWM
{
public:
    HardwarePWM(int channel, int frequency_hz) : channel_(channel)
    {
        // path to the enabled pwm channel where all the configs will be written
        pwm_path_ = "/sys/class/pwm/pwmchip0/pwm" + std::to_string(channel_);
        period_ns_ = static_cast<long long>(1e9 / frequency_hz);

        if (access(pwm_path_.c_str(), F_OK) != 0)
        {
            writeSysfs("/sys/class/pwm/pwmchip0/export", std::to_string(channel_));
            for (int i = 0; i < 100 && access(pwm_path_.c_str(), F_OK) != 0; ++i)
                std::this_thread::sleep_for(10ms);
        }
        if (access(pwm_path_.c_str(), F_OK) != 0)
            throw std::runtime_error("PWM channel " + std::to_string(channel_) +
                " did not appear. Check dtoverlay=pwm-2chan,... in config.txt.");

        try { writeSysfs(pwm_path_ + "/enable", "0"); } catch (...) {}
        writeSysfs(pwm_path_ + "/period", std::to_string(period_ns_));
        writeSysfs(pwm_path_ + "/duty_cycle", "0");
        try { writeSysfs(pwm_path_ + "/polarity", "normal"); }
        catch (const std::exception& e) {
            std::cerr << "WARNING: could not set polarity=normal on channel "
                      << channel_ << ": " << e.what() << "\n";
        }
        writeSysfs(pwm_path_ + "/enable", "1");

        duty_fd_ = open((pwm_path_ + "/duty_cycle").c_str(), O_WRONLY);
        if (duty_fd_ < 0)
            throw std::runtime_error("Could not open duty_cycle fd for channel " + std::to_string(channel_));

        std::cout << "PWM channel " << channel_ << " initialized: period = " << period_ns_ << " ns\n";
    }

    void setDuty(double duty)
    {
        duty = std::clamp(duty, 0.0, 1.0);
        long long duty_ns = static_cast<long long>(duty * period_ns_);
        if (duty_ns >= period_ns_) duty_ns = period_ns_ - 1;
        std::string s = std::to_string(duty_ns);
        lseek(duty_fd_, 0, SEEK_SET);
        // Check if the duty was written or not
        if (write(duty_fd_, s.c_str(), s.size()) < 0) { /* best-effort; a dropped write for one 2ms tick is not safety-critical here */ }
    }

    ~HardwarePWM()
    {
        if (duty_fd_ >= 0) close(duty_fd_);
        try { writeSysfs(pwm_path_ + "/duty_cycle", "0"); writeSysfs(pwm_path_ + "/enable", "0"); }
        catch (...) {}
    }

private:
    int channel_;
    int duty_fd_{-1};
    long long period_ns_;
    std::string pwm_path_;
};

// ---- GPIO: PS (power-save/enable) + encoder lines (identical to characterization) ----
class GPIOController
{
public:
    GPIOController()
    {
        chip_ = gpiod_chip_open("/dev/gpiochip0");
        if (!chip_) throw std::runtime_error("Could not open /dev/gpiochip0");

        ps_line_ = gpiod_chip_get_line(chip_, GPIO_PS);
        dir_line_ = gpiod_chip_get_line(chip_, GPIO_DIR);
        enc_a_line_ = gpiod_chip_get_line(chip_, GPIO_ENC_A);
        enc_b_line_ = gpiod_chip_get_line(chip_, GPIO_ENC_B);
        if (!ps_line_ || !dir_line_ || !enc_a_line_ || !enc_b_line_)
            throw std::runtime_error("Could not obtain GPIO line.");

        if (gpiod_line_request_output(ps_line_, "motor_ps", 0) < 0)
            throw std::runtime_error("Could not request PS GPIO.");
        if (gpiod_line_request_output(dir_line_, "motor_dir", 0) < 0)
            throw std::runtime_error("Could not request DIR GPIO.");
        if (gpiod_line_request_both_edges_events(enc_a_line_, "encoder_a") < 0)
            throw std::runtime_error("Could not request encoder A.");
        if (gpiod_line_request_both_edges_events(enc_b_line_, "encoder_b") < 0)
            throw std::runtime_error("Could not request encoder B.");
    }

    ~GPIOController() { coast(); if (chip_) gpiod_chip_close(chip_); }

    void enableDriver() { gpiod_line_set_value(ps_line_, 1); }
    void coast() { if (ps_line_) gpiod_line_set_value(ps_line_, 0); }
    void setDirection(int dir) { gpiod_line_set_value(dir_line_, dir > 0 ? 0 : 1); }
    gpiod_line* encoderA() { return enc_a_line_; }
    gpiod_line* encoderB() { return enc_b_line_; }

private:
    gpiod_chip* chip_{nullptr};
    gpiod_line* ps_line_{nullptr};
    gpiod_line* dir_line_{nullptr};
    gpiod_line* enc_a_line_{nullptr};
    gpiod_line* enc_b_line_{nullptr};
};

// ---- Encoder thread: sign-corrected quadrature decode ----
inline void encoderThread(gpiod_line* lineA, gpiod_line* lineB)
{
    pollfd fds[2] = {{gpiod_line_event_get_fd(lineA), POLLIN, 0},
                     {gpiod_line_event_get_fd(lineB), POLLIN, 0}};

    static constexpr int8_t transition_table[16] = {
         0, +1, -1,  0,
        -1,  0,  0, +1,
        +1,  0,  0, -1,
         0, -1, +1,  0
    };

    int a = gpiod_line_get_value(lineA), b = gpiod_line_get_value(lineB);
    int previous_state = (a << 1) | b;

    while (running)
    {
        if (poll(fds, 2, 100) <= 0) continue;
        if (fds[0].revents & POLLIN) { gpiod_line_event e; gpiod_line_event_read(lineA, &e); }
        if (fds[1].revents & POLLIN) { gpiod_line_event e; gpiod_line_event_read(lineB, &e); }

        a = gpiod_line_get_value(lineA); b = gpiod_line_get_value(lineB);
        int current_state = (a << 1) | b;
        encoder_count.fetch_add(transition_table[(previous_state << 2) | current_state]);
        previous_state = current_state;
    }
}

// ---- Motor command: PWM + DIR, BD65496MUV EN/IN mode -------------------------
// PWM/MODE must be tied HIGH (3.3V).
// While PS is HIGH:
//   INA(PWM)=HIGH, INB(DIR)=LOW  -> CW drive
//   INA(PWM)=HIGH, INB(DIR)=HIGH -> CCW drive
//   INA(PWM)=LOW                 -> short brake
// Therefore PWM on INA naturally gives DRIVE <-> SHORT BRAKE
// (slow-decay-style behavior) without needing complementary PWM.
inline void setMotor(GPIOController& gpio, HardwarePWM& pwm, double u)
{
    u = std::clamp(u, -1.0, 1.0);
    double m = std::fabs(u);

    if (m == 0.0)
    {
        pwm.setDuty(0.0);
        gpio.coast();   // PS LOW -> outputs open/coast at exact zero
        return;
    }

    // Drop PWM before changing direction.
    pwm.setDuty(0.0);

    // Preserve the old sign convention:
    // +u previously drove INA=1, INB=0 during the DRIVE portion.
    gpio.setDirection(u > 0.0 ? +1 : -1);

    gpio.enableDriver();
    pwm.setDuty(m);
}

// ---- Terminal raw mode -- ECHO stays ON (unlike the sweep) so typed numeric
// arguments (e.g. "p6.5<Enter>") are visible, matching the Serial-monitor feel. ----
inline termios g_orig_termios;
inline void enableRawMode()
{
    tcgetattr(STDIN_FILENO, &g_orig_termios);
    termios raw = g_orig_termios;
    raw.c_lflag &= ~ICANON;      // no line buffering, but keep ECHO
    raw.c_cc[VMIN] = 0; raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}
inline void disableRawMode() { tcsetattr(STDIN_FILENO, TCSANOW, &g_orig_termios); }

inline void signalHandler(int) { running = false; }

// ============================================================================
// Interface between controller.cpp (your PID code) and pendulum_runner.cpp
// (the IMU/motor/keyboard harness that calls it)
// ============================================================================

// ---- Tunable gains: DEFINED by you in controller.cpp, used by the runner's
// keyboard commands (p/d/i/w/f) and control loop. ----
extern double Kp;
extern double Kd;
extern double Ki;
extern double DEADBAND;
extern double INTEGRAL_TERM_MAX;
extern bool integralEnabled;

// ---- Functions: IMPLEMENTED by you in controller.cpp. See that file for the
// full spec of what each one should do. ----
double applyDeadband(double u);

double computePID(double theta, double thetaDot, double dt,
                   double& thetaIntegral, bool integralEnabled_,
                   double& integralTermOut);

// ---- Runs the full balance program (IMU, motor, keyboard interface, safety
// limits, telemetry). Defined in pendulum_runner.cpp; main() just calls this. ----
int runPendulumLab();

