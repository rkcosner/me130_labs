#include "me130_pendulum/hardware.hpp"

#include <gpiod.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fcntl.h>
#include <fstream>
#include <linux/i2c-dev.h>
#include <poll.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <unistd.h>

namespace me130
{
namespace
{

void writeSysfs(const std::string &path, const std::string &value)
{
    std::ofstream f(path);
    if (!f) throw std::runtime_error("Could not open: " + path);
    f << value;
    if (!f) throw std::runtime_error("Could not write to: " + path);
}

}  // namespace

// ---------------------------------------------------------------- HardwarePwm

HardwarePwm::HardwarePwm(int channel, int frequency_hz) : channel_(channel)
{
    pwm_path_ = "/sys/class/pwm/pwmchip0/pwm" + std::to_string(channel_);
    period_ns_ = static_cast<long long>(1e9 / frequency_hz);

    if (access(pwm_path_.c_str(), F_OK) != 0)
    {
        writeSysfs("/sys/class/pwm/pwmchip0/export", std::to_string(channel_));
        for (int i = 0; i < 100 && access(pwm_path_.c_str(), F_OK) != 0; ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (access(pwm_path_.c_str(), F_OK) != 0)
        throw std::runtime_error(
            "PWM channel " + std::to_string(channel_) +
            " did not appear. Check dtoverlay=pwm-2chan,... in /boot/firmware/config.txt, "
            "and that /sys/class/pwm/pwmchip0/export is writable.");

    // Writing export CREATES this subtree, and the kernel makes it root-owned.
    // udev fixes it asynchronously and loses the race against the writes just
    // below, so wait for it -- me130-pwm.service normally exports at boot and
    // makes this a no-op.
    const std::string period_path = pwm_path_ + "/period";
    for (int i = 0; i < 50 && access(period_path.c_str(), W_OK) != 0; ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

    if (access(period_path.c_str(), W_OK) != 0)
        throw std::runtime_error(
            "PWM channel " + std::to_string(channel_) + " is exported but " + period_path +
            " is not writable by this user.\n"
            "  The pwm" + std::to_string(channel_) + "/ directory is created by the export "
            "and is root-owned until something fixes it.\n"
            "  Fix: sudo ./me130_permissions.sh   (installs me130-pwm.service, then reboot)\n"
            "  Right now:  sudo chgrp -R gpio " + pwm_path_ +
            " && sudo chmod -R g+rw " + pwm_path_);

    try { writeSysfs(pwm_path_ + "/enable", "0"); } catch (...) {}
    writeSysfs(pwm_path_ + "/period", std::to_string(period_ns_));
    writeSysfs(pwm_path_ + "/duty_cycle", "0");
    // Some kernels default polarity to "inversed" (duty_cycle = LOW time);
    // the drive/brake math needs "normal".
    try { writeSysfs(pwm_path_ + "/polarity", "normal"); } catch (...) {}
    writeSysfs(pwm_path_ + "/enable", "1");

    duty_fd_ = open((pwm_path_ + "/duty_cycle").c_str(), O_WRONLY);
    if (duty_fd_ < 0)
        throw std::runtime_error("Could not open duty_cycle fd for channel " +
                                 std::to_string(channel_));
}

HardwarePwm::~HardwarePwm()
{
    if (duty_fd_ >= 0) close(duty_fd_);
    try { writeSysfs(pwm_path_ + "/duty_cycle", "0"); writeSysfs(pwm_path_ + "/enable", "0"); }
    catch (...) {}
}

void HardwarePwm::setDuty(double duty)
{
    duty = std::clamp(duty, 0.0, 1.0);
    long long duty_ns = static_cast<long long>(duty * period_ns_);
    // Leave one tick of off-time: many H-bridges want it to refresh the
    // high-side bootstrap, and 20 ns out of 50 us is irrelevant to torque.
    if (duty_ns >= period_ns_) duty_ns = period_ns_ - 1;
    const std::string s = std::to_string(duty_ns);
    lseek(duty_fd_, 0, SEEK_SET);
    if (write(duty_fd_, s.c_str(), s.size()) < 0) { /* best effort: one dropped tick is not critical */ }
}

// ------------------------------------------------------------------ MotorGpio

MotorGpio::MotorGpio(int ps_gpio, int dir_gpio)
{
    chip_ = gpiod_chip_open("/dev/gpiochip0");
    if (!chip_) throw std::runtime_error("Could not open /dev/gpiochip0");

    ps_line_ = gpiod_chip_get_line(chip_, ps_gpio);
    dir_line_ = gpiod_chip_get_line(chip_, dir_gpio);
    if (!ps_line_ || !dir_line_)
        throw std::runtime_error("Could not obtain the PS/DIR GPIO lines.");

    if (gpiod_line_request_output(ps_line_, "me130_motor_ps", 0) < 0)
        throw std::runtime_error("Could not request the PS GPIO.");
    if (gpiod_line_request_output(dir_line_, "me130_motor_dir", 0) < 0)
        throw std::runtime_error("Could not request the DIR GPIO.");
}

MotorGpio::~MotorGpio()
{
    coast();
    if (chip_) gpiod_chip_close(chip_);
}

void MotorGpio::enableDriver() { gpiod_line_set_value(ps_line_, 1); }
void MotorGpio::coast() { if (ps_line_) gpiod_line_set_value(ps_line_, 0); }
void MotorGpio::setDirection(int dir) { gpiod_line_set_value(dir_line_, dir > 0 ? 0 : 1); }

// ----------------------------------------------------------- QuadratureEncoder

QuadratureEncoder::QuadratureEncoder(int gpio_a, int gpio_b)
{
    chip_ = gpiod_chip_open("/dev/gpiochip0");
    if (!chip_) throw std::runtime_error("Could not open /dev/gpiochip0");

    a_line_ = gpiod_chip_get_line(chip_, gpio_a);
    b_line_ = gpiod_chip_get_line(chip_, gpio_b);
    if (!a_line_ || !b_line_)
        throw std::runtime_error("Could not obtain the encoder GPIO lines.");

    if (gpiod_line_request_both_edges_events(a_line_, "me130_encoder_a") < 0)
        throw std::runtime_error("Could not request encoder A.");
    if (gpiod_line_request_both_edges_events(b_line_, "me130_encoder_b") < 0)
        throw std::runtime_error("Could not request encoder B.");

    thread_ = std::thread(&QuadratureEncoder::run, this);
}

QuadratureEncoder::~QuadratureEncoder()
{
    running_.store(false);
    if (thread_.joinable()) thread_.join();
    if (chip_) gpiod_chip_close(chip_);
}

void QuadratureEncoder::run()
{
    pollfd fds[2] = {{gpiod_line_event_get_fd(a_line_), POLLIN, 0},
                     {gpiod_line_event_get_fd(b_line_), POLLIN, 0}};

    // Sign flipped versus raw quadrature math, so a +direction command reads
    // as +counts. The characterization data depends on this convention.
    static constexpr std::int8_t table[16] = {
         0, +1, -1,  0,
        -1,  0,  0, +1,
        +1,  0,  0, -1,
         0, -1, +1,  0};

    int a = gpiod_line_get_value(a_line_);
    int b = gpiod_line_get_value(b_line_);
    int previous = (a << 1) | b;

    while (running_.load())
    {
        if (poll(fds, 2, 100) <= 0) continue;
        if (fds[0].revents & POLLIN) { gpiod_line_event e; gpiod_line_event_read(a_line_, &e); }
        if (fds[1].revents & POLLIN) { gpiod_line_event e; gpiod_line_event_read(b_line_, &e); }

        a = gpiod_line_get_value(a_line_);
        b = gpiod_line_get_value(b_line_);
        const int current = (a << 1) | b;
        counts_.fetch_add(table[(previous << 2) | current]);
        previous = current;
    }
}

// -------------------------------------------------------------------- Mpu6050

Mpu6050::Mpu6050(const std::string &device, int address) : address_(address)
{
    fd_ = open(device.c_str(), O_RDWR);
    if (fd_ < 0)
        throw std::runtime_error("Could not open " + device +
                                 ". Enable I2C and check that the device exists.");
    if (ioctl(fd_, I2C_SLAVE, address_) < 0)
        throw std::runtime_error("Could not select the MPU6050 on the I2C bus.");

    writeReg(0x6B, 0x00);   // PWR_MGMT_1: wake
    writeReg(0x1B, 0x08);   // GYRO_CONFIG:  +/-500 deg/s
    writeReg(0x1C, 0x08);   // ACCEL_CONFIG: +/-4 g
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

Mpu6050::~Mpu6050() { if (fd_ >= 0) close(fd_); }

void Mpu6050::writeReg(std::uint8_t reg, std::uint8_t value)
{
    std::uint8_t buf[2] = {reg, value};
    if (write(fd_, buf, 2) != 2) throw std::runtime_error("MPU6050 register write failed.");
}

bool Mpu6050::read(float &acc_x, float &acc_y, float &gyro_z_dps)
{
    std::uint8_t reg = 0x3B;   // ACCEL_XOUT_H
    if (write(fd_, &reg, 1) != 1) return false;

    std::uint8_t b[14];
    if (::read(fd_, b, sizeof(b)) != 14) return false;

    auto s16 = [](std::uint8_t hi, std::uint8_t lo) -> std::int16_t {
        return static_cast<std::int16_t>((static_cast<std::uint16_t>(hi) << 8) | lo);
    };

    acc_x = static_cast<float>(s16(b[0], b[1]));
    acc_y = static_cast<float>(s16(b[2], b[3]));
    gyro_z_dps = static_cast<float>(s16(b[12], b[13])) / 65.5f;   // +/-500 dps
    return true;
}

double Mpu6050::calibrateGyroBiasRadPerSec(int samples)
{
    double sum_dps = 0.0;
    int good = 0;
    for (int i = 0; i < samples; ++i)
    {
        float ax, ay, gz;
        if (read(ax, ay, gz)) { sum_dps += gz; ++good; }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (good < samples / 2)
        throw std::runtime_error("Too many MPU6050 read failures during gyro calibration.");
    return (sum_dps / good) * (M_PI / 180.0);
}

}  // namespace me130
