/* Encoder-only Inverted Pendulum balance -- Raspberry Pi 4B + BD65496MUV + 25:1 20D
 * Ported from ESP32 balance_enc.cpp. Reuses the motor/encoder infra validated in
 * motor_characterization.cpp (same sign-corrected quadrature decode).
 * UPDATED here for BD65496MUV EN/IN mode: one hardware PWM + one DIR GPIO,
 * while preserving slow-decay-style DRIVE/BRAKE behavior.
 *
 * NO IMU. The rod is on the output shaft, so the motor encoder measures rod angle
 * directly: theta = (count - zero) * 2*pi / COUNTS_PER_OUTPUT_REV. 'z' zeroes it
 * while you hold the rod vertical.
 *
 * theta      : from encoder count (relative to the 'z' zero)
 * theta_dot  : EMA-smoothed finite-difference of theta (encoder is quantized to
 *              360/500 = 0.72 deg/count here; tune smoothing with 'a')
 * Control    : PD(+I)  u = -(Kp*theta + Ki*integral(theta) + Kd*theta_dot), deadband
 *              feed-forward, slow-decay PWM. Integral term is off by default ('t' to
 *              enable) -- it exists to null the steady-state offset a pure PD leaves
 *              when gravity/friction holds the rod at a nonzero angle instead of
 *              returning it to vertical. Simple anti-windup: the integral accumulator
 *              is clamped so Ki*integral can never exceed INTEGRAL_TERM_MAX on its own.
 * Caveat     : encoder is motor-side of the 25:1 box -> gear backlash may cause a
 *              little hunting near vertical. Good enough for a first balance test.
 *
 * SAFETY: boots DISARMED. 'e' arms only within ARM_WINDOW of the 'z' zero.
 *         Auto-coasts if |theta| > THETA_MAX. 'x' = kill. 's' = flip motor sign.
 *         >>> First run: support the rod by hand and verify theta/sign before arming. <<<
 *
 * WIRING (PWM+DIR, BD65496MUV EN/IN mode):
 *   INA->GPIO18(PWM0)   INB->GPIO25(DIR)   PS->GPIO24
 *   PWM/MODE->3.3V      Encoder A->GPIO17  Encoder B->GPIO27
 * REQUIRED in /boot/firmware/config.txt:
 *   dtoverlay=pwm,pin=18,func=2
 * (Your existing pwm-2chan overlay can also remain; this program only uses pwm0.)
 *
 * Commands (type letter, then value if needed, then Enter):
 *   z=set-upright  e=arm  x=disarm  s=flip-motor-sign  t=toggle-integral  q=quit
 *   p<Kp>  d<Kd>  i<Ki>  f<deadband>  a<rate_lpf 0..1>
 */
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

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

// ---- config: BD65496MUV EN/IN mode, PWM + DIR ----
constexpr int GPIO_PS = 24, GPIO_DIR = 25, GPIO_ENC_A = 17, GPIO_ENC_B = 27;
constexpr int PWM_CHANNEL = 0, PWM_FREQ_HZ = 20000;
constexpr double GEAR_RATIO = 25.0, ENCODER_COUNTS_PER_MOTOR_REV = 20.0;
constexpr double COUNTS_PER_OUTPUT_REV = ENCODER_COUNTS_PER_MOTOR_REV * GEAR_RATIO;
constexpr double PI = 3.14159265358979323846;
constexpr double DEG2RAD = PI / 180.0, RAD2DEG = 180.0 / PI;
constexpr double RAD_PER_COUNT = 2.0 * PI / COUNTS_PER_OUTPUT_REV;

// ---- control gains (Kp/Kd/DEADBAND/RATE_LPF/MOTOR_SIGN ported from ESP32; Ki is new
// and defaults off -- start small (e.g. 0.5-2.0) and increase until the steady-state
// offset clears without introducing slow oscillation) ----
double Kp = 6.0, Kd = 0.3, Ki = 1.0, DEADBAND = 0.1, RATE_LPF = 0.05, MOTOR_SIGN = 1.0;
bool integralEnabled = false;
double INTEGRAL_TERM_MAX = 0.5;   // simple anti-windup ceiling: caps |Ki*integral|; tune with 'w'

constexpr double LOOP_HZ = 500.0, DT = 1.0 / LOOP_HZ;
constexpr double ARM_WINDOW = 5.0 * DEG2RAD;
constexpr double THETA_MAX = 90.0 * DEG2RAD;

std::atomic<long long> encoder_count{0};
std::atomic<bool> running{true};

void writeSysfs(const std::string& path, const std::string& value)
{
    std::ofstream f(path);
    if (!f) throw std::runtime_error("Could not open: " + path);
    f << value;
    if (!f) throw std::runtime_error("Could not write to: " + path);
}

// ---- Hardware PWM via sysfs, with a PERSISTENT fd for duty_cycle so a 500Hz
// control loop isn't opening/closing a file 1000x/sec (that adds latency/jitter). ----
class HardwarePWM
{
public:
    HardwarePWM(int channel, int frequency_hz) : channel_(channel)
    {
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

// ---- Encoder thread: identical sign-corrected quadrature decode as motor_characterization.cpp ----
void encoderThread(gpiod_line* lineA, gpiod_line* lineB)
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
void setMotor(GPIOController& gpio, HardwarePWM& pwm, double u)
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

double applyDeadband(double u)
{
    if (std::fabs(u) < 1e-3) return 0.0;
    return std::copysign(DEADBAND, u) + (1.0 - DEADBAND) * u;
}

// ---- Terminal raw mode -- ECHO stays ON (unlike the sweep) so typed numeric
// arguments (e.g. "p6.5<Enter>") are visible, matching the Serial-monitor feel. ----
termios g_orig_termios;
void enableRawMode()
{
    tcgetattr(STDIN_FILENO, &g_orig_termios);
    termios raw = g_orig_termios;
    raw.c_lflag &= ~ICANON;      // no line buffering, but keep ECHO
    raw.c_cc[VMIN] = 0; raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
}
void disableRawMode() { tcsetattr(STDIN_FILENO, TCSANOW, &g_orig_termios); }

void signalHandler(int) { running = false; }

int main()
{
    std::signal(SIGINT, signalHandler);
    try
    {
        std::cout << "=============================================\n"
                  << "Encoder-only Inverted Pendulum Balance (RPi)\n"
                  << "=============================================\n\n"
                  << "Encoder output CPR: " << COUNTS_PER_OUTPUT_REV
                  << "  (" << RAD_PER_COUNT * RAD2DEG << " deg/count)\n\n";

        GPIOController gpio;
        HardwarePWM pwm(PWM_CHANNEL, PWM_FREQ_HZ);   // INA = hardware PWM
        setMotor(gpio, pwm, 0.0);

        std::thread encoder_thread(encoderThread, gpio.encoderA(), gpio.encoderB());

        std::cout << "Hold rod UPRIGHT, 'z' to zero, keep holding, 'e' to arm, release.\n"
                  << "Cmds: z=upright  e=arm  x=disarm  s=flip-sign  t=toggle-integral(currently "
                  << (integralEnabled ? "ON" : "OFF") << ")\n"
                  << "      p<Kp>  d<Kd>  i<Ki>  w<windup_limit>  f<deadband>  a<rateLPF>  q=quit\n\n";

        long long encZero = 0;
        double theta = 0.0, thetaPrev = 0.0, thetaDot = 0.0, thetaIntegral = 0.0;
        bool armed = false;

        enableRawMode();
        char pendingCmd = 0;
        std::string pendingArg;

        auto tPrev = Clock::now();
        auto tTelem = Clock::now();

        while (running)
        {
            // ---- non-blocking keyboard commands ----
            char c;
            while (read(STDIN_FILENO, &c, 1) > 0)
            {
                if (pendingCmd != 0)
                {
                    if (c == '\n' || c == '\r')
                    {
                        try {
                            double v = std::stod(pendingArg);
                            if (pendingCmd == 'p') { Kp = v; std::cout << ">> Kp=" << Kp << "\n"; }
                            else if (pendingCmd == 'd') { Kd = v; std::cout << ">> Kd=" << Kd << "\n"; }
                            else if (pendingCmd == 'i') { Ki = v; std::cout << ">> Ki=" << Ki << "\n"; }
                            else if (pendingCmd == 'w') { INTEGRAL_TERM_MAX = v; std::cout << ">> integral windup limit=" << INTEGRAL_TERM_MAX << "\n"; }
                            else if (pendingCmd == 'f') { DEADBAND = v; std::cout << ">> deadband=" << DEADBAND << "\n"; }
                            else if (pendingCmd == 'a') { RATE_LPF = v; std::cout << ">> rateLPF=" << RATE_LPF << "\n"; }
                        } catch (...) { std::cout << ">> bad value, ignored\n"; }
                        pendingCmd = 0; pendingArg.clear();
                    }
                    else pendingArg += c;
                    continue;
                }

                if (c == 'z') {
                    encZero = encoder_count.load();
                    theta = thetaPrev = thetaDot = thetaIntegral = 0.0;
                    std::cout << ">> upright set (theta=0 here)\n";
                } else if (c == 'e') {
                    if (std::fabs(theta) < ARM_WINDOW) { armed = true; std::cout << ">> ARMED\n"; }
                    else std::cout << ">> too far from upright to arm\n";
                } else if (c == 'x') {
                    armed = false; thetaIntegral = 0.0; setMotor(gpio, pwm, 0.0);
                    std::cout << ">> DISARMED\n";
                } else if (c == 's') {
                    MOTOR_SIGN = -MOTOR_SIGN;
                    std::cout << ">> MOTOR_SIGN=" << (MOTOR_SIGN > 0 ? "+1" : "-1") << "\n";
                } else if (c == 't') {
                    integralEnabled = !integralEnabled;
                    thetaIntegral = 0.0;   // start clean whichever way it's toggled
                    std::cout << ">> integral=" << (integralEnabled ? "ON" : "OFF") << "\n";
                } else if (c == 'q') {
                    running = false;
                } else if (c == 'p' || c == 'd' || c == 'i' || c == 'w' || c == 'f' || c == 'a') {
                    pendingCmd = c; pendingArg.clear();
                }
            }

            // ---- fixed-rate control loop (sleep_until avoids busy-waiting a CPU core) ----
            auto now = Clock::now();
            auto elapsed = std::chrono::duration<double>(now - tPrev).count();
            if (elapsed < DT)
            {
                std::this_thread::sleep_until(tPrev + std::chrono::duration<double>(DT));
                now = Clock::now();
                elapsed = std::chrono::duration<double>(now - tPrev).count();
            }
            double dt = elapsed;
            tPrev = now;

            long long cnt = encoder_count.load();
            theta = (cnt - encZero) * RAD_PER_COUNT;
            double rawRate = (theta - thetaPrev) / dt;
            thetaDot += RATE_LPF * (rawRate - thetaDot);
            thetaPrev = theta;

            if (std::fabs(theta) > THETA_MAX)
            {
                if (armed) std::cout << ">> FELL OVER -- motor off\n";
                armed = false;
                setMotor(gpio, pwm, 0.0);
            }

            double u = 0.0;
            double integralTerm = 0.0;
            if (armed)
            {
                if (integralEnabled)
                {
                    thetaIntegral += theta * dt;
                    double bound = (Ki > 1e-9) ? (INTEGRAL_TERM_MAX / Ki) : 1e9;
                    thetaIntegral = std::clamp(thetaIntegral, -bound, bound);
                    integralTerm = Ki * thetaIntegral;
                }
                u = -(Kp * theta + integralTerm + Kd * thetaDot);
                setMotor(gpio, pwm, applyDeadband(MOTOR_SIGN * u));
            }
            else
            {
                setMotor(gpio, pwm, 0.0);
            }

            double telemElapsed = std::chrono::duration<double>(now - tTelem).count();
            if (telemElapsed > 0.05)
            {
                tTelem = now;
                std::cout << "theta=" << theta * RAD2DEG << " deg  rate=" << thetaDot * RAD2DEG
                          << " dps  u=" << u << "  Iterm=" << integralTerm
                          << "  enc=" << cnt << "  " << (armed ? "ARMED" : "off") << "\n";
            }
        }
        disableRawMode();

        setMotor(gpio, pwm, 0.0);
        running = false;
        encoder_thread.join();

        std::cout << "\nDone.\n";
    }
    catch (const std::exception& e)
    {
        running = false;
        disableRawMode();
        std::cerr << "\nERROR: " << e.what() << "\n";
        return 1;
    }
    return 0;
}