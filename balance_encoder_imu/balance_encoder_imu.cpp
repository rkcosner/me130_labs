/* IMU-based Inverted Pendulum balance -- Raspberry Pi 4B + GY-521/MPU6050
 * + BD65496MUV + 25:1 20D
 *
 * Motor drive: BD65496MUV EN/IN mode, one hardware PWM + one DIR GPIO,
 * preserving slow-decay-style DRIVE/BRAKE behavior.
 *
 * IMU is mounted on the rod like the ESP32 version:
 *   - Z axis points along the motor/pivot shaft
 *   - tilt is in the X-Y accelerometer plane
 *   - gyro Z is the pendulum angular rate
 *
 * theta      : complementary filter of accelerometer absolute tilt + gyro integration
 * theta_dot  : gyro Z directly (bias-corrected)
 * encoder    : telemetry only; it is motor-side of the 25:1 gearbox
 *
 * Control    : PD(+I) u = -(Kp*theta + Ki*integral(theta) + Kd*theta_dot),
 *              then deadband feed-forward and PWM+DIR motor command.
 * Integral term remains available but is OFF by default.
 *
 * OPEN-LOOP (system ID): 'u<duty>' applies a constant duty u (pre-deadband,
 *              same path as the controller: applyDeadband(MOTOR_SIGN*u)).
 *              Intended for step tests with the rod HANGING DOWN ('z' there).
 *              Bypasses the ARM_WINDOW check; still auto-stops if |theta| > THETA_MAX.
 *              |u| is limited to OPEN_LOOP_MAX. 'u0' or 'x' stops it. 'e' cancels it.
 *
 * SAFETY: boots DISARMED. 'e' arms only within ARM_WINDOW of the 'z' zero.
 *         Auto-coasts if |theta| > THETA_MAX. 'x' = kill. 's' = flip motor sign.
 *         >>> First run: support the rod by hand and verify theta/sign before arming. <<<
 *
 * WIRING:
 *   BD65496MUV:
 *     INA->GPIO18(PWM0)   INB->GPIO25(DIR)   PS->GPIO24
 *     PWM/MODE->3.3V      VCC->3.3V
 *   Encoder:
 *     A->GPIO17            B->GPIO27
 *   GY-521 / MPU6050:
 *     VCC->3.3V            GND->GND
 *     SDA->GPIO2 (physical pin 3)
 *     SCL->GPIO3 (physical pin 5)
 * REQUIRED in /boot/firmware/config.txt:
 *   dtoverlay=pwm,pin=18,func=2
 * (Your existing pwm-2chan overlay can also remain; this program only uses pwm0.)
 *
 * Commands (type letter, then value if needed, then Enter):
 *   z=set-upright  e=arm  x=disarm/stop  s=flip-motor-sign
 *   r=flip-rate-sign  t=toggle-integral  q=quit
 *   p<Kp>  d<Kd>  i<Ki>  w<integral_limit>  f<deadband>
 *   c<complementary_filter 0..1>  u<open-loop duty>
 *   l=toggle CSV logging   k=automated open-loop step sequence
 *   b=automated frequency sweep (drives every SWEEP_FREQS entry, logs itself)
 *   A=both, back to back, into a single log file
 *   a<amp>  g<freq_hz>   (open-loop sine; g0 stops)
 *
 * LOGGING: 'l' streams every control tick to <test>_<timestamp>.csv at the
 * full loop rate. Rows go through a 1 MB stream buffer and use '\n' rather than
 * std::endl, so the 500 Hz loop never blocks on a write syscall; the file is
 * flushed when logging is toggled off and again at exit (q, Ctrl-C, or error).
 * The log is named after the test that started it -- steps_, sweep_, full_ or
 * manual_ -- and the mode column is text (coast / pd / step / sine), so free and
 * forced response separate when plotting without needing a legend for the codes.
 * The u_cmd column is the PRE-deadband command.
 *
 * FREQUENCY RESPONSE (rod HANGING DOWN): 'z' to zero, then 'b'. The sweep starts
 * logging, drives every SWEEP_FREQS entry for a hold time computed from
 * SWEEP_SKIP_S + SWEEP_CYCLES/f (so low frequencies are held longer and survive
 * the transient discard in freq_response.py), then stops and flushes the log.
 * Manual equivalent: 'l', 'a<amp>', then 'g<freq>' per point, 'g0' to stop.
 * Sine mode is mutually exclusive with arming and with constant open loop, and
 * is cancelled by 'x', 'e', |theta| > THETA_MAX, and IMU loss.
 */
#include <gpiod.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
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

// ---- control / IMU filter ----
double Kp = 5.0, Kd = 0.6, Ki = 1.0; 
double DEADBAND = 0.065; // TODO: enter your deadband information
double MOTOR_SIGN = 1.0; // TODO: if your motor is spinning backwards, change this to -1.0
double RATE_SIGN = 1.0;
double COMP = 0.995;             // complementary-filter gyro weight
bool integralEnabled = false;
double INTEGRAL_TERM_MAX = 4.0;

constexpr double LOOP_HZ = 500.0, DT = 1.0 / LOOP_HZ;

//it will arm within +- 5 degrees of the selected zero position
constexpr double ARM_WINDOW = 5.0 * DEG2RAD;

//it will automatically disaerm if the pendulum goes 90 degree off the zero position in both directions
constexpr double THETA_MAX = 90.0 * DEG2RAD;
constexpr int IMU_FAIL_MAX = 8;

// open-loop step-test duty limit (pre-deadband). Raise deliberately if needed.
constexpr double OPEN_LOOP_MAX = 0.5;

// ---- high-rate CSV logging ----
// Rows stream through a large stream buffer so the 500 Hz loop does not block
// on a write syscall per tick. Never std::endl (it flushes); flush happens on
// toggle-off and at exit.
constexpr size_t LOG_BUFFER_BYTES = 1 << 20;

// ---- automated open-loop step sequence ('a') ----
// Each duty is held for STEP_HOLD_S, then the motor coasts for STEP_REST_S so
// the rod settles before the next one. Alternating signs keep the rod near its
// rest position instead of winding one way. Edit this list to suit your rig.
constexpr double STEP_DUTIES[] = {0.10, -0.10, 0.15, -0.15, 0.20, -0.20, 0.25, -0.25};
constexpr int STEP_COUNT = static_cast<int>(sizeof(STEP_DUTIES) / sizeof(STEP_DUTIES[0]));
constexpr double STEP_HOLD_S = 1.0, STEP_REST_S = 2.5;

// ---- automated frequency sweep ('b') ----
// Hold time per frequency is computed, not fixed: freq_response.py throws away
// the first SWEEP_SKIP_S seconds as transient and then needs whole cycles to
// fit, so a low frequency must be driven for longer. Keep SWEEP_SKIP_S equal to
// that script's --skip.
constexpr double SWEEP_FREQS[] = {0.3, 0.5, 0.8, 1.0, 1.3, 1.8, 2.5, 3.5};
constexpr int SWEEP_COUNT = static_cast<int>(sizeof(SWEEP_FREQS) / sizeof(SWEEP_FREQS[0]));
constexpr double SWEEP_SKIP_S = 10.0;    // discarded transient, matches --skip
constexpr double SWEEP_CYCLES = 4.0;     // usable cycles wanted after the transient
constexpr double SWEEP_SETTLE_S = 2.0;   // coast between frequencies

inline double sweepHoldSeconds(double f) { return SWEEP_SKIP_S + SWEEP_CYCLES / f; }

std::atomic<long long> encoder_count{0};
std::atomic<bool> running{true};

// mode column: 0 = coasting (free response), 1 = closed-loop PD,
// 2 = open-loop constant step, 3 = open-loop sine.
std::ofstream logFile;
std::vector<char> logStreamBuf;
bool logging = false;
Clock::time_point logStart;
int segment = 0;

// mode column values, written as text so a raw CSV is readable without a key.
inline const char* modeLabel(bool isArmed, bool isOpenLoop, bool isSine)
{
    return isArmed ? "pd" : (isOpenLoop ? "step" : (isSine ? "sine" : "coast"));
}

// tag names the test, so a directory of logs is self-describing:
//   steps_20260919_143022.csv   sweep_...   full_...   manual_...
std::string timestampedLogName(const char* tag)
{
    std::time_t t = std::time(nullptr);
    char stamp[32];
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&t));
    return std::string(tag) + "_" + stamp + ".csv";
}

void startLogging(const char* tag)
{
    if (logging) return;
    const std::string name = timestampedLogName(tag);

    // pubsetbuf must precede open() to take effect.
    logStreamBuf.assign(LOG_BUFFER_BYTES, '\0');
    logFile.rdbuf()->pubsetbuf(logStreamBuf.data(),
                               static_cast<std::streamsize>(logStreamBuf.size()));
    logFile.open(name);
    if (!logFile) { std::cout << ">> could not open " << name << " -- not logging\n"; return; }

    // Default 6 significant digits would quantize t_s once past ~100 s.
    logFile << std::fixed << std::setprecision(6);
    logFile << "t_s,mode,u_cmd,theta_rad,theta_dot_rad_s,enc,freq_hz,amp,segment,step_duty\n";
    logStart = Clock::now();
    logging = true;
    std::cout << ">> LOGGING -> " << name << "\n";
}

void stopLogging()
{
    if (!logging) return;
    logFile.flush();
    logFile.close();
    logging = false;
    std::cout << ">> logging stopped and flushed\n";
}

void writeSysfs(const std::string& path, const std::string& value)
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
        //path to the enables pwm channer where all the configs will be written
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
        //Check if the duty was written or not
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
                  << "IMU-based Inverted Pendulum Balance (RPi + GY-521)\n"
                  << "=============================================\n\n"
                  << "Encoder output CPR: " << COUNTS_PER_OUTPUT_REV
                  << "  (" << RAD_PER_COUNT * RAD2DEG << " deg/count)\n\n";

        GPIOController gpio;
        HardwarePWM pwm(PWM_CHANNEL, PWM_FREQ_HZ);   // INA = hardware PWM
        setMotor(gpio, pwm, 0.0);

        std::thread encoder_thread(encoderThread, gpio.encoderA(), gpio.encoderB());

        MPU6050 imu;
        double gyroBias = imu.calibrateGyroBiasRadPerSec();

        float ax0 = 0.0f, ay0 = 0.0f, gz0 = 0.0f;
        if (!imu.read(ax0, ay0, gz0))
            throw std::runtime_error("Initial MPU6050 read failed.");

        double thetaOffset = 0.0;
        double theta = std::atan2(static_cast<double>(ax0), static_cast<double>(ay0));
        double thetaDot = 0.0, thetaIntegral = 0.0;
        int imuFail = 0;
        bool armed = false;
        bool openLoop = false;
        double openLoopDuty = 0.0;

        enum class StepPhase { Idle, Hold, Rest };
        StepPhase stepPhase = StepPhase::Idle;
        int stepIndex = 0;
        double stepTimer = 0.0;
        bool stepStartedLog = false;

        // ---- open-loop sine (frequency-response test, rod HANGING DOWN) ----
        bool sineOn = false;
        double sineAmp = 0.10;          // pre-deadband duty amplitude
        double sineFreq = 0.0;          // Hz
        auto sineStart = Clock::now();

        enum class SweepPhase { Idle, Drive, Settle };
        SweepPhase sweepPhase = SweepPhase::Idle;
        int sweepIndex = 0;
        double sweepTimer = 0.0;
        bool sweepStartedLog = false;

        bool chainToSweep = false;   // set by the combined run ('A')
        // Peak |theta| within the current step or frequency. For a sweep this is
        // the response amplitude, i.e. the number the Bode gain is built from,
        // so it is the one live value worth watching during an automated run.
        double segPeakTheta = 0.0;

        auto cancelSweep = [&](const char* why) {
            if (sweepPhase == SweepPhase::Idle) return;
            sweepPhase = SweepPhase::Idle;
            sineOn = false; sineFreq = 0.0;
            if (sweepStartedLog) { stopLogging(); sweepStartedLog = false; }
            std::cout << ">> frequency sweep " << why << "\n";
        };

        auto cancelSteps = [&](const char* why) {
            if (stepPhase == StepPhase::Idle) return;
            stepPhase = StepPhase::Idle;
            chainToSweep = false;
            openLoop = false; openLoopDuty = 0.0;
            if (stepStartedLog) { stopLogging(); stepStartedLog = false; }
            std::cout << ">> step sequence " << why << "\n";
        };

        // Shared by 'b' and by the hand-off at the end of the combined run.
        auto startSweep = [&]() {
            double total = 0.0;
            for (int i = 0; i < SWEEP_COUNT; ++i)
                total += sweepHoldSeconds(SWEEP_FREQS[i]) + SWEEP_SETTLE_S;

            sweepIndex = 0; sweepTimer = 0.0;
            sweepPhase = SweepPhase::Drive;
            sineFreq = SWEEP_FREQS[0];
            sineStart = Clock::now();
            sineOn = true;
            ++segment; segPeakTheta = 0.0;
            std::cout << ">> AUTO FREQUENCY SWEEP: " << SWEEP_COUNT
                      << " frequencies, amp=" << sineAmp
                      << ", ~" << static_cast<int>(total + 0.5) << " s total\n"
                      << ">> freq 1/" << SWEEP_COUNT << "  " << sineFreq
                      << " Hz for " << sweepHoldSeconds(sineFreq) << " s\n";
        };

        std::cout << "Ready. Hold rod UPRIGHT, 'z' to set vertical, keep holding, 'e' to arm, release.\n"
                  << "Step test: rod HANGING DOWN, 'z', then u<duty> (e.g. u0.2), u0 or x to stop.\n"
                  << "Auto step test: rod HANGING DOWN, 'z', then 'k' -- runs all "
                  << STEP_COUNT << " steps and logs them.\n"
                  << "Freq response: rod HANGING DOWN, 'z', then 'b' -- sweeps all "
                  << SWEEP_COUNT << " frequencies\n"
                  << "               and logs them. (Manual: a<amp> then g<freq_hz>; g0 stops.)\n"
                  << "Cmds: z=upright  e=arm  x=disarm/stop  s=flip-motor-sign  r=flip-rate-sign\n"
                  << "      t=toggle-integral  l=toggle-logging  k=auto-step-sequence\n"
                  << "      b=auto-frequency-sweep  A=BOTH (steps then sweep, one log)\n"
                  << "      a<sine amp>  g<sine freq Hz, 0=stop>\n"
                  << "      p<Kp>  d<Kd>  i<Ki>  w<windup_limit>\n"
                  << "      f<deadband>  c<COMP>  u<open-loop duty>  q=quit\n\n";

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
                            else if (pendingCmd == 'c') { COMP = std::clamp(v, 0.0, 1.0); std::cout << ">> COMP=" << COMP << "\n"; }
                            else if (pendingCmd == 'a') {
                                sineAmp = std::clamp(std::fabs(v), 0.0, OPEN_LOOP_MAX);
                                if (sineAmp != std::fabs(v))
                                    std::cout << ">> amplitude limited to " << OPEN_LOOP_MAX << "\n";
                                std::cout << ">> sine amp=" << sineAmp << "\n";
                            }
                            else if (pendingCmd == 'g') {
                                cancelSweep("cancelled");
                                if (v <= 0.0) {
                                    sineOn = false; sineFreq = 0.0;
                                    setMotor(gpio, pwm, 0.0);
                                    std::cout << ">> SINE OFF\n";
                                } else {
                                    armed = false; thetaIntegral = 0.0;   // mutually exclusive with PD
                                    openLoop = false; openLoopDuty = 0.0; // ...and with constant open loop
                                    cancelSteps("cancelled");
                                    sineFreq = v;
                                    sineStart = Clock::now();
                                    sineOn = true;
                                    std::cout << ">> SINE f=" << sineFreq << " Hz  amp=" << sineAmp << "\n";
                                }
                            }
                            else if (pendingCmd == 'u') {
                                openLoopDuty = std::clamp(v, -OPEN_LOOP_MAX, OPEN_LOOP_MAX);
                                if (openLoopDuty != v) std::cout << ">> duty limited to +/-" << OPEN_LOOP_MAX << "\n";
                                if (std::fabs(openLoopDuty) < 1e-9) {
                                    openLoop = false; openLoopDuty = 0.0;
                                    setMotor(gpio, pwm, 0.0);
                                    std::cout << ">> OPEN-LOOP OFF\n";
                                } else {
                                    cancelSweep("cancelled");
                                    armed = false; thetaIntegral = 0.0;   // never run PD and open loop together
                                    sineOn = false;                       // ...nor sine and constant open loop
                                    openLoop = true;
                                    std::cout << ">> OPEN-LOOP u=" << openLoopDuty << "\n";
                                }
                            }
                        } catch (...) { std::cout << ">> bad value, ignored\n"; }
                        pendingCmd = 0; pendingArg.clear();
                    }
                    else pendingArg += c;
                    continue;
                }

                if (c == 'z') {
                    thetaOffset += theta;
                    theta = 0.0;
                    thetaDot = 0.0;
                    thetaIntegral = 0.0;
                    std::cout << ">> upright set (theta=0 here)\n";
                } else if (c == 'e') {
                    if (std::fabs(theta) < ARM_WINDOW) {
                        cancelSweep("cancelled");
                        openLoop = false; openLoopDuty = 0.0;
                        sineOn = false; sineFreq = 0.0;
                        armed = true; std::cout << ">> ARMED\n";
                    }
                    else std::cout << ">> too far from upright to arm\n";
                } else if (c == 'x') {
                    cancelSweep("cancelled");
                    armed = false; thetaIntegral = 0.0;
                    openLoop = false; openLoopDuty = 0.0;
                    sineOn = false; sineFreq = 0.0;
                    cancelSteps("cancelled");
                    setMotor(gpio, pwm, 0.0);
                    std::cout << ">> DISARMED\n";
                } else if (c == 'l') {
                    if (logging) stopLogging();
                    else { ++segment; startLogging("manual"); }
                } else if (c == 'b') {
                    if (armed) {
                        std::cout << ">> disarm first ('x') before the frequency sweep\n";
                    } else if (sweepPhase != SweepPhase::Idle) {
                        std::cout << ">> sweep already running ('x' cancels)\n";
                    } else {
                        openLoop = false; openLoopDuty = 0.0;
                        cancelSteps("cancelled");
                        if (!logging) { startLogging("sweep"); sweepStartedLog = true; }
                        startSweep();
                    }
                } else if (c == 'k' || c == 'A') {
                    const bool combined = (c == 'A');
                    if (armed) {
                        std::cout << ">> disarm first ('x') before the step sequence\n";
                    } else if (stepPhase != StepPhase::Idle) {
                        std::cout << ">> step sequence already running ('x' cancels)\n";
                    } else if (sweepPhase != SweepPhase::Idle) {
                        std::cout << ">> sweep already running ('x' cancels)\n";
                    } else {
                        cancelSweep("cancelled");
                        sineOn = false; sineFreq = 0.0;
                        stepIndex = 0; stepTimer = 0.0;
                        stepPhase = StepPhase::Hold;
                        openLoopDuty = std::clamp(STEP_DUTIES[0], -OPEN_LOOP_MAX, OPEN_LOOP_MAX);
                        openLoop = true;
                        chainToSweep = combined;
                        ++segment; segPeakTheta = 0.0;
                        if (!logging) {
                            startLogging(combined ? "full" : "steps");
                            stepStartedLog = true;
                        }
                        if (combined)
                            std::cout << ">> FULL CHARACTERIZATION: " << STEP_COUNT
                                      << " steps, then " << SWEEP_COUNT << " frequencies\n";
                        std::cout << ">> AUTO STEP SEQUENCE: " << STEP_COUNT << " steps\n"
                                  << ">> step 1/" << STEP_COUNT << "  duty=" << openLoopDuty << "\n";
                    }
                } else if (c == 's') {
                    MOTOR_SIGN = -MOTOR_SIGN;
                    std::cout << ">> MOTOR_SIGN=" << (MOTOR_SIGN > 0 ? "+1" : "-1") << "\n";
                } else if (c == 'r') {
                    RATE_SIGN = -RATE_SIGN;
                    std::cout << ">> RATE_SIGN=" << (RATE_SIGN > 0 ? "+1" : "-1") << "\n";
                } else if (c == 't') {
                    integralEnabled = !integralEnabled;
                    thetaIntegral = 0.0;   // start clean whichever way it's toggled
                    std::cout << ">> integral=" << (integralEnabled ? "ON" : "OFF") << "\n";
                } else if (c == 'q') {
                    running = false;
                } else if (c == 'p' || c == 'd' || c == 'i' || c == 'w' || c == 'f' || c == 'c'
                           || c == 'u' || c == 'a' || c == 'g') {
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

            // ---- automated step sequence: non-blocking, so the loop keeps
            // sensing and logging while it advances. Placed before the IMU read
            // so a dropped sample cannot stall the sequence clock. ----
            if (stepPhase != StepPhase::Idle)
            {
                stepTimer += dt;
                if (stepPhase == StepPhase::Hold && stepTimer >= STEP_HOLD_S)
                {
                    openLoop = false; openLoopDuty = 0.0;
                    stepPhase = StepPhase::Rest; stepTimer = 0.0;
                }
                else if (stepPhase == StepPhase::Rest && stepTimer >= STEP_REST_S)
                {
                    ++stepIndex;
                    if (stepIndex >= STEP_COUNT)
                    {
                        stepPhase = StepPhase::Idle;
                        std::cout << ">> step sequence COMPLETE\n";
                        if (chainToSweep)
                        {
                            // Keep the same log open across both tests; the sweep
                            // now owns it and will close it when it finishes.
                            chainToSweep = false;
                            sweepStartedLog = stepStartedLog;
                            stepStartedLog = false;
                            openLoop = false; openLoopDuty = 0.0;
                            startSweep();
                        }
                        else if (stepStartedLog) { stopLogging(); stepStartedLog = false; }
                    }
                    else
                    {
                        openLoopDuty = std::clamp(STEP_DUTIES[stepIndex], -OPEN_LOOP_MAX, OPEN_LOOP_MAX);
                        openLoop = true;
                        stepPhase = StepPhase::Hold; stepTimer = 0.0;
                        ++segment; segPeakTheta = 0.0;
                        std::cout << ">> step " << (stepIndex + 1) << "/" << STEP_COUNT
                                  << "  duty=" << openLoopDuty << "\n";
                    }
                }
            }

            // ---- automated frequency sweep: advances on the loop clock so the
            // sine keeps being driven and logged while it steps through. ----
            if (sweepPhase != SweepPhase::Idle)
            {
                sweepTimer += dt;
                const double hold = sweepHoldSeconds(SWEEP_FREQS[sweepIndex]);
                if (sweepPhase == SweepPhase::Drive && sweepTimer >= hold)
                {
                    sineOn = false; sineFreq = 0.0;
                    setMotor(gpio, pwm, 0.0);
                    sweepPhase = SweepPhase::Settle; sweepTimer = 0.0;
                }
                else if (sweepPhase == SweepPhase::Settle && sweepTimer >= SWEEP_SETTLE_S)
                {
                    ++sweepIndex;
                    if (sweepIndex >= SWEEP_COUNT)
                    {
                        sweepPhase = SweepPhase::Idle;
                        std::cout << ">> frequency sweep COMPLETE\n";
                        if (sweepStartedLog) { stopLogging(); sweepStartedLog = false; }
                    }
                    else
                    {
                        sineFreq = SWEEP_FREQS[sweepIndex];
                        sineStart = now;
                        sineOn = true;
                        sweepPhase = SweepPhase::Drive; sweepTimer = 0.0;
                        ++segment; segPeakTheta = 0.0;
                        std::cout << ">> freq " << (sweepIndex + 1) << "/" << SWEEP_COUNT
                                  << "  " << sineFreq << " Hz for "
                                  << sweepHoldSeconds(sineFreq) << " s\n";
                    }
                }
            }

            long long cnt = encoder_count.load();

            // ---- IMU sensing: complementary-filter structure ----
            float accX = 0.0f, accY = 0.0f, gyroZ_dps = 0.0f;
            if (!imu.read(accX, accY, gyroZ_dps))
            {
                if (++imuFail > IMU_FAIL_MAX)
                {
                    if (armed || openLoop || sineOn) std::cout << ">> IMU lost -- motor off\n";
                    cancelSweep("aborted -- IMU lost");
                    cancelSteps("aborted -- IMU lost");
                    armed = false;
                    openLoop = false; openLoopDuty = 0.0;
                    sineOn = false; sineFreq = 0.0;
                    setMotor(gpio, pwm, 0.0);
                }
                continue;  // hold last motor command briefly and retry next control tick
            }
            imuFail = 0;

            double rate = RATE_SIGN * (static_cast<double>(gyroZ_dps) * DEG2RAD - gyroBias);
            double accelAngle = std::atan2(static_cast<double>(accX), static_cast<double>(accY)) - thetaOffset;

            theta = COMP * (theta + rate * dt) + (1.0 - COMP) * accelAngle;
            thetaDot = rate;

            if (std::fabs(theta) > THETA_MAX)
            {
                if (armed) std::cout << ">> FELL OVER -- motor off\n";
                if (openLoop) std::cout << ">> OPEN-LOOP past THETA_MAX -- motor off\n";
                if (sineOn) std::cout << ">> SINE past THETA_MAX -- motor off\n";
                cancelSweep("ABORTED past THETA_MAX");
                cancelSteps("ABORTED past THETA_MAX");
                armed = false;
                openLoop = false; openLoopDuty = 0.0;
                sineOn = false; sineFreq = 0.0;
                setMotor(gpio, pwm, 0.0);
            }

            double u = 0.0;
            double uCmd = 0.0;
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
                uCmd = applyDeadband(MOTOR_SIGN * u);
                setMotor(gpio, pwm, uCmd);
            }
            else if (openLoop)
            {
                // Same actuation path as the controller, so the identified b matches closed loop.
                u = openLoopDuty;
                uCmd = applyDeadband(MOTOR_SIGN * u);
                setMotor(gpio, pwm, uCmd);
            }
            else if (sineOn)
            {
                // t measured from the start of this sine, so phase is well defined
                // across a frequency sweep. Same actuation path as the controller.
                double ts = std::chrono::duration<double>(now - sineStart).count();
                u = sineAmp * std::sin(2.0 * PI * sineFreq * ts);
                uCmd = applyDeadband(MOTOR_SIGN * u);
                setMotor(gpio, pwm, uCmd);
            }
            else
            {
                setMotor(gpio, pwm, 0.0);
            }

            const char* mode = modeLabel(armed, openLoop, sineOn);
            segPeakTheta = std::max(segPeakTheta, std::fabs(theta));

            if (logging)
            {
                // u, not uCmd: the column is the PRE-deadband command, which is
                // what the frequency-response fit needs as its input signal.
                logFile << std::chrono::duration<double>(now - logStart).count() << ','
                        << mode << ',' << u << ',' << theta << ',' << thetaDot << ','
                        << cnt << ',' << (sineOn ? sineFreq : 0.0) << ','
                        << (sineOn ? sineAmp : 0.0) << ',' << segment << ','
                        << ((stepPhase == StepPhase::Hold) ? openLoopDuty : 0.0) << '\n';
            }

            // An automated run is minutes long; at 20 Hz the raw telemetry buries
            // the progress messages. Print a once-a-second progress line instead,
            // and keep the detailed stream for hand-flown work.
            const bool autoRun = (stepPhase != StepPhase::Idle) ||
                                 (sweepPhase != SweepPhase::Idle);
            double telemElapsed = std::chrono::duration<double>(now - tTelem).count();
            if (telemElapsed > (autoRun ? 1.0 : 0.05))
            {
                tTelem = now;
                const std::streamsize oldPrec = std::cout.precision();
                if (autoRun) std::cout << std::fixed << std::setprecision(2);

                if (sweepPhase != SweepPhase::Idle)
                {
                    const bool driving = (sweepPhase == SweepPhase::Drive);
                    const double left = driving
                        ? sweepHoldSeconds(SWEEP_FREQS[sweepIndex]) - sweepTimer
                        : SWEEP_SETTLE_S - sweepTimer;
                    std::cout << "  sweep " << (sweepIndex + 1) << "/" << SWEEP_COUNT
                              << "   " << SWEEP_FREQS[sweepIndex] << " Hz  "
                              << (driving ? "drive" : "settle")
                              << "   " << left << "s left"
                              << "   peak|theta|=" << segPeakTheta * RAD2DEG << " deg\n";
                }
                else if (stepPhase != StepPhase::Idle)
                {
                    const bool holding = (stepPhase == StepPhase::Hold);
                    const double left = holding ? STEP_HOLD_S - stepTimer
                                                : STEP_REST_S - stepTimer;
                    std::cout << "  step " << (stepIndex + 1) << "/" << STEP_COUNT
                              << "   duty=" << STEP_DUTIES[stepIndex] << "  "
                              << (holding ? "hold" : "rest")
                              << "   " << left << "s left"
                              << "   peak|theta|=" << segPeakTheta * RAD2DEG << " deg\n";
                }
                else
                {
                    std::cout << "theta=" << theta * RAD2DEG << " deg  rate=" << thetaDot * RAD2DEG
                              << " dps  u=" << u << "  Iterm=" << integralTerm
                              << "  enc=" << cnt << "  imuFail=" << imuFail
                              << "  " << (armed ? "ARMED" : (openLoop ? "OPEN" : (sineOn ? "SINE" : "off"))) << "\n";
                }

                if (autoRun) { std::cout.unsetf(std::ios::floatfield); std::cout.precision(oldPrec); }
            }
        }
        disableRawMode();

        setMotor(gpio, pwm, 0.0);
        running = false;
        encoder_thread.join();

        stopLogging();
        std::cout << "\nDone.\n";
    }
    catch (const std::exception& e)
    {
        running = false;
        disableRawMode();
        stopLogging();
        std::cerr << "\nERROR: " << e.what() << "\n";
        return 1;
    }
    return 0;
}