/* Duty-vs-Speed sweep + Encoder -- Raspberry Pi 4B + BD65496MUV + Pololu 25:1 20D
 *
 * PWM+DIR port of motor_characterization_pwmpwm.cpp. Same job: find BREAKAWAY
 * duty (stiction, up-sweep), STOP duty (Coulomb friction, down-sweep), and the
 * roughly-linear duty->speed region above the deadband.
 *
 * DRIVER: BD65496MUV in EN/IN mode. One hardware PWM on INA sets magnitude,
 * one plain GPIO on INB sets direction. This matches the wiring used by
 * balance_encoder_imu, balance_encoder_pwmdir and the inverted_pendulum_lab,
 * so the duties measured here transfer directly to those programs.
 *
 * WIRING:
 *   INA -> GPIO18 (hw PWM ch0)   INB -> GPIO25 (DIR, plain GPIO)
 *   PS  -> GPIO24 (HIGH=active)  PWM/MODE pin -> 3.3V (selects EN/IN mode --
 *          tying it to GND selects IN/IN mode and breaks this control scheme)
 *   VCC -> 3.3V                  Encoder A -> GPIO17   Encoder B -> GPIO27
 *
 * DECAY MODE: EN/IN mode gives SLOW DECAY only. PWM on INA alternates
 * drive <-> short-brake, which is drive/brake by construction. Fast decay
 * (drive/coast) needs the outputs floated during the off-time, which on this
 * driver means pulling PS low -- and PS is a plain GPIO that cannot be
 * switched at 20 kHz from userspace. To characterize fast decay you must
 * rewire for IN/IN mode and use motor_characterization_pwmpwm.cpp instead.
 *
 * REQUIRED in /boot/firmware/config.txt (then reboot):
 *     dtoverlay=pwm-2chan,pin=18,func=2,pin2=19,func2=2
 * Only channel 0 is used here; the 2-chan overlay is harmless.
 *
 * Output: motor_characterization.csv
 * Runs the full protocol automatically after the encoder check: both
 * directions, RUNS_PER_DIRECTION sweeps each, then an averaged summary.
 * Manual commands afterwards: r=run sweep  f=flip dir  0=stop  q=quit
 */
#include <gpiod.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <iostream>
#include <poll.h>
#include <stdexcept>
#include <string>
#include <thread>
#include <algorithm>
#include <vector>
#include <iomanip>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

using namespace std::chrono_literals;

// ---- config ----
constexpr int GPIO_PS = 24, GPIO_DIR = 25, GPIO_ENC_A = 17, GPIO_ENC_B = 27;
constexpr int PWM_CHANNEL = 0, PWM_FREQ_HZ = 20000;
constexpr double SUPPLY_VOLTAGE = 12.0, GEAR_RATIO = 25.0;
// Pololu 20D encoder: 20 counts/motor-rev (older ESP32 disc was 12) -- match YOUR encoder.
constexpr double ENCODER_COUNTS_PER_MOTOR_REV = 20.0;
constexpr double COUNTS_PER_OUTPUT_REV = ENCODER_COUNTS_PER_MOTOR_REV * GEAR_RATIO;
constexpr double DUTY_START = 0.00, DUTY_END = 1.00, DUTY_STEP = 0.05;
constexpr int SETTLE_MS = 300, MEASURE_MS = 200;
constexpr double MOVE_THRESHOLD_CPS = 8.0;
// Automated protocol: repeat each direction this many times, then average.
constexpr int RUNS_PER_DIRECTION = 3;
constexpr int REST_MS = 1500;   // coast between runs so the motor settles

std::atomic<long long> encoder_count{0};
std::atomic<bool> running{true};

void writeSysfs(const std::string& path, const std::string& value)
{
    std::ofstream file(path);
    if (!file) throw std::runtime_error("Could not open: " + path);
    file << value;
    if (!file) throw std::runtime_error("Could not write to: " + path);
}

// ---- Hardware PWM via sysfs ----
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
        // Some kernels default polarity to "inversed" (duty_cycle=LOW time);
        // drive/brake math needs "normal" (duty_cycle=HIGH time).
        try { writeSysfs(pwm_path_ + "/polarity", "normal"); }
        catch (const std::exception& e) {
            std::cerr << "WARNING: could not set polarity=normal on channel "
                      << channel_ << ": " << e.what() << "\n";
        }
        writeSysfs(pwm_path_ + "/enable", "1");
        std::cout << "PWM channel " << channel_ << " initialized: period = " << period_ns_ << " ns\n";
    }

    // duty in [0,1] -- magnitude only; direction comes from the DIR line.
    void setDuty(double duty)
    {
        duty = std::clamp(duty, 0.0, 1.0);
        long long duty_ns = static_cast<long long>(duty * period_ns_);
        if (duty_ns >= period_ns_) duty_ns = period_ns_ - 1;
        writeSysfs(pwm_path_ + "/duty_cycle", std::to_string(duty_ns));
    }

    ~HardwarePWM()
    {
        try { writeSysfs(pwm_path_ + "/duty_cycle", "0"); writeSysfs(pwm_path_ + "/enable", "0"); }
        catch (...) {}
    }

private:
    int channel_;
    long long period_ns_;
    std::string pwm_path_;
};

// ---- GPIO: PS (power-save/enable) + DIR + encoder lines ----
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

        // PS: LOW=power-save/coast, HIGH=driver active (per datasheet)
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
    // Same mapping as balance_encoder_imu / pendulum.hpp, so a direction that
    // measures as "+1" here is the same physical direction there.
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

// ---- Encoder thread: full state-transition-table quadrature decode ----
void encoderThread(gpiod_line* lineA, gpiod_line* lineB)
{
    pollfd fds[2] = {{gpiod_line_event_get_fd(lineA), POLLIN, 0},
                     {gpiod_line_event_get_fd(lineB), POLLIN, 0}};

    // Sign flipped vs. raw quadrature math so +dir commands give +cps and
    // -dir commands give -cps (matches this hardware's rotation sense).
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

double measureCountsPerSecond(int measurement_ms)
{
    long long count_start = encoder_count.load();
    auto t_start = std::chrono::steady_clock::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(measurement_ms));
    auto t_end = std::chrono::steady_clock::now();
    long long count_end = encoder_count.load();
    double dt = std::chrono::duration<double>(t_end - t_start).count();
    return static_cast<double>(count_end - count_start) / dt;
}

// ---- Motor command: BD65496MUV EN/IN mode, PWM on INA + DIR on INB ----
// u is signed duty in [-1,1]: sign=direction, magnitude=speed.
//   INA(PWM)=HIGH, INB(DIR)=LOW  -> drive one way
//   INA(PWM)=HIGH, INB(DIR)=HIGH -> drive the other way
//   INA(PWM)=LOW                 -> short brake
// So PWM on INA is inherently DRIVE <-> BRAKE, i.e. slow decay.
void setMotor(GPIOController& gpio, HardwarePWM& pwm, double u)
{
    u = std::clamp(u, -1.0, 1.0);
    double m = std::fabs(u);

    if (m == 0.0) { pwm.setDuty(0.0); gpio.coast(); return; }

    // Drop PWM before changing direction so the bridge never flips mid-drive.
    pwm.setDuty(0.0);
    gpio.setDirection(u > 0.0 ? +1 : -1);
    gpio.enableDriver();
    pwm.setDuty(m);
}

struct Measurement { double cps; double rpm; };
// Negative field = that threshold was never crossed during the sweep.
struct SweepResult { double breakaway; double stop; };

Measurement doStep(GPIOController& gpio, HardwarePWM& pwm, std::ofstream& csv,
                   int direction, int run, double duty, const std::string& phase)
{
    setMotor(gpio, pwm, direction * duty);
    std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_MS));

    double cps = measureCountsPerSecond(MEASURE_MS);
    double rpm = cps / COUNTS_PER_OUTPUT_REV * 60.0;
    double approx_voltage = direction * duty * SUPPLY_VOLTAGE;

    std::cout << phase << ", dir=" << direction << ", duty=" << duty << ", V~=" << approx_voltage
              << ", cps=" << cps << ", rpm=" << rpm << '\n';
    csv << phase << "," << direction << "," << run << "," << duty << "," << approx_voltage << ","
        << cps << "," << rpm << "\n";
    csv.flush();
    return {cps, rpm};
}

SweepResult runSweep(GPIOController& gpio, HardwarePWM& pwm, int direction, int run, std::ofstream& csv)
{
    std::cout << "\n==== SWEEP direction = " << direction << "  (decay=SLOW, forced by EN/IN mode) ====\n";

    bool moved = false;
    double breakaway_duty = -1.0;
    for (double duty = DUTY_START; duty <= DUTY_END + 1e-9 && running; duty += DUTY_STEP)
    {
        Measurement m = doStep(gpio, pwm, csv, direction, run, duty, "UP");
        if (!moved && std::abs(m.cps) > MOVE_THRESHOLD_CPS)
        {
            moved = true; breakaway_duty = duty;
            std::cout << "\n*** BREAKAWAY: " << duty << " ***\n\n";
        }
    }

    bool stopped = false;
    double stop_duty = -1.0;
    for (double duty = DUTY_END; duty >= DUTY_START - 1e-9 && running; duty -= DUTY_STEP)
    {
        Measurement m = doStep(gpio, pwm, csv, direction, run, duty, "DOWN");
        if (!stopped && std::abs(m.cps) < MOVE_THRESHOLD_CPS)
        {
            stopped = true; stop_duty = duty;
            std::cout << "\n*** STOP: " << duty << " ***\n\n";
        }
    }

    setMotor(gpio, pwm, 0.0);

    std::cout << "\n------------------------------------\n"
              << "RESULT direction " << direction << "\n"
              << "Breakaway duty = " << breakaway_duty << "\n"
              << "Stop duty      = " << stop_duty << "\n";
    if (breakaway_duty >= 0 && stop_duty >= 0)
        std::cout << "Deadband width = " << breakaway_duty - stop_duty << "\n";
    std::cout << "------------------------------------\n";
    return {breakaway_duty, stop_duty};
}

// Averages only the runs where a threshold was actually found.
static bool meanValid(const std::vector<double>& xs, double& out)
{
    double sum = 0.0;
    int count = 0;
    for (double x : xs) if (x >= 0.0) { sum += x; ++count; }
    if (count == 0) return false;
    out = sum / count;
    return true;
}

static void printSummaryRow(int direction, const std::vector<SweepResult>& runs)
{
    std::vector<double> breakaways, stops, deadbands;
    for (const SweepResult& r : runs)
    {
        breakaways.push_back(r.breakaway);
        stops.push_back(r.stop);
        deadbands.push_back((r.breakaway >= 0.0 && r.stop >= 0.0) ? r.breakaway - r.stop : -1.0);
    }

    double mb = 0.0, ms = 0.0, md = 0.0;
    bool hb = meanValid(breakaways, mb), hs = meanValid(stops, ms), hd = meanValid(deadbands, md);

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "  " << std::setw(9) << direction << " | ";
    if (hb) std::cout << std::setw(14) << mb; else std::cout << std::setw(14) << "n/a";
    std::cout << " | ";
    if (hs) std::cout << std::setw(9) << ms; else std::cout << std::setw(9) << "n/a";
    std::cout << " | ";
    if (hd) std::cout << std::setw(13) << md; else std::cout << std::setw(13) << "n/a";
    std::cout << "\n";
    std::cout.unsetf(std::ios::floatfield);
}

// Runs the whole protocol unattended: every direction, RUNS_PER_DIRECTION each.
void runAutoCharacterization(GPIOController& gpio, HardwarePWM& pwm, std::ofstream& csv)
{
    const int directions[2] = {+1, -1};
    std::vector<SweepResult> results[2];

    for (int d = 0; d < 2 && running; ++d)
    {
        for (int run = 1; run <= RUNS_PER_DIRECTION && running; ++run)
        {
            std::cout << "\n##############################################\n"
                      << "  direction " << directions[d] << ", run " << run
                      << " of " << RUNS_PER_DIRECTION << "\n"
                      << "##############################################\n";

            results[d].push_back(runSweep(gpio, pwm, directions[d], run, csv));

            setMotor(gpio, pwm, 0.0);
            std::this_thread::sleep_for(std::chrono::milliseconds(REST_MS));
        }
    }

    setMotor(gpio, pwm, 0.0);

    if (!running)
        std::cout << "\n>>> ABORTED before the sequence finished -- partial results below <<<\n";

    std::cout << "\n==================================================\n"
              << "SUMMARY (slow decay, EN/IN mode)\n"
              << "averaged over up to " << RUNS_PER_DIRECTION << " runs per direction\n"
              << "==================================================\n"
              << "  Direction | Mean breakaway | Mean stop | Mean deadband\n"
              << "  ----------+----------------+-----------+--------------\n";
    for (int d = 0; d < 2; ++d)
        if (!results[d].empty()) printSummaryRow(directions[d], results[d]);
    std::cout << "==================================================\n"
              << "Per-step data in motor_characterization.csv (see the 'run' column).\n";
}

// ---- Terminal raw mode for single-keypress commands ----
termios g_orig_termios;
void enableRawMode()
{
    tcgetattr(STDIN_FILENO, &g_orig_termios);
    termios raw = g_orig_termios;
    raw.c_lflag &= ~(ICANON | ECHO);
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
                  << "Pololu 20D Motor Characterization (BD65496MUV)\n"
                  << "EN/IN mode: PWM on INA (GPIO18) + DIR on INB (GPIO" << GPIO_DIR << ")\n"
                  << "Raspberry Pi 4 / Ubuntu 24.04\n"
                  << "=============================================\n\n"
                  << "PWM frequency: " << PWM_FREQ_HZ << " Hz\n"
                  << "Gear ratio: " << GEAR_RATIO << ":1\n"
                  << "Encoder motor CPR: " << ENCODER_COUNTS_PER_MOTOR_REV << "\n"
                  << "Encoder output CPR: " << COUNTS_PER_OUTPUT_REV << "\n"
                  << "Decay mode: SLOW only (EN/IN mode PWMs drive<->brake).\n"
                  << "  For fast decay, rewire for IN/IN mode and use the pwmpwm build.\n\n";

        GPIOController gpio;
        HardwarePWM pwm(PWM_CHANNEL, PWM_FREQ_HZ);   // INA
        setMotor(gpio, pwm, 0.0);

        std::thread encoder_thread(encoderThread, gpio.encoderA(), gpio.encoderB());

        std::ofstream csv("motor_characterization.csv");
        if (!csv) throw std::runtime_error("Could not create CSV file.");
        csv << "phase,direction,run,duty,approx_voltage,counts_per_second,output_rpm\n";

        std::cout << "Motor is currently COASTING.\n\n"
                  << "Encoder test mode.\nTurn the output shaft by hand.\n"
                  << "Press ENTER when finished.\n\n";

        std::atomic<bool> encoder_test_running{true};
        std::thread encoder_print_thread([&]() {
            long long previous = encoder_count.load();
            while (encoder_test_running) {
                long long current = encoder_count.load();
                if (current != previous) {
                    std::cout << "\rEncoder count: " << current << "          " << std::flush;
                    previous = current;
                }
                std::this_thread::sleep_for(20ms);
            }
        });
        std::cin.get();
        encoder_test_running = false;
        encoder_print_thread.join();

        std::cout << "\n\nStarting automated characterization: " << RUNS_PER_DIRECTION
                  << " runs in each direction, no further input needed.\n"
                  << ">>> Keep clear of the motor and check the shaft can spin freely. <<<\n"
                  << "Ctrl-C aborts and still prints whatever completed.\n";
        std::this_thread::sleep_for(3s);

        runAutoCharacterization(gpio, pwm, csv);

        int dir = +1;
        std::cout << "\nManual commands, if you want extra runs:\n"
                  << "  r=run one sweep  f=flip dir (currently +1)  0=stop  q=quit\n";

        enableRawMode();
        while (running)
        {
            char c;
            if (read(STDIN_FILENO, &c, 1) > 0)
            {
                if (c == 'r') { runSweep(gpio, pwm, dir, 0, csv);
                    std::cout << "\nCommands: r=run  f=flip dir  0=stop  q=quit\n"; }
                else if (c == 'f') { dir = -dir; std::cout << "dir=" << dir << "\n"; }
                else if (c == '0') { setMotor(gpio, pwm, 0.0); std::cout << "stopped\n"; }
                else if (c == 'd') { std::cout << "decay is SLOW only in EN/IN mode -- see header comment\n"; }
                else if (c == 'q') { running = false; }
            }
            std::this_thread::sleep_for(50ms);
        }
        disableRawMode();

        setMotor(gpio, pwm, 0.0);
        running = false;
        encoder_thread.join();

        std::cout << "\n=============================================\n"
                  << "Done. Data saved to motor_characterization.csv\n"
                  << "=============================================\n";
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
