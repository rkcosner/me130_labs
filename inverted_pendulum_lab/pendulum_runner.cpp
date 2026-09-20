/* ============================================================================
 * pendulum_runner.cpp -- IMU pendulum balance harness (provided, not part of
 * the assignment)
 * ============================================================================
 * Owns main() and the full control loop: IMU sensing + complementary filter,
 * motor driving, encoder, keyboard interface, arm/disarm safety, telemetry.
 * It calls into computePID() / applyDeadband(), which are implemented by you
 * in controller.cpp (or controller_solution.cpp for the reference build) --
 * see pendulum.hpp for the interface between this file and that one.
 *
 * You should not need to modify this file for the lab.
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
 *
 * Commands (type letter, then value if needed, then Enter):
 *   z=set-upright  e=arm  x=disarm  s=flip-motor-sign
 *   r=flip-rate-sign  t=toggle-integral  q=quit
 *   p<Kp>  d<Kd>  i<Ki>  w<integral_term_limit>  f<deadband>
 *   c<complementary_filter 0..1>
 * ==========================================================================*/
#include "pendulum.hpp"

// ---- sensor-fusion / motor-mapping tuning knobs (provided, not part of the
// PID assignment, but tunable live from the keyboard like everything else) ----
double MOTOR_SIGN = -1.0;
double RATE_SIGN = 1.0;
double COMP = 0.995;             // complementary-filter gyro weight

int runPendulumLab()
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

        MPU6050 imu;
        double gyroBias = imu.calibrateGyroBiasRadPerSec();

        float ax0 = 0.0f, ay0 = 0.0f, gz0 = 0.0f;
        if (!imu.read(ax0, ay0, gz0))
            throw std::runtime_error("Initial MPU6050 read failed.");

        // Started only after IMU init succeeds: nothing before this point
        // needs the encoder, and starting it earlier meant a throw during
        // IMU setup would destroy this joinable thread while it was still
        // running, which calls std::terminate() instead of reporting the
        // real error.
        std::thread encoder_thread(encoderThread, gpio.encoderA(), gpio.encoderB());

        double thetaOffset = 0.0;
        double theta = std::atan2(static_cast<double>(ax0), static_cast<double>(ay0));
        double thetaDot = 0.0, thetaIntegral = 0.0;
        int imuFail = 0;
        bool armed = false;

        std::cout << "Ready. Hold rod UPRIGHT, 'z' to set vertical, keep holding, 'e' to arm, release.\n"
                  << "Cmds: z=upright  e=arm  x=disarm  s=flip-motor-sign  r=flip-rate-sign\n"
                  << "      t=toggle-integral  p<Kp>  d<Kd>  i<Ki>  w<windup_limit>\n"
                  << "      f<deadband>  c<COMP>  q=quit\n\n";

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
                    if (std::fabs(theta) < ARM_WINDOW) { armed = true; std::cout << ">> ARMED\n"; }
                    else std::cout << ">> too far from upright to arm\n";
                } else if (c == 'x') {
                    armed = false; thetaIntegral = 0.0; setMotor(gpio, pwm, 0.0);
                    std::cout << ">> DISARMED\n";
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
                } else if (c == 'p' || c == 'd' || c == 'i' || c == 'w' || c == 'f' || c == 'c') {
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

            // ---- IMU sensing: complementary-filter structure (provided) ----
            float accX = 0.0f, accY = 0.0f, gyroZ_dps = 0.0f;
            if (!imu.read(accX, accY, gyroZ_dps))
            {
                if (++imuFail > IMU_FAIL_MAX)
                {
                    if (armed) std::cout << ">> IMU lost -- motor off\n";
                    armed = false;
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
                armed = false;
                setMotor(gpio, pwm, 0.0);
            }

            // ---- student PID controller ----
            double u = 0.0;
            double integralTerm = 0.0;
            if (armed)
            {
                u = computePID(theta, thetaDot, dt, thetaIntegral, integralEnabled, integralTerm);
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
                          << "  enc=" << cnt << "  imuFail=" << imuFail
                          << "  " << (armed ? "ARMED" : "off") << "\n";
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

int main()
{
    return runPendulumLab();
}