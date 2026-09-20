// motor_characterization_node -- measures the friction deadband.
//
// Sweeps duty up until the shaft breaks away, then back down until it stops,
// in both directions, several times, and averages. The deadband it reports is
// the `deadband` parameter the other three labs take.
//
// This test needs the ENCODER only, which is why encoder_node is a separate
// block: the launch file for this lab never starts the IMU.
//
// The sweep is inherently sequential, so it runs on its own thread while the
// executor keeps servicing the encoder subscription.
#include "me130_pendulum/hardware.hpp"
#include "me130_interfaces/msg/encoder_state.hpp"
#include "me130_interfaces/msg/motor_command.hpp"

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

class MotorCharacterizationNode : public rclcpp::Node
{
public:
    MotorCharacterizationNode() : rclcpp::Node("motor_characterization_node")
    {
        duty_start_ = declare_parameter("duty_start", 0.0);
        duty_end_ = declare_parameter("duty_end", 1.0);
        duty_step_ = declare_parameter("duty_step", 0.05);
        settle_ms_ = declare_parameter("settle_ms", 300);
        measure_ms_ = declare_parameter("measure_ms", 200);
        threshold_cps_ = declare_parameter("move_threshold_cps", 8.0);
        runs_ = declare_parameter("runs_per_direction", 3);
        supply_v_ = declare_parameter("supply_voltage", 12.0);
        csv_path_ = declare_parameter("output_csv", std::string("motor_characterization.csv"));

        pub_ = create_publisher<me130_interfaces::msg::MotorCommand>(
            "/motor/command", rclcpp::SensorDataQoS());

        sub_ = create_subscription<me130_interfaces::msg::EncoderState>(
            "/encoder/state", rclcpp::SensorDataQoS(),
            [this](const me130_interfaces::msg::EncoderState::SharedPtr m) {
                cps_.store(m->counts_per_sec);
                seen_.store(true);
                fresh_.store(true);
            });

        worker_ = std::thread(&MotorCharacterizationNode::run, this);
    }

    ~MotorCharacterizationNode() override
    {
        stop_.store(true);
        if (worker_.joinable()) worker_.join();
    }

private:
    void command(double u)
    {
        me130_interfaces::msg::MotorCommand msg;
        msg.header.stamp = now();
        msg.u = u;
        msg.mode = (std::fabs(u) < 1e-9) ? "coast" : "step";
        msg.step_duty = u;
        msg.segment = segment_;
        pub_->publish(msg);
    }

    // Hold a duty, let it settle, then read the rate the encoder reports.
    double measure(int direction, double duty, const char *phase, int run)
    {
        command(direction * duty);
        std::this_thread::sleep_for(std::chrono::milliseconds(settle_ms_));
        std::this_thread::sleep_for(std::chrono::milliseconds(measure_ms_));

        // Guard against a stale reading being averaged into the thresholds if
        // encoder_node dies part-way through a sweep.
        if (fresh_.exchange(false)) stale_ = 0; else ++stale_;
        if (stale_ > 10 && !stop_.load())
        {
            RCLCPP_FATAL(get_logger(), "/encoder/state stopped updating -- aborting.");
            stop_.store(true);
        }

        const double cps = cps_.load();
        const double rpm = cps / me130::kCountsPerOutputRev * 60.0;
        const double volts = direction * duty * supply_v_;

        RCLCPP_INFO(get_logger(), "%s dir=%+d run=%d duty=%.2f  cps=%8.1f  rpm=%6.1f",
                    phase, direction, run, duty, cps, rpm);
        csv_ << phase << ',' << direction << ',' << run << ',' << duty << ',' << volts
             << ',' << cps << ',' << rpm << '\n';
        csv_.flush();
        return cps;
    }

    // One up-sweep and one down-sweep. Returns {breakaway, stop}; a negative
    // entry means that threshold was never crossed.
    std::pair<double, double> sweep(int direction, int run)
    {
        ++segment_;
        double breakaway = -1.0, stop = -1.0;

        for (double d = duty_start_; d <= duty_end_ + 1e-9 && !stop_.load(); d += duty_step_)
            if (breakaway < 0.0 && std::fabs(measure(direction, d, "UP", run)) > threshold_cps_)
            {
                breakaway = d;
                RCLCPP_INFO(get_logger(), "*** BREAKAWAY dir=%+d: %.3f ***", direction, d);
            }

        for (double d = duty_end_; d >= duty_start_ - 1e-9 && !stop_.load(); d -= duty_step_)
            if (stop < 0.0 && std::fabs(measure(direction, d, "DOWN", run)) < threshold_cps_)
            {
                stop = d;
                RCLCPP_INFO(get_logger(), "*** STOP dir=%+d: %.3f ***", direction, d);
            }

        command(0.0);
        return {breakaway, stop};
    }

    static double meanValid(const std::vector<double> &xs)
    {
        double sum = 0.0;
        int n = 0;
        for (double x : xs) if (x >= 0.0) { sum += x; ++n; }
        return n ? sum / n : -1.0;
    }

    void run()
    {
        // Let the executor start and the first encoder samples arrive.
        for (int i = 0; i < 100 && !seen_.load() && !stop_.load(); ++i)
            std::this_thread::sleep_for(50ms);

        // Without the encoder there is nothing to measure. Sweeping anyway
        // reads friction thresholds off sensor noise and prints a confident
        // wrong deadband, which is far worse than failing outright.
        if (!seen_.load())
        {
            RCLCPP_FATAL(get_logger(),
                         "No /encoder/state after 5 s -- aborting. encoder_node is "
                         "probably dead; the usual cause is permissions:\n"
                         "    sudo ./me130_permissions.sh   (then log out and back in)");
            rclcpp::shutdown();
            return;
        }

        csv_.open(csv_path_);
        if (!csv_)
        {
            RCLCPP_FATAL(get_logger(), "Could not open %s", csv_path_.c_str());
            rclcpp::shutdown();
            return;
        }
        csv_ << "phase,direction,run,duty,approx_voltage,counts_per_second,output_rpm\n";

        std::vector<double> brk[2], stp[2];
        const int directions[2] = {+1, -1};

        for (int d = 0; d < 2 && !stop_.load(); ++d)
            for (int run = 1; run <= runs_ && !stop_.load(); ++run)
            {
                RCLCPP_INFO(get_logger(), "==== direction %+d, run %d/%d ====",
                            directions[d], run, runs_);
                auto [b, s] = sweep(directions[d], run);
                brk[d].push_back(b);
                stp[d].push_back(s);
                std::this_thread::sleep_for(1s);
            }

        command(0.0);
        csv_.flush();
        csv_.close();

        if (stop_.load())
            RCLCPP_ERROR(get_logger(),
                         "RUN ABORTED -- the numbers below are incomplete. Do not use them.");
        RCLCPP_INFO(get_logger(), "======== SUMMARY (averaged over %d runs) ========", runs_);
        double all_breakaway = 0.0;
        int counted = 0;
        for (int d = 0; d < 2; ++d)
        {
            const double mb = meanValid(brk[d]), ms = meanValid(stp[d]);
            RCLCPP_INFO(get_logger(), "  dir %+d   breakaway=%.3f   stop=%.3f   deadband=%.3f",
                        directions[d], mb, ms,
                        (mb >= 0.0 && ms >= 0.0) ? mb - ms : -1.0);
            if (mb >= 0.0) { all_breakaway += mb; ++counted; }
        }
        if (counted)
        {
            const double d = all_breakaway / counted;
            RCLCPP_INFO(get_logger(), "Use this for the other labs:");
            RCLCPP_INFO(get_logger(), "  deadband:=%.3f", d);
        }
        RCLCPP_INFO(get_logger(), "Per-step data in %s", csv_path_.c_str());
        rclcpp::shutdown();
    }

    double duty_start_{0.0}, duty_end_{1.0}, duty_step_{0.05};
    int settle_ms_{300}, measure_ms_{200}, runs_{3};
    double threshold_cps_{8.0}, supply_v_{12.0};
    std::string csv_path_;
    std::ofstream csv_;
    int segment_{0};

    std::atomic<double> cps_{0.0};
    std::atomic<bool> seen_{false};
    std::atomic<bool> fresh_{false};
    int stale_{0};
    std::atomic<bool> stop_{false};
    std::thread worker_;

    rclcpp::Publisher<me130_interfaces::msg::MotorCommand>::SharedPtr pub_;
    rclcpp::Subscription<me130_interfaces::msg::EncoderState>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorCharacterizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
