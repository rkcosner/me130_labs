// sine_sweep_node -- drives the open-loop frequency sweep, hands-off.
//
// Rod HANGING DOWN. u = amplitude * sin(2*pi*f*t), t measured from the start
// of each frequency so the phase is well defined.
//
// Hold time is COMPUTED, not fixed: freq_response.py throws away the first
// skip_s seconds as transient and then needs whole cycles to fit, so a low
// frequency has to be driven for longer. Keep skip_s equal to that script's
// SKIP_S or the low-frequency points will be rejected.
#include "me130_interfaces/msg/motor_command.hpp"
#include "me130_interfaces/msg/pendulum_angle.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <numeric>
#include <vector>

class SineSweepNode : public rclcpp::Node
{
public:
    SineSweepNode() : rclcpp::Node("sine_sweep_node")
    {
        // Denser through the resonance, where the gain and phase actually move;
        // sparser on the flat ends where extra points buy little.
        freqs_ = declare_parameter("frequencies_hz",
                                   std::vector<double>{0.30, 0.45, 0.60, 0.75,
                                                       0.85, 0.92, 1.00, 1.08,
                                                       1.16, 1.30, 1.50, 1.80,
                                                       2.20, 2.70, 3.20, 3.80});
        amplitude_ = declare_parameter("amplitude", 0.10);
        skip_s_ = declare_parameter("skip_s", 10.0);
        cycles_ = declare_parameter("cycles", 4.0);
        settle_s_ = declare_parameter("settle_s", 2.0);
        const double rate_hz = declare_parameter("publish_rate_hz", 500.0);
        // A hanging rod driven at resonance can swing a long way -- 50 deg was
        // measured on this rig at 1 Hz. The monolith aborted past THETA_MAX;
        // nothing here did, so re-add the guard. Exceeding it skips that one
        // frequency rather than binning the whole sweep.
        theta_max_deg_ = declare_parameter("theta_max_deg", 75.0);
        // DDS discovery is NOT instant -- measured at ~2.5 s on this Pi. A
        // fixed delay is a guess; instead wait until the expected consumers
        // have actually connected to this publisher, then settle briefly.
        // Without this the logger misses the whole first segment, which then
        // reads as a short malformed step (or vanishes from the analysis).
        //   1 = motor_node only, 2 = motor_node + logger_node.
        required_subs_ = declare_parameter("wait_for_subscribers", 1);
        wait_timeout_s_ = declare_parameter("wait_timeout_s", 15.0);
        startup_s_ = declare_parameter("startup_delay_s", 0.5);

        if (freqs_.empty())
        {
            RCLCPP_ERROR(get_logger(), "frequencies_hz is empty -- nothing to run.");
            rclcpp::shutdown();
            return;
        }
        for (double f : freqs_)
        {
            if (f <= 0.0)
            {
                RCLCPP_ERROR(get_logger(), "frequencies_hz must all be > 0 (got %.3f).", f);
                rclcpp::shutdown();
                return;
            }
        }

        pub_ = create_publisher<me130_interfaces::msg::MotorCommand>(
            "/motor/command", rclcpp::SensorDataQoS());

        angle_sub_ = create_subscription<me130_interfaces::msg::PendulumAngle>(
            "/pendulum/angle", rclcpp::SensorDataQoS(),
            [this](const me130_interfaces::msg::PendulumAngle::SharedPtr m) {
                theta_ = m->theta_rad;
            });

        double total = 0.0;
        for (double f : freqs_) total += holdSeconds(f) + settle_s_;

        dt_ = 1.0 / rate_hz;
        timer_ = create_wall_timer(std::chrono::duration<double>(dt_), [this]() { tick(); });

        RCLCPP_INFO(get_logger(), "sine sweep: %zu frequencies, amp=%.3f, ~%.0f s total. "
                                  "Rod should be HANGING DOWN.",
                    freqs_.size(), amplitude_, total);
        RCLCPP_INFO(get_logger(), "freq 1/%zu  %.3f Hz for %.1f s",
                    freqs_.size(), freqs_[0], holdSeconds(freqs_[0]));
    }

private:
    double holdSeconds(double f) const { return skip_s_ + cycles_ / f; }

    void publish(double u, const char *mode, double freq)
    {
        me130_interfaces::msg::MotorCommand msg;
        msg.header.stamp = now();
        msg.u = u;
        msg.mode = mode;
        msg.freq_hz = freq;
        msg.amplitude = (freq > 0.0) ? amplitude_ : 0.0;
        msg.segment = index_ + 1;
        pub_->publish(msg);
    }

    void tick()
    {
        if (!started_)
        {
            startup_elapsed_ += dt_;
            publish(0.0, "coast", 0.0);

            const int subs = static_cast<int>(pub_->get_subscription_count());
            if (!connected_ && subs >= required_subs_)
            {
                connected_ = true;
                settle_at_ = startup_elapsed_ + startup_s_;
                RCLCPP_INFO(get_logger(), "%d subscriber(s) connected after %.2f s",
                            subs, startup_elapsed_);
            }
            if (!connected_)
            {
                if (startup_elapsed_ >= wait_timeout_s_)
                {
                    RCLCPP_WARN(get_logger(),
                                "Only %d of %d subscriber(s) after %.1f s -- starting anyway. "
                                "Any log from this run may be missing its first segment.",
                                subs, required_subs_, startup_elapsed_);
                    started_ = true;
                }
                return;
            }
            if (startup_elapsed_ < settle_at_) return;
            started_ = true;
        }
        const double f = freqs_[index_];
        elapsed_ += dt_;

        if (driving_)
        {
            if (std::fabs(theta_) > theta_max_deg_ * M_PI / 180.0)
            {
                RCLCPP_WARN(get_logger(),
                            "|theta| = %.1f deg exceeded theta_max (%.1f) at %.3f Hz -- "
                            "skipping this frequency. Lower `amplitude` near resonance.",
                            std::fabs(theta_) * 180.0 / M_PI, theta_max_deg_, f);
                driving_ = false;
                elapsed_ = 0.0;
                publish(0.0, "coast", 0.0);
                return;
            }
            if (elapsed_ < holdSeconds(f))
            {
                publish(amplitude_ * std::sin(2.0 * M_PI * f * elapsed_), "sine", f);
                return;
            }
            driving_ = false;
            elapsed_ = 0.0;
        }

        if (elapsed_ < settle_s_) { publish(0.0, "coast", 0.0); return; }

        if (++index_ >= freqs_.size())
        {
            publish(0.0, "coast", 0.0);
            RCLCPP_INFO(get_logger(), "frequency sweep COMPLETE");
            timer_->cancel();
            rclcpp::shutdown();
            return;
        }

        driving_ = true;
        elapsed_ = 0.0;
        RCLCPP_INFO(get_logger(), "freq %zu/%zu  %.3f Hz for %.1f s",
                    index_ + 1, freqs_.size(), freqs_[index_], holdSeconds(freqs_[index_]));
    }

    std::vector<double> freqs_;
    double amplitude_{0.10}, skip_s_{10.0}, cycles_{4.0}, settle_s_{2.0};
    double theta_max_deg_{75.0}, theta_{0.0};
    rclcpp::Subscription<me130_interfaces::msg::PendulumAngle>::SharedPtr angle_sub_;
    double dt_{0.002}, elapsed_{0.0};
    double startup_s_{0.5}, startup_elapsed_{0.0}, settle_at_{0.0}, wait_timeout_s_{15.0};
    int required_subs_{1};
    bool connected_{false}, started_{false};
    std::size_t index_{0};
    bool driving_{true};
    rclcpp::Publisher<me130_interfaces::msg::MotorCommand>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SineSweepNode>());
    rclcpp::shutdown();
    return 0;
}
