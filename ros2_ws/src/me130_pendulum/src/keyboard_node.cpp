// keyboard_node -- the old single-keystroke console, as a ROS node.
//
// Typing `ros2 service call ...` while balancing a pendulum is not usable, so
// this puts the pre-ROS keyboard interface back without putting the UI inside
// the controller. It owns no hardware and no control law: every key turns into
// a service call or a parameter set on another node.
//
// Run it in its OWN terminal, alongside the launch file. ros2 launch does not
// give child processes a usable stdin, so it cannot live inside the launch:
//
//     terminal 1:  ros2 launch me130_pendulum balance.launch.py deadband:=0.065
//     terminal 2:  ros2 run me130_pendulum keyboard_node
//
// Keys (value commands: letter, number, Enter -- e.g. p10.0<Enter>):
//   z = zero theta here      e = arm            x = disarm
//   p<Kp>  d<Kd>  i<Ki>  w<integral limit>      -> controller_node
//   f<deadband>  s = flip motor sign            -> motor_node
//   t = telemetry on/off     q = quit
#include "me130_interfaces/msg/pendulum_angle.hpp"

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <termios.h>
#include <unistd.h>

using namespace std::chrono_literals;

namespace
{
termios g_original_termios;
bool g_raw_mode = false;

void enableRawMode()
{
    if (!isatty(STDIN_FILENO)) return;
    tcgetattr(STDIN_FILENO, &g_original_termios);
    termios raw = g_original_termios;
    raw.c_lflag &= ~ICANON;          // no line buffering...
    raw.c_cc[VMIN] = 0;              // ...but keep ECHO on, so typed numbers
    raw.c_cc[VTIME] = 0;             //    are visible, like the old console
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    g_raw_mode = true;
}

void disableRawMode()
{
    if (!g_raw_mode) return;
    tcsetattr(STDIN_FILENO, TCSANOW, &g_original_termios);
    g_raw_mode = false;
}
}  // namespace

class KeyboardNode : public rclcpp::Node
{
public:
    KeyboardNode() : rclcpp::Node("keyboard_node")
    {
        controller_ = declare_parameter("controller_node", std::string("/controller_node"));
        motor_ = declare_parameter("motor_node", std::string("/motor_node"));
        imu_ = declare_parameter("imu_node", std::string("/imu_node"));

        zero_ = create_client<std_srvs::srv::Trigger>(imu_ + "/zero");
        arm_ = create_client<std_srvs::srv::Trigger>(controller_ + "/arm");
        disarm_ = create_client<std_srvs::srv::Trigger>(controller_ + "/disarm");

        controller_params_ =
            std::make_shared<rclcpp::AsyncParametersClient>(this, controller_);
        motor_params_ = std::make_shared<rclcpp::AsyncParametersClient>(this, motor_);

        angle_sub_ = create_subscription<me130_interfaces::msg::PendulumAngle>(
            "/pendulum/angle", rclcpp::SensorDataQoS(),
            [this](const me130_interfaces::msg::PendulumAngle::SharedPtr m) {
                theta_deg_ = m->theta_rad * 180.0 / M_PI;
                rate_dps_ = m->theta_dot_rad_s * 180.0 / M_PI;
                have_angle_ = true;
            });

        enableRawMode();
        printHelp();

        keys_ = create_wall_timer(20ms, [this]() { pollKeys(); });
        telem_ = create_wall_timer(500ms, [this]() { printTelemetry(); });
    }

    ~KeyboardNode() override { disableRawMode(); }

private:
    void printHelp()
    {
        std::printf(
            "\n"
            "  ME130 pendulum console\n"
            "  ----------------------\n"
            "  z  zero theta at the current position (hold the rod there first)\n"
            "  e  arm          x  disarm          q  quit\n"
            "  s  flip motor sign                 t  telemetry on/off\n"
            "  p<Kp>  d<Kd>  i<Ki>  w<integral limit>   e.g.  p10.0<Enter>\n"
            "  f<deadband>\n\n"
            "  Gains start at ZERO. Arming does nothing until you set them.\n\n");
        std::fflush(stdout);
    }

    void call(const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr &client,
              const char *what)
    {
        if (!client->wait_for_service(200ms))
        {
            std::printf(">> %s: service unavailable (is the launch file running?)\n", what);
            std::fflush(stdout);
            return;
        }
        auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
        // Async: a blocking call from inside a spin callback would deadlock.
        client->async_send_request(
            req, [what](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
                const auto res = future.get();
                std::printf(">> %s: %s\n", what,
                            res->message.empty() ? (res->success ? "ok" : "refused")
                                                 : res->message.c_str());
                std::fflush(stdout);
            });
    }

    void setParam(const std::shared_ptr<rclcpp::AsyncParametersClient> &client,
                  const std::string &node, const std::string &name, double value)
    {
        if (!client->service_is_ready())
        {
            std::printf(">> %s not reachable -- is it running?\n", node.c_str());
            std::fflush(stdout);
            return;
        }
        // Report from the result, not optimistically before it lands: a set
        // can be rejected, and claiming success either way is worse than slow.
        const std::string label = node + " " + name;
        client->set_parameters(
            {rclcpp::Parameter(name, value)},
            [label, value](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> f) {
                const auto results = f.get();
                if (!results.empty() && results[0].successful)
                    std::printf(">> %s = %g\n", label.c_str(), value);
                else
                    std::printf(">> %s REJECTED%s%s\n", label.c_str(),
                                (results.empty() || results[0].reason.empty()) ? "" : ": ",
                                (results.empty() || results[0].reason.empty())
                                    ? "" : results[0].reason.c_str());
                std::fflush(stdout);
            });
    }

    void applyPending()
    {
        double v = 0.0;
        try { v = std::stod(pending_arg_); }
        catch (...) { std::printf(">> bad value, ignored\n"); std::fflush(stdout); return; }

        switch (pending_)
        {
        case 'p': setParam(controller_params_, controller_, "kp", v); break;
        case 'd': setParam(controller_params_, controller_, "kd", v); break;
        case 'i': setParam(controller_params_, controller_, "ki", v); break;
        case 'w': setParam(controller_params_, controller_, "integral_limit", v); break;
        case 'f': setParam(motor_params_, motor_, "deadband", v); break;
        default: break;
        }
    }

    void pollKeys()
    {
        char c;
        while (read(STDIN_FILENO, &c, 1) > 0)
        {
            if (pending_ != 0)
            {
                if (c == '\n' || c == '\r') { applyPending(); pending_ = 0; pending_arg_.clear(); }
                else pending_arg_ += c;
                continue;
            }

            switch (c)
            {
            case 'z': call(zero_, "zero"); break;
            case 'e': call(arm_, "arm"); break;
            case 'x': call(disarm_, "disarm"); break;
            case 's':
                motor_sign_ = -motor_sign_;
                setParam(motor_params_, motor_, "motor_sign", motor_sign_);
                break;
            case 't':
                telemetry_ = !telemetry_;
                std::printf(">> telemetry %s\n", telemetry_ ? "ON" : "OFF");
                std::fflush(stdout);
                break;
            case 'q': rclcpp::shutdown(); break;
            case 'p': case 'd': case 'i': case 'w': case 'f':
                pending_ = c; pending_arg_.clear(); break;
            default: break;
            }
        }
    }

    void printTelemetry()
    {
        // Stay quiet mid-command so a half-typed number is not buried.
        if (!telemetry_ || !have_angle_ || pending_ != 0) return;
        std::printf("   theta=%+7.2f deg   rate=%+8.2f deg/s\n", theta_deg_, rate_dps_);
        std::fflush(stdout);
    }

    std::string controller_, motor_, imu_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr zero_, arm_, disarm_;
    std::shared_ptr<rclcpp::AsyncParametersClient> controller_params_, motor_params_;
    rclcpp::Subscription<me130_interfaces::msg::PendulumAngle>::SharedPtr angle_sub_;
    rclcpp::TimerBase::SharedPtr keys_, telem_;

    char pending_{0};
    std::string pending_arg_;
    double motor_sign_{1.0};
    double theta_deg_{0.0}, rate_dps_{0.0};
    bool have_angle_{false}, telemetry_{true};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    {
        auto node = std::make_shared<KeyboardNode>();
        rclcpp::spin(node);
    }
    disableRawMode();
    rclcpp::shutdown();
    std::printf("\n");
    return 0;
}
