"""Lab 3 -- frequency response.

Rod HANGING DOWN. Drives a sine at each frequency in turn and logs it, so
freq_response.py can fit gain and phase per frequency and draw the Bode plot.

    ros2 launch me130_pendulum frequency_response.launch.py deadband:=0.065

Writes sweep_<timestamp>.csv.

Hold time per frequency is computed as skip_s + cycles/f, so low frequencies
are driven longer. Keep skip_s equal to SKIP_S in freq_response.py, or the
low-frequency points get discarded as transient.

Blocks:  imu_node ------\\
         encoder_node ---> logger_node
         sine_sweep_node -> motor_node -> hardware
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    deadband = LaunchConfiguration("deadband")
    motor_sign = LaunchConfiguration("motor_sign")

    return LaunchDescription([
        DeclareLaunchArgument("deadband", default_value="0.1",
                              description="from lab 1; 0.0 means no compensation"),
        DeclareLaunchArgument("motor_sign", default_value="1.0"),
        DeclareLaunchArgument("amplitude", default_value="0.10",
                              description="sine amplitude, pre-deadband duty"),
        DeclareLaunchArgument("skip_s", default_value="10.0",
                              description="must match SKIP_S in freq_response.py"),
        DeclareLaunchArgument("cycles", default_value="4.0",
                              description="usable cycles wanted after the transient"),
        DeclareLaunchArgument("output_dir", default_value="."),

        # The rod hangs at rest for this test, so take that pose as theta = 0.
        Node(package="me130_pendulum", executable="imu_node", name="imu_node",
             output="screen",
             parameters=[{"zero_on_start": True}],
             on_exit=Shutdown()),
        Node(package="me130_pendulum", executable="encoder_node", name="encoder_node",
             output="screen",
             on_exit=Shutdown()),
        Node(package="me130_pendulum", executable="motor_node", name="motor_node",
             output="screen",
             parameters=[{"deadband": deadband, "motor_sign": motor_sign}],
             on_exit=Shutdown()),

        Node(package="me130_pendulum", executable="logger_node", name="logger_node",
             output="screen",
             parameters=[{"prefix": "sweep",
                          "output_dir": LaunchConfiguration("output_dir")}]),

        Node(package="me130_pendulum", executable="sine_sweep_node",
             name="sine_sweep_node", output="screen",
             parameters=[{"amplitude": LaunchConfiguration("amplitude"),
                          "skip_s": LaunchConfiguration("skip_s"),
                          "cycles": LaunchConfiguration("cycles"),
                          # motor_node + logger_node must both be connected
                          # before the first frequency, or the log loses it.
                          "wait_for_subscribers": 2}],
             on_exit=Shutdown()),
    ])
