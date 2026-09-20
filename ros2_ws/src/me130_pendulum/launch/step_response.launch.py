"""Lab 2 -- step response (free and forced).

Rod HANGING DOWN. Holds each duty in turn, then releases and lets the rod ring
down, so every segment gives a forced response followed by a free response.

    ros2 launch me130_pendulum step_response.launch.py deadband:=0.065

Writes steps_<timestamp>.csv; plot it with step_response.py, which also
identifies the second-order model.

Blocks:  imu_node ------\\
         encoder_node ---> logger_node
         step_sequence_node -> motor_node -> hardware
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
        DeclareLaunchArgument("hold_s", default_value="1.0"),
        DeclareLaunchArgument("rest_s", default_value="2.5",
                              description="raise this if the rod is still ringing "
                                          "when the next step starts"),
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
             parameters=[{"prefix": "steps",
                          "output_dir": LaunchConfiguration("output_dir")}]),

        Node(package="me130_pendulum", executable="step_sequence_node",
             name="step_sequence_node", output="screen",
             parameters=[{"hold_s": LaunchConfiguration("hold_s"),
                          "rest_s": LaunchConfiguration("rest_s"),
                          # motor_node + logger_node must both be connected
                          # before the first step, or the log loses segment 1.
                          "wait_for_subscribers": 2}],
             on_exit=Shutdown()),
    ])
