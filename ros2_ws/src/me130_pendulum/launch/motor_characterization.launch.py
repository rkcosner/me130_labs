"""Lab 1 -- motor characterization.

Measures the friction deadband: the duty below which the motor does not turn.
The number this prints is the `deadband:=` argument the other three labs take.

    ros2 launch me130_pendulum motor_characterization.launch.py

Detach the pendulum first -- the shaft sweeps to full duty in both directions.

Blocks:  encoder_node -> motor_characterization_node -> motor_node -> hardware

The IMU is deliberately NOT launched: this test measures the motor, and the
rod angle has nothing to do with it.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    runs = LaunchConfiguration("runs_per_direction")
    motor_sign = LaunchConfiguration("motor_sign")
    output_csv = LaunchConfiguration("output_csv")

    return LaunchDescription([
        DeclareLaunchArgument("runs_per_direction", default_value="3",
                              description="sweeps per direction, averaged"),
        DeclareLaunchArgument("motor_sign", default_value="1.0",
                              description="flip to -1.0 if +duty spins the wrong way"),
        DeclareLaunchArgument("output_csv", default_value="motor_characterization.csv"),

        Node(package="me130_pendulum", executable="encoder_node", name="encoder_node",
             output="screen",
             on_exit=Shutdown()),

        # deadband is 0.0 here ON PURPOSE. This lab MEASURES the deadband, so
        # the motor must be driven raw; compensating first would hide it.
        Node(package="me130_pendulum", executable="motor_node", name="motor_node",
             output="screen",
             parameters=[{"deadband": 0.0, "motor_sign": motor_sign}],
             on_exit=Shutdown()),

        Node(package="me130_pendulum", executable="motor_characterization_node",
             name="motor_characterization_node", output="screen",
             parameters=[{"runs_per_direction": runs, "output_csv": output_csv}],
             on_exit=Shutdown()),
    ])
