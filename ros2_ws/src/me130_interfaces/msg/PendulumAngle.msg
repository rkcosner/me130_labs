# Pendulum angle from the IMU node.
# theta is measured from the zero set by the imu_node "zero" service:
# hanging straight down for the characterization tests, upright for balancing.
std_msgs/Header header
float64 theta_rad
float64 theta_dot_rad_s
