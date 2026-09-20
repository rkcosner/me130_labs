# Command into the motor node.
#
# u is the PRE-deadband signed duty in [-1, 1]: sign is direction, magnitude is
# effort. motor_node applies the deadband feed-forward and the motor sign, so
# every producer speaks the same units and only one node knows the hardware.
#
# The remaining fields are test context, carried here so logger_node can write
# a self-describing CSV without subscribing to each test node individually.
std_msgs/Header header
float64 u

# coast | pd | step | sine
string mode

# sine drive only, otherwise 0
float64 freq_hz
float64 amplitude

# constant-step drive only, otherwise 0
float64 step_duty

# increments once per test segment (one step, or one frequency)
int32 segment
