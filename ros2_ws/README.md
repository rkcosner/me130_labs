# ME130 pendulum — ROS 2 workspace

ROS 2 Jazzy. The control code is C++; the analysis is Python.

## Block diagram

```
  imu_node ───/pendulum/angle──┬──> controller_node ──┐
  (MPU6050)                    │   (student gains)    │
                               │                      ├──/motor/command──> motor_node ──> BD65496MUV
  encoder_node ──/encoder/state┤   step_sequence_node ─┤                   (deadband,      + motor
  (GPIO17/27)                  │   sine_sweep_node ────┤                    motor_sign)
                               │   motor_characterization_node             (PWM18/DIR25/PS24)
                               └──> logger_node ──> <test>_<timestamp>.csv
```

Each node owns a disjoint slice of the hardware, so they never contend:

| node | hardware |
|---|---|
| `motor_node` | PWM ch0 (GPIO18), DIR GPIO25, PS GPIO24 |
| `encoder_node` | GPIO17, GPIO27 |
| `imu_node` | `/dev/i2c-1` |

`motor_node` is the only node that moves the motor. Everyone else publishes a
**pre-deadband** signed duty on `/motor/command`; `motor_node` applies the
deadband feed-forward and the motor sign. Change the rig and only that node
changes.

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ~/me130_labs/ros2_ws
colcon build
source install/setup.bash
```

## The four labs

### 1. Motor characterization — find the deadband

```bash
ros2 launch me130_pendulum motor_characterization.launch.py
ros2 run me130_pendulum plot_characterization.py
```

**Detach the pendulum first.** The shaft sweeps to full duty both ways. Prints
the mean breakaway, stop and deadband, and writes
`motor_characterization.csv`. `deadband` is 0.0 here on purpose — this lab
*measures* the deadband, so the motor must be driven raw.

The number it prints is the `deadband:=` argument for labs 2, 3 and 4.

### 2. Step response — free and forced

```bash
ros2 launch me130_pendulum step_response.launch.py deadband:=0.065
ros2 run me130_pendulum step_response.py
```

Rod **hanging down**. Each duty is held, then released so the rod rings down:
every segment gives a forced response followed by a free response. Writes
`steps_<timestamp>.csv`. The script plots both and identifies

    theta'' + c*theta' + a*theta = b*u

Raise `rest_s:=5.0` if the rod is still moving when the next step begins.

### 3. Frequency response

```bash
ros2 launch me130_pendulum frequency_response.launch.py deadband:=0.065
ros2 run me130_pendulum freq_response.py
```

Rod **hanging down**. Writes `sweep_<timestamp>.csv`. The Bode plot overlays
the model identified from the newest `steps_*.csv` — so the steps predict the
curve and the sweep measures it, independently.

Hold time per frequency is `skip_s + cycles/f`, so low frequencies are driven
longer. Keep `skip_s` equal to `SKIP_S` in `freq_response.py`.

### 4. Balance upright

```bash
ros2 launch me130_pendulum balance.launch.py deadband:=0.065
```

**The gains default to zero, so this does nothing until you set them.** That
is the lab.

Open a SECOND terminal and run the console:

```bash
ros2 run me130_pendulum keyboard_node
```

It needs its own terminal because `ros2 launch` does not give child processes
a usable stdin. Single keystrokes, as in the pre-ROS firmware:

```
  z  zero theta here (hold the rod upright first)
  e  arm            x  disarm            q  quit
  s  flip motor sign                     t  telemetry on/off
  p<Kp>  d<Kd>  i<Ki>  w<integral limit>      e.g.  p10.0<Enter>
  f<deadband>
```

Gains can be changed while armed; the controller re-reads them every tick.

Everything it does is a plain service call or parameter set, so the long form
still works if you prefer it:

```bash
ros2 service call /imu_node/zero std_srvs/srv/Trigger
ros2 param set /controller_node kp 10.0
ros2 service call /controller_node/arm std_srvs/srv/Trigger
```

Add `log:=true` to record a `balance_*.csv`.

## Model conventions

Standard quadratic ordering throughout, so the letters match `a*s^2 + b*s + c`:

```
    P(s) = K / (s^2 + b*s + c)        theta'' + b*theta' + c*theta = K*u
      K  input gain        b  damping (friction, back-EMF)
      c  stiffness = wn^2  ->  wn = sqrt(c),  zeta = b/(2*sqrt(c))
```

Hanging, gravity restores and `c > 0`. **Inverted, gravity destabilises and the
STIFFNESS flips sign**: `s^2 + b*s - c`. Damping does not flip -- friction does
not care which way up the rod is.

With `u = -(Kp*theta + Kd*theta_dot)` the closed loop is

```
    s^2 + (b + K*Kd) s + (K*Kp - c) = 0
```

so stability needs `Kp > c/K` and `Kd > -b/K`.

## Safety

- `motor_node` coasts if no command arrives for `command_timeout_s` (0.5 s), so
  killing a test node stops the motor rather than leaving it driving.
- `controller_node` disarms itself past `theta_max_deg` (90°).
- Arming is refused outside `arm_window_deg` (5°) of the zero.
- Gains start at zero so a fresh checkout cannot lunge.

## Parameters worth knowing

| parameter | node | default | meaning |
|---|---|---|---|
| `deadband` | `motor_node` | 0.0 | from lab 1; 0.0 disables compensation |
| `motor_sign` | `motor_node` | 1.0 | flip to -1.0 if +duty spins the wrong way |
| `kp`, `ki`, `kd` | `controller_node` | 0.0 | students set these |
| `step_duties` | `step_sequence_node` | ±0.10…±0.25 | raise if the small steps stall |
| `frequencies_hz` | `sine_sweep_node` | 0.3…3.5 | centre on your resonance |
