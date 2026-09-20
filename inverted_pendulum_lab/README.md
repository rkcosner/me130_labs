# IMU Pendulum Balance — PID Lab

## Files

| File | Who touches it | What's in it |
|---|---|---|
| `pendulum.hpp` | Nobody | Hardware layer (MPU6050 I2C driver, hardware PWM, GPIO/encoder, motor mapping, raw terminal input) plus the `extern`-declared interface between the harness and your controller. |
| `pendulum_runner.cpp` | Nobody | Owns `main()` and the full loop: IMU sensing + sensor fusion, motor driving, encoder, keyboard interface, arm/disarm safety, telemetry. Calls `computePID()`/`applyDeadband()` every tick. |
| `controller.cpp` | **Students** | The handout. Only the gains and two `TODO` functions — `computePID()` and `applyDeadband()`. |
| `controller_solution.cpp` | Instructor | Same shape as `controller.cpp` with both functions filled in exactly as the original program computed them (`Kp=6.0, Kd=0.6, Ki=1.0`, deadband formula, antiwindup). |
| `CMakeLists.txt` | — | Builds `pendulum` (from `controller.cpp` + `pendulum_runner.cpp`) and `pendulum_solution` (from `controller_solution.cpp` + `pendulum_runner.cpp`). |

## What students implement

Everything is in `controller.cpp`:

1. **Gains** — `Kp`, `Kd`, `Ki` (Ki is optional; leave the integral off with the `t` command if unused), `DEADBAND`, `INTEGRAL_TERM_MAX`.
2. **`applyDeadband(u)`** — deadband compensation so small commands still overcome motor static friction.
3. **`computePID(...)`** — the PID law itself: `u = -(Kp*theta + Ki*integral(theta) + Kd*theta_dot)`, with antiwindup clamping the integral *term* to `±INTEGRAL_TERM_MAX`.

All gains are also tunable live at runtime without recompiling, via the keyboard commands documented at the top of the file (`p<Kp>`, `d<Kd>`, `i<Ki>`, `w<integral_limit>`, `f<deadband>`, `c<complementary_filter>`, plus `z`/`e`/`x`/`s`/`r`/`t`/`q`).

## Building

On the Raspberry Pi (needs `libgpiod-dev` and `dtoverlay=pwm,pin=18,func=2` in `/boot/firmware/config.txt`, as noted in `pendulum_runner.cpp`):

```bash
mkdir build && cd build
cmake ..
make
sudo ./pendulum            # student build
sudo ./pendulum_solution   # reference build
```

## Safety reminder (unchanged from the original)

Boots disarmed. `e` only arms within 5° of the `z`-zeroed upright position. Auto-coasts past 90° tilt or if the IMU drops out. **First run: support the rod by hand and verify `theta` sign (via telemetry) and motor sign (`s`) before arming.**