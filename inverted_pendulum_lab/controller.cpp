/* ============================================================================
 * controller.cpp -- IMU Pendulum Balance -- PID CONTROLLER ASSIGNMENT
 * ============================================================================
 * This is the only file you need to edit. Everything else -- reading the
 * IMU, fusing it into an angle, driving the motor, the keyboard interface,
 * safety limits -- lives in pendulum_runner.cpp and runs automatically; it
 * calls the two functions below every control tick.
 *
 * theta      : filtered tilt angle (rad), 0 = upright. This is your ERROR
 *              signal, since the setpoint is always 0 (upright).
 * theta_dot  : angular rate (rad/s), already bias-corrected.
 *
 * SAFETY (handled for you, described here so you know what to expect):
 *   Boots disarmed. 'e' arms only within 5 deg of the 'z'-zeroed upright
 *   position. Auto-coasts if |theta| > 90 deg or the IMU drops out.
 *   'x' = kill. 's' = flip motor sign.
 *   >>> First run: support the rod by hand and verify theta/sign before arming. <<<
 *
 * Commands available while it's running (type letter, value if needed, Enter):
 *   z=set-upright  e=arm  x=disarm  s=flip-motor-sign
 *   r=flip-rate-sign  t=toggle-integral  q=quit
 *   p<Kp>  d<Kd>  i<Ki>  w<integral_term_limit>  f<deadband>
 *   c<complementary_filter 0..1>
 * ==========================================================================*/
#include <cmath>
#include <algorithm>
#include "pendulum.hpp"

// =============================================================================
// Gains -- these are live-tunable at runtime with the p / d / i / w / f
// keyboard commands, so you can leave the numbers below as a starting point
// and tune from the keyboard while it's running -- you don't have to
// recompile every time.
// =============================================================================
double Kp = 0.0;                 // TODO: proportional gain
double Kd = 0.0;                 // TODO: derivative gain
double Ki = 0.0;                 // TODO: integral gain -- OPTIONAL. Leave at 0
                                  //       and keep the integral OFF ('t' command)
                                  //       if you don't want to use it.
double DEADBAND = 0.0;           // TODO: deadband compensation, 0..1 (see applyDeadband below)
double INTEGRAL_TERM_MAX = 0.5;  // Antiwindup clamp on the INTEGRAL TERM
                                  // (Ki * integral), not on the raw integral.
                                  // Only matters if you enable the I-term.

bool integralEnabled = false;    // toggled live with the 't' command

/*
 * applyDeadband(u)
 * ----------------
 * A real motor won't turn at all below some minimum duty (static friction /
 * stiction in the gearbox). If your PID output u is small, the commanded PWM
 * duty can be too small to ever move the motor.
 *
 * Deadband compensation fixes this by adding a "jump" so that ANY nonzero
 * command gets at least DEADBAND worth of duty cycle.
 *
 * TODO: implement so that:
 *   - if u == 0                -> return 0
 *   - if u != 0                -> return sign(u) * DEADBAND + (1 - DEADBAND) * u
 *
 * (Use std::copysign(DEADBAND, u) to get a value with magnitude DEADBAND and
 * the same sign as u.)
 *
 * u is the raw controller output in [-1, 1]; the return value is what
 * actually gets sent to the motor as a PWM duty command (also in [-1, 1]).
 */
double applyDeadband(double u)
{
    // TODO: implement deadband compensation.
    return u;
}

/*
 * computePID(...)
 * ----------------
 * Implements the control law:
 *
 *   u = -( Kp * theta  +  Ki * integral(theta dt)  +  Kd * theta_dot )
 *
 * Arguments:
 *   theta           : current tilt angle (rad). This is your error signal
 *                      directly, since setpoint = 0 (upright) always.
 *   thetaDot         : current angular rate (rad/s)
 *   dt               : time since the last control update (s)
 *   thetaIntegral    : running integral accumulator -- persists between
 *                      calls (pass by reference, update it in place)
 *   integralEnabled  : whether the operator has turned the I-term on
 *                      (the 't' command toggles this)
 *   integralTermOut  : write the actual integral term you used
 *                      (Ki * thetaIntegral, post-antiwindup) here so it can
 *                      be printed in telemetry. Write 0.0 if I-term is off.
 *
 * TODO:
 *   1. If integralEnabled, accumulate: thetaIntegral += theta * dt
 *   2. Antiwindup: clamp the INTEGRAL TERM (Ki * thetaIntegral) to
 *      +/- INTEGRAL_TERM_MAX. The cleanest way is to clamp thetaIntegral
 *      itself to +/- (INTEGRAL_TERM_MAX / Ki) before multiplying by Ki
 *      (guard against Ki == 0).
 *   3. Compute integralTermOut = Ki * thetaIntegral (0.0 if I-term is off).
 *   4. Compute and return u = -(Kp*theta + integralTermOut + Kd*thetaDot).
 *
 * Return the raw control command u, in roughly [-1, 1] (it gets clamped
 * downstream anyway). Do NOT apply the deadband here -- that happens
 * separately in applyDeadband() after MOTOR_SIGN is applied.
 */
double computePID(double theta, double thetaDot, double dt,
                   double& thetaIntegral, bool integralEnabled_,
                   double& integralTermOut)
{
    integralTermOut = 0.0;
    double u = 0.0;

    // TODO: implement the PID law described above.

    return u;
}
