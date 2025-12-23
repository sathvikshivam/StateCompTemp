#include "robot-config.h"
#include "drive.h"
#include "odometry.h"
#include <cmath>

// ==================================================
// Utility helpers
// ==================================================

double clamp(double v, double minV, double maxV) {
  return fmax(minV, fmin(maxV, v));
}

// Keep angle between -pi and +pi
double angleWrap(double a) {
  while (a > M_PI)  a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}

// Minimum turn voltage so robot actually rotates
double applyMinTurn(double turn, double err) {
  if (fabs(err) > 0.05 && fabs(turn) < 3.0) {
    return (turn > 0 ? 3.0 : -3.0);
  }
  return turn;
}

// Minimum forward voltage so robot actually moves
double applyMinForward(double fwd) {
  if (fabs(fwd) > 0.05 && fabs(fwd) < 5.0) {
    return (fwd > 0 ? 5.0 : -5.0);
  }
  return fwd;
}

// ==================================================
// Low-level drive (voltage control)
// ==================================================

void driveVolts(double left, double right) {
  LeftFront.spin(fwd, left, volt);
  LeftBack.spin(fwd, left, volt);
  RightFront.spin(fwd, right, volt);
  RightBack.spin(fwd, right, volt);
}

// ==================================================
// Turn to heading (radians)
// ==================================================

void turnTo(double targetTheta) {
  const double kP = 7.0;     // stronger turn
  const double kD = 0.6;

  double prevErr = 0;

  while (true) {
    double err   = angleWrap(targetTheta - theta);
    double deriv = err - prevErr;
    prevErr = err;

    double out = clamp(kP * err + kD * deriv, -12, 12);
    out = applyMinTurn(out, err);

    driveVolts(out, -out);

    if (fabs(err) < 0.02) break;
    wait(10, msec);
  }

  driveVolts(0, 0);
}

// ==================================================
// Drive to field-relative point (inches)
// ==================================================

void driveToPoint(double tx, double ty) {
  const double kP_dist = 1.3;
  const double kP_turn = 3.5;

  while (true) {
    double dx = tx - x;
    double dy = ty - y;

    double dist = sqrt(dx * dx + dy * dy);
    if (dist < 0.5) break;

    double targetAngle = atan2(dy, dx);
    double headingErr  = angleWrap(targetAngle - theta);

    double forward = clamp(kP_dist * dist, -12, 12);
    double turn    = clamp(kP_turn * headingErr, -10, 10);

    forward = applyMinForward(forward);
    turn    = applyMinTurn(turn, headingErr);

    // -----------------------------
    // Heading-first control
    // -----------------------------
    if (fabs(headingErr) > 0.35) {
      // Big misalignment → TURN ONLY
      driveVolts(turn, -turn);
    } else {
      // Small misalignment → DRIVE + STEER
      driveVolts(forward + turn, forward - turn);
    }

    wait(15, msec);
  }

  driveVolts(0, 0);
}
