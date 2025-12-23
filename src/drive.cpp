#include "robot-config.h"
#include "drive.h"
#include "odometry.h"
#include <cmath>

// --------------------------------------------------
// Utility helpers
// --------------------------------------------------

double clamp(double v, double minV, double maxV) {
  return fmax(minV, fmin(maxV, v));
}

double angleWrap(double a) {
  while (a > M_PI)  a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}

// Enforce minimum turning power so one side never stalls
double minTurn(double turn, double err) {
  if (fabs(err) > 0.05 && fabs(turn) < 2.0) {
    return (turn > 0 ? 2.0 : -2.0);
  }
  return turn;
}

// --------------------------------------------------
// Low-level drive
// --------------------------------------------------

void driveVolts(double left, double right) {
  LeftFront.spin(fwd, left, volt);
  LeftBack.spin(fwd, left, volt);
  RightFront.spin(fwd, right, volt);
  RightBack.spin(fwd, right, volt);
}

// --------------------------------------------------
// Turn to heading (radians)
// --------------------------------------------------

void turnTo(double targetTheta) {
  double kP = 6.0;
  double kD = 0.8;

  double prevErr = 0;

  while (true) {
    double err   = angleWrap(targetTheta - theta);
    double deriv = err - prevErr;
    prevErr = err;

    double out = clamp(kP * err + kD * deriv, -10, 10);
    out = minTurn(out, err);

    driveVolts(out, -out);

    if (fabs(err) < 0.02) break;
    wait(10, msec);
  }

  driveVolts(0, 0);
}

// --------------------------------------------------
// Drive to field-relative point (inches)
// --------------------------------------------------

void driveToPoint(double tx, double ty) {
  double kP_dist = 1.0;   // forward gain
  double kP_turn = 2.5;   // turn gain

  while (true) {
    double dx = tx - x;
    double dy = ty - y;

    double dist = sqrt(dx * dx + dy * dy);
    if (dist < 0.5) break;

    double targetAngle = atan2(dy, dx);
    double headingErr  = angleWrap(targetAngle - theta);

    double forward = clamp(kP_dist * dist, -7, 7);
    double turn    = clamp(kP_turn * headingErr, -5, 5);
    turn = minTurn(turn, headingErr);

    // 🔑 Heading-first control (THIS is what fixes jerking)
    if (fabs(headingErr) > 0.25) {
      // Too misaligned → turn in place
      driveVolts(turn, -turn);
    } else {
      // Aligned → drive with steering
      driveVolts(forward + turn, forward - turn);
    }

    wait(15, msec);
  }

  driveVolts(0, 0);
}
