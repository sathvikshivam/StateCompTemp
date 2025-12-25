#include "robot-config.h"
#include "drive.h"
#include "odometry.h"
#include <cmath>

using namespace vex;

// --------------------------------------------------
// Helpers
// --------------------------------------------------

static double angleWrap(double a) {
  while (a > M_PI)  a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}

static double clamp(double v, double max) {
  if (v >  max) return  max;
  if (v < -max) return -max;
  return v;
}

// --------------------------------------------------
// Low-level drive
// --------------------------------------------------

void drivePercent(double left, double right) {
  LeftFront.spin(fwd, left, percent);
  LeftBack.spin(fwd, left, percent);
  RightFront.spin(fwd, right, percent);
  RightBack.spin(fwd, right, percent);
}

void stopDrive() {
  drivePercent(0, 0);
}

// --------------------------------------------------
// Turn to heading (radians)
// --------------------------------------------------

void turnTo(double targetDeg) {
  Inertial.resetRotation();
  while (Inertial.isCalibrating()) {
    wait(5, vex::msec);
  }

  const double FAST_PWR  = 35;   // slightly lower
  const double SLOW_PWR  = 12;
  const double SLOW_ZONE = 20.0;
  const double STOP_ZONE = 1.5;
  const double LEAD      = 8.5;  // stop early to counter momentum

  bool crossedTarget = false;

  printf("\nTURN START target=%.2f\n", targetDeg);

  while (true) {
    double current = -Inertial.rotation(deg);

    // stop early to compensate for inertia
    double adjustedTarget =
      targetDeg - LEAD * (targetDeg > 0 ? 1 : -1);

    double error = adjustedTarget - current;

    if ((targetDeg > 0 && current >= adjustedTarget) ||
        (targetDeg < 0 && current <= adjustedTarget)) {
      crossedTarget = true;
    }

    printf("cur=%.2f adjTarget=%.2f err=%.2f crossed=%d\n",
           current, adjustedTarget, error, crossedTarget);

    if (fabs(error) < STOP_ZONE || crossedTarget) {
      printf("STOP LOGIC cur=%.2f\n\n", current);
      break;
    }

    double power = (fabs(error) > SLOW_ZONE) ? FAST_PWR : SLOW_PWR;
    if (error < 0) power = -power;

    leftDrive.spin(vex::fwd, -power, vex::percent);
    rightDrive.spin(vex::fwd,  power, vex::percent);

    wait(20, vex::msec);
  }

  // HARD BRAKE to kill momentum
  leftDrive.spin(vex::fwd,  20, vex::percent);
  rightDrive.spin(vex::fwd, -20, vex::percent);
  wait(70, vex::msec);

  leftDrive.stop(vex::hold);
  rightDrive.stop(vex::hold);
}











// --------------------------------------------------
// Drive straight (encoder based, smooth)
// --------------------------------------------------

void driveStraight(double inches) {
  // ---------- Constants ----------
  const double WHEEL_DIAMETER = 2.75;     // inches (measure yours!)
  const double WHEEL_CIRC     = M_PI * WHEEL_DIAMETER;

  const double FAST_PWR  = 50;
  const double SLOW_PWR  = 20;
  const double SLOW_ZONE = 6.0;    // inches
  const double STOP_ZONE = 0.5;    // inches

  const double LEAD_IN   = 1.0;    // stop early to counter coast
  const double kH        = 1.2;    // heading correction gain

  // ---------- Reset encoders ----------
  LeftFront.resetPosition();
  LeftBack.resetPosition();
  RightFront.resetPosition();
  RightBack.resetPosition();

  // ---------- Reset & lock heading ----------
  Inertial.resetRotation();
  while (Inertial.isCalibrating()) {
    wait(5, vex::msec);
  }
  double desiredHeading = -Inertial.rotation(deg);

  // ---------- Targets ----------
  double adjustedTargetIn = inches - (inches > 0 ? LEAD_IN : -LEAD_IN);
  double targetDeg = (adjustedTargetIn / WHEEL_CIRC) * 360.0;

  // ---------- Timing start ----------
  double startMs = Brain.timer(vex::msec);
  double logicStopMs = -1;

  printf("\nDRIVE START cmd=%.2f in (adj=%.2f in)\n", inches, adjustedTargetIn);
  printf("Desired heading = %.2f deg\n", desiredHeading);

  while (true) {
    // ----- Encoder distance -----
    double leftDeg  = (LeftFront.position(deg) + LeftBack.position(deg)) / 2.0;
    double rightDeg = (RightFront.position(deg) + RightBack.position(deg)) / 2.0;
    double avgDeg   = (leftDeg + rightDeg) / 2.0;

    double errorDeg = targetDeg - avgDeg;
    double errorIn  = (errorDeg / 360.0) * WHEEL_CIRC;

    // ----- Stop condition (LOGIC STOP) -----
    if (fabs(errorIn) < STOP_ZONE) {
      logicStopMs = Brain.timer(vex::msec);
      printf("LOGIC STOP t=%.0f ms | distErrToAdj=%.2f in\n",
             logicStopMs - startMs, errorIn);
      break;
    }

    // ----- Base power -----
    double basePower = (fabs(errorIn) > SLOW_ZONE) ? FAST_PWR : SLOW_PWR;
    if (errorIn < 0) basePower = -basePower;

    // ----- Heading correction -----
    double currentHeading = -Inertial.rotation(deg);
    double headingError   = desiredHeading - currentHeading;
    double turnCorrection = headingError * kH;

    // ----- Motor outputs -----
    double leftPower  = basePower - turnCorrection;
    double rightPower = basePower + turnCorrection;

    // Clamp
    if (leftPower  >  100) leftPower  =  100;
    if (leftPower  < -100) leftPower  = -100;
    if (rightPower >  100) rightPower =  100;
    if (rightPower < -100) rightPower = -100;

    // ----- Debug -----
    printf("errAdj=%.2f in  headErr=%.2f deg  L=%.1f  R=%.1f\n",
           errorIn, headingError, leftPower, rightPower);

    // ----- Drive -----
    leftDrive.spin(vex::fwd, leftPower,  vex::percent);
    rightDrive.spin(vex::fwd, rightPower, vex::percent);

    wait(20, vex::msec);
  }

  // ---------- Brake pulse ----------
  leftDrive.spin(vex::fwd, -15, vex::percent);
  rightDrive.spin(vex::fwd, -15, vex::percent);
  wait(60, vex::msec);

  leftDrive.stop(vex::hold);
  rightDrive.stop(vex::hold);

  // ---------- Timing end ----------
  double endMs = Brain.timer(vex::msec);

  // ---------- Final readings ----------
  double fLeftDeg  = (LeftFront.position(deg) + LeftBack.position(deg)) / 2.0;
  double fRightDeg = (RightFront.position(deg) + RightBack.position(deg)) / 2.0;
  double fAvgDeg   = (fLeftDeg + fRightDeg) / 2.0;

  double encoderDistIn = (fAvgDeg / 360.0) * WHEEL_CIRC;

  double distErrVsCmd = inches - encoderDistIn;
  double distErrVsAdj = adjustedTargetIn - encoderDistIn;

  double finalHeading    = -Inertial.rotation(deg);
  double finalHeadingErr = desiredHeading - finalHeading;

  // ---------- Final report ----------
  printf("=== DRIVE RESULT ===\n");
  printf("Commanded distance : %.2f in\n", inches);
  printf("Adjusted target    : %.2f in\n", adjustedTargetIn);
  printf("Encoder distance   : %.2f in\n", encoderDistIn);
  printf("Err vs commanded   : %.2f in\n", distErrVsCmd);
  printf("Err vs adjusted    : %.2f in\n", distErrVsAdj);
  printf("Final heading err  : %.2f deg\n", finalHeadingErr);
  printf("Logic-stop time    : %.0f ms\n", (logicStopMs < 0 ? -1 : (logicStopMs - startMs)));
  printf("Total function time: %.0f ms\n\n", endMs - startMs);
}






// --------------------------------------------------
// Go to point (uses odometry globals x, y, theta)
// --------------------------------------------------

void goToPoint(double targetX, double targetY) {
  double dx = targetX - x;
  double dy = targetY - y;

  double targetAngle = atan2(dy, dx);
  double distance    = sqrt(dx * dx + dy * dy);

  turnTo(targetAngle);
  driveStraight(distance);
}
