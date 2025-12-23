#include "robot-config.h"   // REQUIRED
#include "usercontrol.h"

// ================================================================
//                    DRIVER TUNING VARIABLES
// ================================================================

// Driving sensitivity (expo curve)
// 1.0 = linear, 1.5–2.0 = smoother low speed
constexpr double kDriveSensitivity = 3;

// Turn assist
// 0.0 = OFF (pure tank)
// 0.15–0.3 = recommended
constexpr double kTurnAssist = 0.3;

// Braking sensitivity (deceleration rate)
// LOWER = harder braking
// HIGHER = smoother braking
constexpr double kBrakeSensitivity = 200.0;

// Acceleration rate (leave higher than brake)
constexpr double kAccelRate = 400.0;

// Deadband
constexpr double kDeadbandPct = 5.0;


// ================================================================
//                         HELPERS
// ================================================================
double clamp100(double v) {
  if (v > 100) return 100;
  if (v < -100) return -100;
  return v;
}

double applyDeadband(double v, double db) {
  if (fabs(v) < db) return 0;
  return v;
}

double expoPercent(double v, double exp) {
  double sign = (v >= 0) ? 1.0 : -1.0;
  double norm = fabs(v) / 100.0;
  return sign * pow(norm, exp) * 100.0;
}

// Slew-rate limiter (controls accel & braking)
double ramp(double target, double current, double step) {
  if (target > current + step) return current + step;
  if (target < current - step) return current - step;
  return target;
}


// ================================================================
//                     USER CONTROL LOOP
// ================================================================
void userControlRoutine() {

  // Stored motor commands for braking control
  double leftCmd  = 0;
  double rightCmd = 0;

  while (true) {

    // ---------------- Tank Drive Input ----------------
    double leftRaw  = Controller1.Axis3.position(percent); // Left stick
    double rightRaw = Controller1.Axis2.position(percent); // Right stick

    // Sensitivity (expo)
    leftRaw  = expoPercent(leftRaw,  kDriveSensitivity);
    rightRaw = expoPercent(rightRaw, kDriveSensitivity);

    // Deadband
    leftRaw  = applyDeadband(leftRaw,  kDeadbandPct);
    rightRaw = applyDeadband(rightRaw, kDeadbandPct);

    // ---------------- Turn Assist ----------------
    double turn   = leftRaw - rightRaw;
    double assist = turn * kTurnAssist;

    double leftTarget  = clamp100(leftRaw  + assist);
    double rightTarget = clamp100(rightRaw - assist);

    // ---------------- Braking Sensitivity ----------------
    double leftStep  = (fabs(leftTarget)  < fabs(leftCmd))  ? kBrakeSensitivity : kAccelRate;
    double rightStep = (fabs(rightTarget) < fabs(rightCmd)) ? kBrakeSensitivity : kAccelRate;

    leftCmd  = ramp(leftTarget,  leftCmd,  leftStep);
    rightCmd = ramp(rightTarget, rightCmd, rightStep);

    // Drive motors
    LeftFront.spin(forward, leftCmd, percent);
    LeftBack.spin(forward, leftCmd, percent);
    RightFront.spin(forward, rightCmd, percent);
    RightBack.spin(forward, rightCmd, percent);

    // ---------------- Game Piece Control ----------------
    if (Controller1.ButtonL2.pressing()) {
      conveyorMotor.stop(hold);
      hoodMotor.stop(hold);
      intakeMotor.spin(forward, 100, percent);
      scoreMotor.stop(hold);

    } 
    else if (Controller1.ButtonL1.pressing()) {
      conveyorMotor.spin(reverse, 100, percent);
      hoodMotor.spin(reverse, 100, percent);
      intakeMotor.spin(reverse, 100, percent);
      scoreMotor.spin(reverse, 100, percent);

    } 
    else if (Controller1.ButtonR1.pressing()) {
      conveyorMotor.spin(forward, 100, percent);
      hoodMotor.stop(hold);
      intakeMotor.spin(forward, 100, percent);
      scoreMotor.stop(hold);

    } 
    else if (Controller1.ButtonR2.pressing()) {
      conveyorMotor.spin(forward, 100, percent);
      hoodMotor.spin(forward, 100, percent);
      intakeMotor.spin(forward, 100, percent);
      scoreMotor.spin(forward, 100, percent);

    }
    else if (Controller1.ButtonLeft.pressing()) {
      scraper.set(true);
      aligner.set(false);
    }
    else if (Controller1.ButtonRight.pressing()) {
      scraper.set(false);
      aligner.set(true);

    }
    else if (Controller1.ButtonUp.pressing()) {
      aligner.set(false);

    }
    else {
      conveyorMotor.stop(coast);
      intakeMotor.stop(coast);
      hoodMotor.stop(coast);
      scoreMotor.stop(coast);
    }

    wait(10, msec);
  }
}
