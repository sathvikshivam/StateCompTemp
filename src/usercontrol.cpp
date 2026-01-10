#include "robot-config.h"   // REQUIRED
#include "usercontrol.h"
#include "main.h"
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
constexpr double kBrakeSensitivity = 550.0;

// Acceleration rate (leave higher than brake)
constexpr double kAccelRate = 400.0;

// Deadband
constexpr double kDeadbandPct = 5.0;

constexpr double kCurveExp    = 5.0;  // 2.0 = mild expo, higher = softer near center

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
vex::color getAllianceColor() {
  return (alliance == "red") ? vex::red : vex::blue;
}
bool isAllianceColor() {
  if (!Optical.installed()) return false;

  vex::color c = Optical.color();

  if (alliance == "red") {
    return c == vex::red;
  }
  else if (alliance == "blue") {
    return c == vex::blue;
  }

  return false;
}


// ================================================================
//                     USER CONTROL LOOP
// ================================================================
void userControlRoutine() {
  Optical.setLightPower(100, percent);
  Optical.setLight(ledState::on);
  // Stored motor commands for braking control
  double leftCmd  = 0;
  double rightCmd = 0;

  while (true) {
    // ---------------- Tank Drive Input ----------------
    double forwardRaw = Controller1.Axis3.position(percent);
    double turnRaw    = Controller1.Axis1.position(percent);

    forwardRaw = expoPercent(forwardRaw, kCurveExp);
    turnRaw    = expoPercent(turnRaw,    kCurveExp);

    forwardRaw = applyDeadband(forwardRaw, kDeadbandPct);
    turnRaw    = applyDeadband(turnRaw,    kDeadbandPct);

    double leftVal  = clamp100(forwardRaw + turnRaw);
    double rightVal = clamp100(forwardRaw - turnRaw);

    leftDrive.spin(forward, leftVal, percent);
    rightDrive.spin(forward, rightVal, percent);

    // ---------------- Game Piece Control ----------------

 if (Controller1.ButtonR1.pressing()) {

  // ---------- DEBUG ----------
printf("installed=%d hue=%d\n",
       Optical.installed(),
       Optical.hue());


  // ---------- ALWAYS RUN THESE ----------
  conveyorMotor.spin(forward, 100, percent);
  hoodMotor.stop(hold);
  intakeMotor.spin(forward, 100, percent);

  // ---------- SENSOR MISSING ----------
 if (!Optical.installed()) {
  scoreMotor.stop(hold);
}
else {

  bool allianceBall = false;
  int hue = Optical.hue();

  if (hue >= 0 && hue < 360) {
    if (alliance == "red") {
      allianceBall = (hue <= 30 || hue >= 330);
    }
    else if (alliance == "blue") {
      allianceBall = (hue >= 200 && hue <= 260);
    }
  }

  if (allianceBall) {
    scoreMotor.stop(hold);
  }
  else {
    printf(">>> COMMANDING SCORE MOTOR 30%% <<<\n");

    scoreMotor.spin(forward, 30, percent);
  }
}
}



    else if (Controller1.ButtonR2.pressing()) {
      conveyorMotor.spin(forward, 100, percent);
      hoodMotor.spin(forward, 100, percent);
      intakeMotor.spin(forward, 100, percent);
      scoreMotor.spin(forward, 100, percent);

    }
    else if (Controller1.ButtonX.pressing()) {
      descore.set(true);
    }
    else if (Controller1.ButtonA.pressing()) {
      descore.set(false);

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
        else if (Controller1.ButtonL2.pressing()) {
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
    else {
      conveyorMotor.stop(coast);
      intakeMotor.stop(coast);
      hoodMotor.stop(coast);
      scoreMotor.stop(coast);
    }

    wait(10, msec);
  }
}
