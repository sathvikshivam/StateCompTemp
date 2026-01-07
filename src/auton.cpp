#include "vex.h"
#include <cmath>
#include <algorithm>
#include "robot-config.h"
#include "main.h"

// --- Tunables ---
constexpr int    kDeadbandPct = 5;
constexpr double kCurveExp    = 5.0;  // 2.0 = mild expo, higher = softer near center
// --- Helper functions ---


int getSideSign() {
  if (autonSide == "left") return -1;
  if (autonSide == "right") return 1;
  // Fallback to right-side behavior if an unexpected value is provided
  return 1;
}
/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*---------------------------------------------------------------------------*/
const double WHEEL_DIAMETER = 3.25;     // inches
const double TRACK_WIDTH = 12.0;        // center-to-center wheel distance (MEASURE THIS)
const double GEAR_RATIO = 1.0;          // 1.0 if direct drive, change if geared
void driveInches(double inches, int speed = 30) {

  double circumference = M_PI * WHEEL_DIAMETER;
  double rotations = (inches / circumference) * GEAR_RATIO;

  leftDrive.setVelocity(speed, percent);
  rightDrive.setVelocity(speed, percent);

  leftDrive.spinFor(forward, rotations, rev, false);
  rightDrive.spinFor(forward, rotations, rev, true);

  leftDrive.stop(coast);
  rightDrive.stop(coast);
}
void turnDegrees(double degrees, int speed = 25) {
  degrees=degrees*getSideSign();
  leftDrive.stop(coast);
  rightDrive.stop(coast);

  double turnCircumference = M_PI * TRACK_WIDTH;
  double distance = (degrees / 360.0) * turnCircumference;
  double rotations = distance / (M_PI * WHEEL_DIAMETER);

  leftDrive.setVelocity(speed, percent);
  rightDrive.setVelocity(speed, percent);

  // ✅ SAME direction, config handles reversal
  leftDrive.spinFor(fwd,  rotations, rev, false);
  rightDrive.spinFor(reverse, rotations, rev, true);

  leftDrive.stop(hold);
  rightDrive.stop(hold);
}


void intakeOn(int speed, directionType dir) {
  intakeMotor.setVelocity(speed, percent);
  intakeMotor.spin(dir);
}

void intakeOff() {
  intakeMotor.stop(brake);
}
void conveyorOn(int speed, directionType dir) {
  conveyorMotor.setVelocity(speed, percent);
  conveyorMotor.spin(dir);
}

void conveyorOff() {
  conveyorMotor.stop(brake);
}
void scoreOn(int speed, directionType dir) {
  scoreMotor.setVelocity(speed, percent);
  scoreMotor.spin(dir);
  hoodMotor.setVelocity(speed, percent);
  hoodMotor.spin(dir);
  conveyorOn(speed, dir);
  intakeOn(speed, dir);

}



void scoreOff() {
  scoreMotor.stop(brake);
  hoodMotor.stop(brake);
}
void driveInchesAsync(double inches, int speed = 30) {

  double circumference = M_PI * WHEEL_DIAMETER;
  double rotations = (inches / circumference) * GEAR_RATIO;

  leftDrive.setVelocity(speed, percent);
  rightDrive.setVelocity(speed, percent);

  leftDrive.spinFor(forward, rotations, rev, false); // doesn't wait
  rightDrive.spinFor(forward, rotations, rev, false);
}
void scoreOnAutoUnjam(int speed, vex::directionType dir) {

  // Set speeds
  scoreMotor.setVelocity(speed, percent);
  hoodMotor.setVelocity(speed, percent);
  conveyorMotor.setVelocity(speed, percent);
  intakeMotor.setVelocity(speed, percent);

  // Start motors
  scoreMotor.spin(dir);
  hoodMotor.spin(dir);
  conveyorMotor.spin(dir);
  intakeMotor.spin(dir);

  // Let them try to move
  wait(250, msec);

  // ---- JAM CHECKS ----
  if (fabs(scoreMotor.position(degrees)) < 3) {
    scoreMotor.spinFor(reverse, 5, degrees);
    scoreMotor.spin(dir);
  }

  if (fabs(hoodMotor.position(degrees)) < 3) {
    hoodMotor.spinFor(reverse, 5, degrees);
    hoodMotor.spin(dir);
  }

  if (fabs(conveyorMotor.position(degrees)) < 3) {
    conveyorMotor.spinFor(reverse, 5, degrees);
    conveyorMotor.spin(dir);
  }

  if (fabs(intakeMotor.position(degrees)) < 3) {
    intakeMotor.spinFor(reverse, 5, degrees);
    intakeMotor.spin(dir);
  }
}

void hoarderOn(int speed, vex::directionType dir) {

  // Stop scoring system
  hoodMotor.stop(hold);
  scoreMotor.stop(hold);

  // Set speeds
  intakeMotor.setVelocity(speed, percent);
  conveyorMotor.setVelocity(speed, percent);

  // Start hoarder system
  intakeMotor.spin(dir);
  conveyorMotor.spin(dir);

  // Let motors try to move
  wait(250, msec);

  // ---- JAM CHECK: INTAKE ----
  if (fabs(intakeMotor.position(degrees)) < 3) {
    intakeMotor.spinFor(reverse, 5, degrees);
    intakeMotor.spin(dir);
  }

  // ---- JAM CHECK: CONVEYOR ----
  if (fabs(conveyorMotor.position(degrees)) < 3) {
    conveyorMotor.spinFor(reverse, 5, degrees);
    conveyorMotor.spin(dir);
  }
}




/*---------------------------------------------------------------------------*/
/*                              Autonomous Task                              */
/*---------------------------------------------------------------------------*/
void autonomousRoutine() {

    driveInches(10.0, 30);
    turnDegrees(-49.0, 30);
    driveInches(-16.6, 30);
    turnDegrees(-54.81, 20);
    scraper.set(true);
    wait(300, msec);
    intakeOn(100, forward);
    conveyorOn(100, forward);
    scoreMotor.stop(hold);
    driveInchesAsync(11.9, 30);
    wait(2400, msec);
    intakeOff();
    driveInches(-3.5, 20);
    //turnDegrees(-8.8, 20);
    aligner.set(true);
    driveInches(-9.75, 20);
    scraper.set(false);
    scoreOnAutoUnjam(100, forward);
    wait(2500, msec);
    scraper.set(true);
    aligner.set(false);
    scoreOff();
    turnDegrees(-4.2, 20);
    scoreOn(100, forward);
    driveInches(1.75, 20);
    driveInches(-3.75, 100);



  // If you need these in auton, set them her

}