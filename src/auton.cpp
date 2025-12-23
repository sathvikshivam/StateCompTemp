#include "robot-config.h"   // REQUIRED
#include "auton.h"
#include "drive.h"
#include "odometry.h"



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


void autonomousRoutine() {

  // Reset odometry so we start at (0,0) facing forward
  resetOdometry();
  startOdometry();

  // --------------------------------------------------
  // 1) Move forward to point (0, 5)
  //    This means: same X, 5 inches to the left
  // --------------------------------------------------
  //driveToPoint(0, 5);

  // --------------------------------------------------
  // 2) Turn 90 degrees left
  //    90 degrees = π / 2 radians
  // --------------------------------------------------
  //turnTo(M_PI / 2);

  // --------------------------------------------------
  // 3) Move to point (5, 5)
  // --------------------------------------------------
  //driveToPoint(5, 5);

  // --------------------------------------------------
  // 4) Move to point (10, 10)
  // --------------------------------------------------
  driveToPoint(10, 10);
}


