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
void hoardForTime(int durationMs) {

  printf("HOARD START for %d ms\n", durationMs);

  int start = Brain.timer(vex::msec);

  while (Brain.timer(vex::msec) - start < durationMs) {

    // Intake & conveyor ON
    intakeMotor.spin(vex::forward, 100, vex::percent);
    conveyorMotor.spin(vex::forward, 100, vex::percent);

    // Score & hood held
    scoreMotor.stop(vex::hold);
    hoodMotor.stop(vex::hold);

    wait(20, vex::msec);  // control loop
  }

  // Optional: stop intake/conveyor after hoard
  intakeMotor.stop(vex::brake);
  conveyorMotor.stop(vex::brake);

  printf("HOARD END\n");
}

void driveInchesAsync(double inches, int speed = 30) {

  double circumference = M_PI * 2.75;
  double rotations = (inches / circumference) * (36/48); // gear ratio 48:36

  leftDrive.setVelocity(speed, percent);
  rightDrive.setVelocity(speed, percent);

  leftDrive.spinFor(forward, rotations, rev, false); // doesn't wait
  rightDrive.spinFor(forward, rotations, rev, false);
}


void autonomousRoutine() {

hoardForTime(2000);
driveStraight(22);




}


