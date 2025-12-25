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

double startTime = Brain.timer(vex::msec);

driveStraight(14);

double endTime = Brain.timer(vex::msec);





}


