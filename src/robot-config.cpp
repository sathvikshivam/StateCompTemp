#include "robot-config.h"

// ---------------- Brain & controller ----------------
brain Brain;
controller Controller1;

// ---------------- Drive motors ----------------
motor LeftFront  = motor(PORT10, ratio18_1, true);   // reversed YES
motor LeftBack   = motor(PORT9,  ratio18_1, true);   // reversed YES
motor RightFront = motor(PORT2,  ratio18_1, false);  // reversed NO
motor RightBack  = motor(PORT1,  ratio18_1, false);  // reversed NO

motor_group leftDrive(LeftFront, LeftBack);
motor_group rightDrive(RightFront, RightBack);

// ---------------- Other motors ----------------
motor intakeMotor   = motor(PORT8,  ratio18_1, true);
motor conveyorMotor = motor(PORT4,  ratio18_1, false);
motor hoodMotor     = motor(PORT15, ratio18_1, false);
motor scoreMotor    = motor(PORT7,  ratio18_1, true);   // reversed YES

// ---------------- Sensors ----------------
inertial Inertial = inertial(PORT5);

// ---------------- Pneumatics ----------------
digital_out scraper = digital_out(Brain.ThreeWirePort.E);
digital_out aligner = digital_out(Brain.ThreeWirePort.H);   

// ---------------- VEXCODE INIT ----------------
void vexcodeInit(void) {
  // Calibrate sensors if needed
  Inertial.calibrate();
}