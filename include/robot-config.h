#pragma once
#include "vex.h"

using namespace vex;

// Brain & controller
extern brain Brain;
extern controller Controller1;

// ---------------- Drive motors ----------------
extern motor LeftFront;
extern motor LeftBack;
extern motor RightFront;
extern motor RightBack;

extern motor_group leftDrive;
extern motor_group rightDrive;

// ---------------- Other motors ----------------
extern motor intakeMotor;
extern motor conveyorMotor;
extern motor hoodMotor;
extern motor scoreMotor;

// ---------------- Sensors ----------------
extern inertial Inertial;

// ---------------- Pneumatics ----------------
extern digital_out scraper;
extern digital_out aligner;
