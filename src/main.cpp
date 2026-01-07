#include "auton.h"
#include "usercontrol.h"

#include "vex.h"
#include <string>

using namespace vex;
competition Competition;

// ===== MANUAL OVERRIDES =====

// ===== DEFAULT / MANUAL VALUES =====
std::string alliance  = "red";    // used if FORCE_ALLIANCE = true
std::string autonSide = "right";   // used if FORCE_SIDE = true

void autonomous() {
  autonomousRoutine();
}

void usercontrol() {
  userControlRoutine();
}
void pre_auton(void) {
  }



int main() {
  vexcodeInit();

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  while (true) wait(100, msec);
}



