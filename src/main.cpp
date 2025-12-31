#include "auton.h"
#include "usercontrol.h"

#include "vex.h"
#include <string>

using namespace vex;
competition Competition;

// ===== MANUAL OVERRIDES =====
// Set these to true when you want to hard-code values
#define FORCE_ALLIANCE  true
#define FORCE_SIDE      true

// ===== DEFAULT / MANUAL VALUES =====
std::string alliance  = "red";    // used if FORCE_ALLIANCE = true
std::string autonSide = "left";   // used if FORCE_SIDE = true

int getSideSign() {
  if (autonSide == "left") return 1;
  if (autonSide == "right") return -1;
  // Fallback to right-side behavior if an unexpected value is provided
  return 1;
}
#define AUTON_ONLY 1   // 🔁 change to 0 when done
void updateControllerScreen() {
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
Controller1.Screen.print("Alliance: %s%s",
  alliance.c_str(),
  FORCE_ALLIANCE ? " (LOCK)" : "");
  Controller1.Screen.setCursor(2, 1);
Controller1.Screen.print("Side: %s%s",
  autonSide.c_str(),
  FORCE_SIDE ? " (LOCK)" : "");
}


void autonomous() {
  autonomousRoutine();
}

void usercontrol() {
  //userControlRoutine();
}
void pre_auton(void) {

  while (!Competition.isEnabled()) {

    // -------- Alliance select (only if NOT forced) --------
    if (!FORCE_ALLIANCE) {
      if (Controller1.ButtonX.pressing()) {
        alliance = "red";
      }
      if (Controller1.ButtonB.pressing()) {
        alliance = "blue";
      }
    }

    // -------- Side select (only if NOT forced) --------
    if (!FORCE_SIDE) {
      if (Controller1.ButtonLeft.pressing()) {
        autonSide = "left";
      }
      if (Controller1.ButtonRight.pressing()) {
        autonSide = "right";
      }
    }

    updateControllerScreen();
    wait(200, msec);
  }
}



int main() {
  vexcodeInit();
#if AUTON_ONLY
  autonomousRoutine();
#else

  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
  while (true) wait(100, msec);
#endif
}



