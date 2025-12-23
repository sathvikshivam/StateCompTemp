#include "vex.h"
#include "auton.h"
#include "usercontrol.h"

competition Competition;

#define AUTON_ONLY 1   // 🔁 change to 0 when done

void autonomous() {
  autonomousRoutine();
}

void usercontrol() {
  userControlRoutine();
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
