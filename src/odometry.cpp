#include "robot-config.h"
#include "odometry.h"
#include <cmath>

// ------------------ Constants ------------------
constexpr double WHEEL_DIAMETER = 2.75;
constexpr double WHEEL_CIRC = M_PI * WHEEL_DIAMETER;
constexpr double GEAR_RATIO = 36.0 / 48.0; // motor → wheel

// ------------------ Global pose ----------------
double x = 0;       // inches
double y = 0;       // inches
double theta = 0;   // radians

static double prevLeftIn = 0;
static double prevRightIn = 0;

// ------------------ Helpers --------------------
double motorDegToIn(double deg) {
  return (deg / 360.0) * GEAR_RATIO * WHEEL_CIRC;
}
double angleWrap(double a) {
  while (a > M_PI)  a -= 2 * M_PI;
  while (a < -M_PI) a += 2 * M_PI;
  return a;
}

// ------------------ Update ---------------------
void updateOdometry() {
  double leftDeg =
    (LeftFront.position(deg) + LeftBack.position(deg)) / 2.0;
  double rightDeg =
    (RightFront.position(deg) + RightBack.position(deg)) / 2.0;

  double leftIn  = motorDegToIn(leftDeg);
  double rightIn = motorDegToIn(rightDeg);

  double dL = leftIn  - prevLeftIn;
  double dR = rightIn - prevRightIn;

  prevLeftIn  = leftIn;
  prevRightIn = rightIn;

  double ds = (dL + dR) / 2.0;

  // Sideways-mounted inertial → rotation()
theta = Inertial.heading(deg) * M_PI / 180.0;   // 0 → 2π, NEVER wrapped
  // If turning left makes theta go down, flip sign:
  // theta = -Inertial.rotation(deg) * M_PI / 180.0;

  x += ds * sin(theta);
  y += ds * cos(theta);
}

// ------------------ Task -----------------------
int odomTask() {
  while (true) {
    updateOdometry();
    wait(10, msec);
  }
  return 0;
}

// ------------------ API ------------------------
void resetOdometry() {
  LeftFront.setPosition(0, deg);
  LeftBack.setPosition(0, deg);
  RightFront.setPosition(0, deg);
  RightBack.setPosition(0, deg);

  prevLeftIn = prevRightIn = 0;
  x = y = theta = 0;

  Inertial.resetRotation();
  while (Inertial.isCalibrating()) {
    wait(10, msec);
  }
}

void startOdometry() {
  task odom(odomTask);
}
