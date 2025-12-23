#include "robot-config.h"   // REQUIRED
#include "odometry.h"
#include "drive.h"

constexpr double WHEEL_DIAMETER = 2.75;
constexpr double WHEEL_CIRC = M_PI * WHEEL_DIAMETER;
constexpr double GEAR_RATIO = 36.0 / 48.0;

double x = 0, y = 0, theta = 0;

static double prevLeftIn = 0;
static double prevRightIn = 0;
static double lastHeadingRad = 0;
static double continuousTheta = 0;

double motorDegToInches(double deg) {
  return (deg / 360.0) * GEAR_RATIO * WHEEL_CIRC;
}


void updateOdometry() {
  double leftDeg =
    (LeftFront.position(deg) + LeftBack.position(deg)) / 2.0;
  double rightDeg =
    (RightFront.position(deg) + RightBack.position(deg)) / 2.0;

  double leftIn = motorDegToInches(leftDeg);
  double rightIn = motorDegToInches(rightDeg);

  double dL = leftIn - prevLeftIn;
  double dR = rightIn - prevRightIn;

  prevLeftIn = leftIn;
  prevRightIn = rightIn;

  double ds = (dL + dR) / 2.0;

  double rawTheta = Inertial.heading(deg) * M_PI / 180.0;
  double dTheta = angleWrap(rawTheta - lastHeadingRad);
  continuousTheta += dTheta;
  lastHeadingRad = rawTheta;

  theta = continuousTheta;

  x += ds * cos(theta);
  y += ds * sin(theta);
}

int odomTask() {
  while (true) {
    updateOdometry();
    wait(10, msec);
  }
  return 0;
}

void resetOdometry() {
  LeftFront.setPosition(0, deg);
  LeftBack.setPosition(0, deg);
  RightFront.setPosition(0, deg);
  RightBack.setPosition(0, deg);

  x = y = theta = 0;
  prevLeftIn = prevRightIn = 0;

  Inertial.resetHeading();
  while (Inertial.isCalibrating()) wait(10, msec);

  lastHeadingRad = 0;
  continuousTheta = 0;
}

void startOdometry() {
  task odom(odomTask);
}
