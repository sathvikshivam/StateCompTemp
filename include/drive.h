#pragma once
#include "vex.h"
#include <cmath>

void driveVolts(double left, double right);
void turnTo(double targetTheta);
void driveToPoint(double tx, double ty);
double angleWrap(double a);
