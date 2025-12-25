#pragma once

// Global robot pose (in inches and radians)
extern double x;
extern double y;
extern double theta;

// Odometry control
void resetOdometry();
void startOdometry();
