#pragma once

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "v5.h"
#include "v5_vcs.h"

// Project specific configuration
#include "robot-config.h"

using namespace vex;

// Global initialization function
void vexcodeInit(void);

// Convenience macros
#define waitUntil(condition)         \
  do {                               \
    wait(5, msec);                   \
  } while (!(condition))

#define repeat(iterations)           \
  for (int iterator = 0; iterator < iterations; iterator++)
