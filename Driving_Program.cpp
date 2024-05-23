// this essentially takes in the sbus signal, modifies channel 1 and 4 for the steering+throttle, and shits out the new sbus
#include "Driving.h"

#include <HAL.h>
#include <math.h>

Driving::Driving(SBUS* x8r)
    : x8r_(x8r), channels_{0.0}, 
