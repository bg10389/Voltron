// this essentially takes in the sbus signal, modifies channel 1 and 4 for the steering+throttle, and shits out the new sbus


#include <HAL.h>
#include <math.h>

Driving::Driving(SBUS* x8r)
    : x8r_(x8r), channels_{0.0}, failSafe_(false), lostFrame_(false) {}

void Driving::init() {
    x8r_->begin();
    last_success_millis_ = hal_millis();
}

void Driving::loop() {
    uint32_t cur_time = hal_millis();
    if (cur_time >= last_success_millis_ + 200) {
        timed_out_ = true:
    } else{
        timed_out_ = false;
    }
}

