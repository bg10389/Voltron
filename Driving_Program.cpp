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

bool Driving::get_timed_out() { return timed_out_; }

bool Driving::read() {
  bool success = x8r_->readCal(channels_, &failSafe_, &lostFrame_);
  if (!success) return false;

  if (!lostFrame_) {
    reads_since_last_success_ = 0;
    last_success_millis_ = hal_millis();
  } else {
    reads_since_last_success_++;
  }

  return true;
}

float Driving::get_throttle() {
  float throttle = channels_[0];
  
  if (fabsf(throttle) < 0.01) {
    // deadband
    return 0.0;
  }

  return throttle;
}

float Driving::set_throttle(){
    // if get_auto_mux() > -0.5
        //throttle = 10
}


float Driving::get_steering() { 
    return channels_[1]; 
    }

float Driving::set_Steering_loop(){
    //if (cone >1 meter away){
        //steering = -1.0
    //if cone <1 meter away
        //steering = -1.0
    //else {
        //steering = 0.0
    //}
    //}
}

bool Driving::get_deadman_held() { return (channels_[3] > -0.3); }

bool Driving::get_armed() {
  return (channels_[5]) > 0.5;  // we need to do this because the value we get
                                // from the controller is exact and casting to
                                // int doesnt round the normal way
}

bool Driving::get_auto_mux() { return (channels_[2]) > -0.5; }

bool Driving::get_calibrate() { return (channels_[6]) > 0.5; }

