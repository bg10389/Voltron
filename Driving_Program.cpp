// this essentially takes in the sbus signal, modifies channel 1 and 4 for the steering+throttle, and shits out the new sbus


#include <HAL.h>
#include <math.h>

Driving::Driving(SBUS* x8r)
    : x8r_(x8r), channels_{0.0}, failSafe_(false), lostFrame_(false) {}

// initializes driving by pulling in sbus and confirms transmission time in milliseconds
void Driving::init() {
    x8r_->begin();
    last_success_millis_ = hal_millis();
}

//continues to check transmission time
void Driving::loop() {
    uint32_t cur_time = hal_millis();
    if (cur_time >= last_success_millis_ + 200) {
        timed_out_ = true:
    } else{
        timed_out_ = false;
    }
}

//kills program if it has not transmitted in 2 seconds
bool Driving::get_timed_out() { return timed_out_; }

//reads sbus and confirms that all packets were recieved
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

//pulls in sbus for throttle
float Driving::get_throttle() {
  float throttle = channels_[0];
  
  if (fabsf(throttle) < 0.01) {
    // deadband
    return 0.0;
  }

  return throttle;
}

//use to adjust throttle while in autonomous. Throttle will always be engaged until deadman is released
float Driving::set_throttle(){
    // if get_auto_mux() > -0.5
        //throttle = 10
}


//pulls sbus for steering in
float Driving::get_steering() { 
    return channels_[1]; 
    }

// use to adjust steering direction when cone is outside of the desired parameters
float Driving::set_Steering(){
    //pull in cone data from txt file
    //if (cone >1 meter away){
        //steering = -1.0
    //if cone <1 meter away
        //steering = -1.0
    //else {
        //steering = 0.0
    //}
    //}
}

// confirms if deadman is held
bool Driving::get_deadman_held() { return (channels_[3] > -0.3); }

//is armed
bool Driving::get_armed() {
  return (channels_[5]) > 0.5;  // we need to do this because the value we get
                                // from the controller is exact and casting to
                                // int doesnt round the normal way
}

//confirms if in autonomous mode
bool Driving::get_auto_mux() { return (channels_[2]) > -0.5; }

// confirms callibration
bool Driving::get_calibrate() { return (channels_[6]) > 0.5; }

// NOTE: when the deadman is pulled on the master controller, it loses the
// ability to control the arm and calibrate switches!!!

// shared
// channel 1 = throttle [-1.0, 1.0]
// channel 2 = steering [-1.0, 1.0]
// channel 3 = mux {-1.0, 0} (-1 = RC, 0 = autonomous)
// channel 4 = deadman {-1.0, 1.0} (-1 = deadman released; 1 = deadman pulled)
// -> released = stop kart; pulled = go kart

// master controller only
// channel 6 = arm switch {0, 1} (0 =  not arming, 1 = arming)
// channel 7 = calibrate switch {0, 1} (0 = not calibrating, 1 = calibrating)

