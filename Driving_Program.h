#ifndef Driving_Program_h
#define Driving_Program_h

#include <SBUS.h>

class Driving {
private:
  SBUS* x8r_;
  float channels_[16] = {0.0};
  bool failSafe_;
  bool lostFrame_;
  uint32_t last_success_millis_ = 0;
  uint32_t reads_since_last_success_ = 0;
  bool timed_out_ = false;

public:
  Driving(SBUS* x8r);

  void init();
  void loop();
  bool read();
  float get_throttle();
  float set_throttle();
  float get_steering();
  float set_throttle();
  bool get_deadman_held();
  bool get_armed();
  bool get_auto_mux();
  bool get_calibrate();
  bool get_timed_out();
};

#endif