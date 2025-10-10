#pragma once
#include "params.hpp"

class Ultrasonic {
public:
  virtual ~Ultrasonic() = default;
  virtual void begin(const Params &) = 0;
  virtual int read_distance_cm() = 0; // 0 لو فشل/تايم آوت
};

Ultrasonic *create_ultrasonic_sim();
Ultrasonic *create_ultrasonic_pi();
