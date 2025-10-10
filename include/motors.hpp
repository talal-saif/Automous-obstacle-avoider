#pragma once
#include "params.hpp"

class Motors {
public:
  virtual ~Motors() = default;
  virtual void begin(const Params &) = 0;
  virtual void stop() = 0;
  virtual void forward(int pwm) = 0;
  virtual void back(int pwm) = 0;
  virtual void turn_left(int pwm) = 0;
  virtual void turn_right(int pwm) = 0;
};

Motors *create_motors_sim();
Motors *create_motors_pi();
