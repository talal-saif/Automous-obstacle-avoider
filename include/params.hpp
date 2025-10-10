#pragma once
#include <string>

struct Pins {
  int trig = 23;
  int echo = 24;
  int motor_left_pwm = 12;
  int motor_left_in1 = 5;
  int motor_left_in2 = 6;
  int motor_right_pwm = 13;
  int motor_right_in1 = 19;
  int motor_right_in2 = 26;
};

struct Params {
  int safe_distance_cm = 25;
  int backoff_ms = 300;
  int scan_turn_ms = 350;
  int loop_delay_ms = 60;
  int max_speed_pwm = 180;  // 0..255
  std::string mode = "sim"; // sim | pi
  Pins pins;
};

Params load_params(const std::string &path);
