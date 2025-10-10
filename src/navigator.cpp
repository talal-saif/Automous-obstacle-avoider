#include "navigator.hpp"
#include "logger.hpp"
#include <chrono>
#include <iostream>
#include <thread>

Navigator::Navigator(std::unique_ptr<Motors> m, std::unique_ptr<Ultrasonic> u,
                     const Params &p, Logger *logger)
    : motors_(std::move(m)), us_(std::move(u)), params_(p), logger_(logger) {}

void Navigator::loop_once() {
  int d = us_->read_distance_cm();
  last_distance_ = d;
  std::string action;

  if (d == 0) {
    action = "sensor_glitch_stop";
    motors_->stop();
  } else if (d < params_.safe_distance_cm) {
    action = "back_then_turn_left";
    motors_->back(params_.max_speed_pwm / 2);
    std::this_thread::sleep_for(std::chrono::milliseconds(params_.backoff_ms));
    motors_->turn_left(params_.max_speed_pwm / 2);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(params_.scan_turn_ms));
  } else {
    action = "forward";
    motors_->forward(params_.max_speed_pwm);
  }

  if (logger_)
    logger_->write(d, action);
  std::this_thread::sleep_for(std::chrono::milliseconds(params_.loop_delay_ms));
}
