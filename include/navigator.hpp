#pragma once
#include "motors.hpp"
#include "params.hpp"
#include "ultrasonic.hpp"
#include <memory>
#include <string>

class Logger; // forward-declare

class Navigator {
public:
  Navigator(std::unique_ptr<Motors> m, std::unique_ptr<Ultrasonic> u,
            const Params &p, Logger *logger = nullptr);
  void loop_once();

private:
  std::unique_ptr<Motors> motors_;
  std::unique_ptr<Ultrasonic> us_;
  Params params_;
  int last_distance_ = 0;
  Logger *logger_ = nullptr;
};
