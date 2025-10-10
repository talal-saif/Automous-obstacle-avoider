#include "logger.hpp"
#include "motors.hpp"
#include "navigator.hpp"
#include "params.hpp"
#include "ultrasonic.hpp"
#include <iostream>
#include <memory>
#include <yaml-cpp/yaml.h>

// تحميل الإعدادات
Params load_params(const std::string &path) {
  Params p;
  auto root = YAML::LoadFile(path);
  if (root["safe_distance_cm"])
    p.safe_distance_cm = root["safe_distance_cm"].as<int>();
  if (root["backoff_ms"])
    p.backoff_ms = root["backoff_ms"].as<int>();
  if (root["scan_turn_ms"])
    p.scan_turn_ms = root["scan_turn_ms"].as<int>();
  if (root["loop_delay_ms"])
    p.loop_delay_ms = root["loop_delay_ms"].as<int>();
  if (root["max_speed_pwm"])
    p.max_speed_pwm = root["max_speed_pwm"].as<int>();
  if (root["mode"])
    p.mode = root["mode"].as<std::string>();
  if (root["pins"]) {
    auto pn = root["pins"];
    auto &pp = p.pins;
    if (pn["trig"])
      pp.trig = pn["trig"].as<int>();
    if (pn["echo"])
      pp.echo = pn["echo"].as<int>();
    if (pn["motor_left_pwm"])
      pp.motor_left_pwm = pn["motor_left_pwm"].as<int>();
    if (pn["motor_left_in1"])
      pp.motor_left_in1 = pn["motor_left_in1"].as<int>();
    if (pn["motor_left_in2"])
      pp.motor_left_in2 = pn["motor_left_in2"].as<int>();
    if (pn["motor_right_pwm"])
      pp.motor_right_pwm = pn["motor_right_pwm"].as<int>();
    if (pn["motor_right_in1"])
      pp.motor_right_in1 = pn["motor_right_in1"].as<int>();
    if (pn["motor_right_in2"])
      pp.motor_right_in2 = pn["motor_right_in2"].as<int>();
  }
  return p;
}

Motors *create_motors_pi();
Ultrasonic *create_ultrasonic_pi();

int main() {
  auto params = load_params("config/params.yaml");

  Logger logger("run_log.csv");
  std::unique_ptr<Motors> m(create_motors_pi());
  std::unique_ptr<Ultrasonic> u(create_ultrasonic_pi());
  m->begin(params);
  u->begin(params);

  Navigator nav(std::move(m), std::move(u), params, &logger);
  std::cout << "[PI] Running... logging to run_log.csv (Ctrl+C to quit)\n";
  while (true)
    nav.loop_once();
}
