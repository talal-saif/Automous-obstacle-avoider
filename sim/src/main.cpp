#include "motors.hpp"
#include "navigator.hpp"
#include "ultrasonic.hpp"
#include <algorithm>
#include <iostream>
#include <random>

struct MockMotors : IMotors {
  void forward(int pwm) override {
    std::cout << "[MOTOR] forward pwm=" << pwm << "\n";
  }
  void backward(int pwm) override {
    std::cout << "[MOTOR] backward pwm=" << pwm << "\n";
  }
  void turnRight(int pwm) override {
    std::cout << "[MOTOR] RIGHT   pwm=" << pwm << "\n";
  }
  void turnLeft(int pwm) override {
    std::cout << "[MOTOR] LEFT    pwm=" << pwm << "\n";
  }
  void stop() override { std::cout << "[MOTOR] STOP\n"; }
};

struct MockUltrasonic : IUltrasonic {
  std::mt19937 rng{321};
  std::uniform_real_distribution<float> jitter{-3.f, 3.f};
  float obstacle_cm = 400.f; // 400 = clear

  float distance_cm() override {
    std::uniform_int_distribution<int> chance(1, 10);
    std::uniform_real_distribution<float> newObs(15.f, 60.f);
    if (chance(rng) == 1)
      obstacle_cm = newObs(rng);

    if (obstacle_cm < 380.f)
      obstacle_cm = std::max(5.f, obstacle_cm - 5.f);
    else
      obstacle_cm = 400.f;

    float noisy = obstacle_cm + jitter(rng);
    return std::clamp(noisy, 5.f, 400.f);
  }
};

int main() {
  std::cout << "=== SIM START ===\n";
  MockMotors motors;
  MockUltrasonic ultra;
  Navigator nav(motors, ultra);

  nav.DRIVE_PWM = 170;
  nav.STOP_CM = 25.f;
  nav.CLEAR_CM = 40.f;

  nav.run_for_seconds(12.0f);
  std::cout << "=== SIM END ===\n";
  return 0;
}
