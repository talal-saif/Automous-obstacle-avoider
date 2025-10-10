#include "motors.hpp"
#include <iostream>

class MotorsSim : public Motors {
public:
  void begin(const Params &) override { std::cout << "[SIM] Motors ready\n"; }
  void stop() override { std::cout << "[SIM] STOP\n"; }
  void forward(int pwm) override {
    std::cout << "[SIM] FORWARD pwm=" << pwm << "\n";
  }
  void back(int pwm) override { std::cout << "[SIM] BACK pwm=" << pwm << "\n"; }
  void turn_left(int pwm) override {
    std::cout << "[SIM] TURN LEFT pwm=" << pwm << "\n";
  }
  void turn_right(int pwm) override {
    std::cout << "[SIM] TURN RIGHT pwm=" << pwm << "\n";
  }
};

Motors *create_motors_sim() { return new MotorsSim(); }
