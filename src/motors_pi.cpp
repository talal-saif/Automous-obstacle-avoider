#include "motors.hpp"
#include <iostream>
#ifdef HAVE_PIGPIO
#include <pigpio.h>
#endif

class MotorsPi : public Motors {
public:
  void begin(const Params &p) override {
#ifdef HAVE_PIGPIO
    params_ = p;
    if (gpioInitialise() < 0) {
      std::cerr << "[PI] pigpio init failed\n";
      ok_ = false;
      return;
    }
    gpioSetMode(p.pins.motor_left_pwm, PI_OUTPUT);
    gpioSetMode(p.pins.motor_left_in1, PI_OUTPUT);
    gpioSetMode(p.pins.motor_left_in2, PI_OUTPUT);
    gpioSetMode(p.pins.motor_right_pwm, PI_OUTPUT);
    gpioSetMode(p.pins.motor_right_in1, PI_OUTPUT);
    gpioSetMode(p.pins.motor_right_in2, PI_OUTPUT);
    ok_ = true;
#else
    (void)p;
    std::cerr << "[PI] pigpio not available at build time\n";
    ok_ = false;
#endif
  }

  void stop() override { drive(0, 0, 0, 0); }
  void forward(int pwm) override { drive(pwm, 1, pwm, 1); }
  void back(int pwm) override { drive(pwm, -1, pwm, -1); }
  void turn_left(int pwm) override { drive(pwm, -1, pwm, 1); }
  void turn_right(int pwm) override { drive(pwm, 1, pwm, -1); }

private:
  void drive(int pwmL, int dirL, int pwmR, int dirR) {
#ifdef HAVE_PIGPIO
    if (!ok_)
      return;
    // Left
    gpioWrite(params_.pins.motor_left_in1, dirL > 0);
    gpioWrite(params_.pins.motor_left_in2, dirL < 0);
    gpioPWM(params_.pins.motor_left_pwm, pwmL);
    // Right
    gpioWrite(params_.pins.motor_right_in1, dirR > 0);
    gpioWrite(params_.pins.motor_right_in2, dirR < 0);
    gpioPWM(params_.pins.motor_right_pwm, pwmR);
#else
    (void)pwmL;
    (void)dirL;
    (void)pwmR;
    (void)dirR;
#endif
  }

#ifdef HAVE_PIGPIO
  Params params_{};
  bool ok_ = false;
#endif
};

Motors *create_motors_pi() { return new MotorsPi(); }
