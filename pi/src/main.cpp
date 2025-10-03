#include "motors.hpp"
#include "navigator.hpp"
#include "ultrasonic.hpp"
#include <algorithm>
#include <chrono>
#include <cstdint> // uint32_t
#include <iostream>
#include <pigpio.h>
#include <thread>

using u32 = uint32_t;

static inline void sleep_us(unsigned us) {
  std::this_thread::sleep_for(std::chrono::microseconds(us));
}

struct PiMotors : IMotors {
  const unsigned IN1 = 17, IN2 = 27, ENA = 22;
  const unsigned IN3 = 23, IN4 = 24, ENB = 25;

  PiMotors() {
    gpioSetMode(IN1, static_cast<unsigned>(PI_OUTPUT));
    gpioSetMode(IN2, static_cast<unsigned>(PI_OUTPUT));
    gpioSetMode(IN3, static_cast<unsigned>(PI_OUTPUT));
    gpioSetMode(IN4, static_cast<unsigned>(PI_OUTPUT));
    gpioSetMode(ENA, static_cast<unsigned>(PI_OUTPUT));
    gpioSetMode(ENB, static_cast<unsigned>(PI_OUTPUT));
    stop();
  }

  void forward(int pwm) override {
    gpioWrite(IN1, static_cast<unsigned>(1));
    gpioWrite(IN2, static_cast<unsigned>(0));
    gpioWrite(IN3, static_cast<unsigned>(1));
    gpioWrite(IN4, static_cast<unsigned>(0));
    gpioPWM(ENA, static_cast<unsigned>(std::clamp(pwm, 0, 255)));
    gpioPWM(ENB, static_cast<unsigned>(std::clamp(pwm, 0, 255)));
  }
  void backward(int pwm) override {
    gpioWrite(IN1, static_cast<unsigned>(0));
    gpioWrite(IN2, static_cast<unsigned>(1));
    gpioWrite(IN3, static_cast<unsigned>(0));
    gpioWrite(IN4, static_cast<unsigned>(1));
    gpioPWM(ENA, static_cast<unsigned>(std::clamp(pwm, 0, 255)));
    gpioPWM(ENB, static_cast<unsigned>(std::clamp(pwm, 0, 255)));
  }
  void turnRight(int pwm) override {
    gpioWrite(IN1, static_cast<unsigned>(1));
    gpioWrite(IN2, static_cast<unsigned>(0));
    gpioWrite(IN3, static_cast<unsigned>(0));
    gpioWrite(IN4, static_cast<unsigned>(1));
    gpioPWM(ENA, static_cast<unsigned>(std::clamp(pwm, 0, 255)));
    gpioPWM(ENB, static_cast<unsigned>(std::clamp(pwm, 0, 255)));
  }
  void turnLeft(int pwm) override {
    gpioWrite(IN1, static_cast<unsigned>(0));
    gpioWrite(IN2, static_cast<unsigned>(1));
    gpioWrite(IN3, static_cast<unsigned>(1));
    gpioWrite(IN4, static_cast<unsigned>(0));
    gpioPWM(ENA, static_cast<unsigned>(std::clamp(pwm, 0, 255)));
    gpioPWM(ENB, static_cast<unsigned>(std::clamp(pwm, 0, 255)));
  }
  void stop() override {
    gpioWrite(IN1, static_cast<unsigned>(0));
    gpioWrite(IN2, static_cast<unsigned>(0));
    gpioWrite(IN3, static_cast<unsigned>(0));
    gpioWrite(IN4, static_cast<unsigned>(0));
    gpioPWM(ENA, static_cast<unsigned>(0));
    gpioPWM(ENB, static_cast<unsigned>(0));
  }
};

struct PiUltrasonic : IUltrasonic {
  const unsigned TRIG = 5, ECHO = 6;
  const u32 TIMEOUT_US = 25000u; // خليه unsigned لتفادي تحذير المقارنة

  PiUltrasonic() {
    gpioSetMode(TRIG, static_cast<unsigned>(PI_OUTPUT));
    gpioSetMode(ECHO, static_cast<unsigned>(PI_INPUT));
    gpioWrite(TRIG, static_cast<unsigned>(0));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  float once_cm() {
    // 10us trigger
    gpioWrite(TRIG, static_cast<unsigned>(0));
    sleep_us(2);
    gpioWrite(TRIG, static_cast<unsigned>(1));
    sleep_us(10);
    gpioWrite(TRIG, static_cast<unsigned>(0));

    u32 t0 = gpioTick();
    while (gpioRead(ECHO) == 0) {
      if (gpioTick() - t0 > TIMEOUT_US)
        return 400.0f;
    }
    u32 start = gpioTick();
    while (gpioRead(ECHO) == 1) {
      if (gpioTick() - start > TIMEOUT_US)
        return 400.0f;
    }
    u32 end = gpioTick();
    return float(end - start) / 58.0f; // cm
  }

  float distance_cm() override {
    float v[5];
    for (int i = 0; i < 5; i++) {
      v[i] = once_cm();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::sort(v, v + 5);
    return v[2];
  }
};

int main() {
  if (gpioInitialise() < 0) {
    std::cerr << "pigpio init failed. Run: sudo pigpiod\n";
    return 1;
  }

  PiMotors motors;
  PiUltrasonic ultra;
  Navigator nav(motors, ultra);

  nav.DRIVE_PWM = 180;
  nav.TURN_PWM = 170;
  nav.BACK_PWM = 150;
  nav.STOP_CM = 25.f;
  nav.CLEAR_CM = 40.f;

  std::cout << "Robot running. Ctrl+C to exit.\n";
  nav.run_for_seconds(300.0f);
  gpioTerminate();
  return 0;
}
