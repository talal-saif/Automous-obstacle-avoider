#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <pigpio.h>
#include <thread>

using u32 = uint32_t;
static inline void sleep_us(unsigned us) {
  std::this_thread::sleep_for(std::chrono::microseconds(us));
}

struct PiMotors {
  const unsigned IN1 = 17, IN2 = 27, ENA = 22;
  const unsigned IN3 = 23, IN4 = 24, ENB = 25;

  PiMotors() {
    gpioSetMode(IN1, PI_OUTPUT);
    gpioSetMode(IN2, PI_OUTPUT);
    gpioSetMode(IN3, PI_OUTPUT);
    gpioSetMode(IN4, PI_OUTPUT);
    gpioSetMode(ENA, PI_OUTPUT);
    gpioSetMode(ENB, PI_OUTPUT);
    stop();
  }
  void setPWM(unsigned gpio, unsigned duty) {
    duty = std::clamp(duty, 0u, 255u);
    gpioPWM(gpio, duty);
  }
  void forward(int pwm) {
    pwm = std::clamp(pwm, 0, 255);
    gpioWrite(IN1, 1);
    gpioWrite(IN2, 0);
    gpioWrite(IN3, 1);
    gpioWrite(IN4, 0);
    setPWM(ENA, pwm);
    setPWM(ENB, pwm);
  }
  void left(int pwm) {
    pwm = std::clamp(pwm, 0, 255);
    gpioWrite(IN1, 0);
    gpioWrite(IN2, 1);
    gpioWrite(IN3, 1);
    gpioWrite(IN4, 0);
    setPWM(ENA, pwm);
    setPWM(ENB, pwm);
  }
  void stop() {
    setPWM(ENA, 0);
    setPWM(ENB, 0);
    gpioWrite(IN1, 0);
    gpioWrite(IN2, 0);
    gpioWrite(IN3, 0);
    gpioWrite(IN4, 0);
  }
};

struct PiUltrasonic {
  const unsigned TRIG = 5, ECHO = 6;
  const u32 TIMEOUT_US = 250000; // 250ms

  PiUltrasonic() {
    gpioSetMode(TRIG, PI_OUTPUT);
    gpioSetMode(ECHO, PI_INPUT);
    gpioWrite(TRIG, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  float once_cm() {
    gpioWrite(TRIG, 0);
    sleep_us(2);
    gpioWrite(TRIG, 1);
    sleep_us(10);
    gpioWrite(TRIG, 0);

    u32 start = gpioTick();
    while (gpioRead(ECHO) == 0) {
      if (gpioTick() - start > TIMEOUT_US)
        return 400.0f;
    }
    u32 t0 = gpioTick();
    while (gpioRead(ECHO) == 1) {
      if (gpioTick() - t0 > TIMEOUT_US)
        return 400.0f;
    }
    u32 t1 = gpioTick();
    float us = float(t1 - t0);
    return us / 58.0f; // cm
  }

  float distance_cm() {
    float v[5];
    for (int i = 0; i < 5; i++) {
      v[i] = once_cm();
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::sort(v, v + 5);
    return v[2]; // median
  }
};

int main() {
  if (gpioInitialise() < 0) {
    std::fprintf(stderr, "pigpio init failed\n");
    return 1;
  }

  PiMotors m;
  PiUltrasonic us;

  const int FWD_PWM = 160;
  const int TURN_PWM = 150;
  const float SAFE = 25.0f; // cm

  while (true) {
    float d = us.distance_cm();
    std::printf("dist = %.1f cm\n", d);

    if (d < SAFE) {
      m.left(TURN_PWM);
      std::this_thread::sleep_for(std::chrono::milliseconds(350));
      m.stop();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } else {
      m.forward(FWD_PWM);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  gpioTerminate();
  return 0;
}
