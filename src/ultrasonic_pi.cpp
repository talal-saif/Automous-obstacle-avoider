#include "ultrasonic.hpp"
#include <chrono>
#include <thread>
#ifdef HAVE_PIGPIO
#include <pigpio.h>
#endif

class UltrasonicPi : public Ultrasonic {
public:
  void begin(const Params &p) override {
#ifdef HAVE_PIGPIO
    params_ = p;
    gpioSetMode(p.pins.trig, PI_OUTPUT);
    gpioSetMode(p.pins.echo, PI_INPUT);
    gpioWrite(p.pins.trig, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
#else
    (void)p;
#endif
  }

  int read_distance_cm() override {
#ifdef HAVE_PIGPIO
    // نبضة 10 ميكروثانية
    gpioWrite(params_.pins.trig, 1);
    gpioDelay(10);
    gpioWrite(params_.pins.trig, 0);

    uint32_t start = 0, end = 0;
    uint32_t timeout = gpioTick() + 30000; // 30ms
    while (gpioRead(params_.pins.echo) == 0) {
      if (gpioTick() > timeout)
        return 0;
    }
    start = gpioTick();
    timeout = gpioTick() + 30000;
    while (gpioRead(params_.pins.echo) == 1) {
      if (gpioTick() > timeout)
        return 0;
    }
    end = gpioTick();

    uint32_t diff =
        (end >= start) ? (end - start) : (end + (0xFFFFFFFF - start));
    return static_cast<int>(diff / 58.0); // ≈58µs/سم (ذهاب وعودة)
#else
    return 0;
#endif
  }

#ifdef HAVE_PIGPIO
  Params params_{};
#endif
};

Ultrasonic *create_ultrasonic_pi() { return new UltrasonicPi(); }
