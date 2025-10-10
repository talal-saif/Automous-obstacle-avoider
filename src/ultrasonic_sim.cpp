#include "ultrasonic.hpp"
#include <random>

class UltrasonicSim : public Ultrasonic {
public:
  void begin(const Params &) override { rng.seed(std::random_device{}()); }
  int read_distance_cm() override {
    // طريق شبه فاضي مع فرصة لعائق قريب
    std::uniform_int_distribution<int> dist(10, 120);
    int d = dist(rng);
    if (std::uniform_int_distribution<int>(0, 4)(rng) == 0) {
      d = std::uniform_int_distribution<int>(10, 40)(rng);
    }
    return d;
  }

private:
  std::mt19937 rng;
};

Ultrasonic *create_ultrasonic_sim() { return new UltrasonicSim(); }
