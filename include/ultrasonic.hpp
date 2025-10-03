#pragma once
struct IUltrasonic {
  virtual ~IUltrasonic() = default;
  // returns cm (typical range 5..400 for HC-SR04)
  virtual float distance_cm() = 0;
};
