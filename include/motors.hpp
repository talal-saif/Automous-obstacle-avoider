#pragma once
struct IMotors {
  virtual ~IMotors() = default;
  virtual void forward(int pwm) = 0;
  virtual void backward(int pwm) = 0;
  virtual void turnRight(int pwm) = 0; // pivot
  virtual void turnLeft(int pwm) = 0;  // pivot
  virtual void stop() = 0;
};
