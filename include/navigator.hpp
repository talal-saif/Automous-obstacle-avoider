#pragma once
#include "motors.hpp"
#include "ultrasonic.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

inline void sleep_ms(int ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

struct Navigator {
  IMotors &m;
  IUltrasonic &u;

  // Tunables
  int DRIVE_PWM = 170;
  int TURN_PWM = 160;
  int BACK_PWM = 140;
  float STOP_CM = 25.0f;  // nearer than this => obstacle
  float CLEAR_CM = 40.0f; // farther than this => clear

  explicit Navigator(IMotors &motors, IUltrasonic &ultra)
      : m(motors), u(ultra) {}

  bool pathIsClear(float th_cm) {
    std::vector<float> v;
    v.reserve(5);
    for (int i = 0; i < 5; i++) {
      v.push_back(u.distance_cm());
      sleep_ms(20);
    }
    std::sort(v.begin(), v.end());
    float med = v[v.size() / 2];
    return med >= th_cm;
  }

  void backup_ms(int ms) {
    m.backward(BACK_PWM);
    sleep_ms(ms);
    m.stop();
  }
  void pivotRight_ms(int ms) {
    m.turnRight(TURN_PWM);
    sleep_ms(ms);
    m.stop();
  }
  void pivotLeft_ms(int ms) {
    m.turnLeft(TURN_PWM);
    sleep_ms(ms);
    m.stop();
  }

  // time-based loop; simple reactive avoidance
  void run_for_seconds(float secs) {
    const int step = 200; // ms
    int total_ms = int(secs * 1000.0f);
    int t = 0;
    while (t < total_ms) {
      if (!pathIsClear(STOP_CM)) {
        m.stop();
        std::cout << "[NAV] Obstacle → avoidance\n";
        // try right -> else left -> else back + harder right
        pivotRight_ms(300);
        if (!pathIsClear(CLEAR_CM)) {
          pivotLeft_ms(600);
          if (!pathIsClear(CLEAR_CM)) {
            pivotRight_ms(300);
            backup_ms(300);
            pivotRight_ms(600);
          }
        }
      }
      m.forward(DRIVE_PWM);
      sleep_ms(step);
      m.stop();
      t += step;
    }
  }
};
