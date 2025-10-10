#pragma once
#include <chrono>
#include <fstream>
#include <string>

class Logger {
public:
  explicit Logger(const std::string &csv_path);
  ~Logger();
  void write(int distance_cm, const std::string &action);

private:
  std::ofstream out_;
  std::chrono::steady_clock::time_point t0_;
};
