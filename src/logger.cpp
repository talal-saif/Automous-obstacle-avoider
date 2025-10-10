#include "logger.hpp"

Logger::Logger(const std::string &csv_path) : out_(csv_path) {
  t0_ = std::chrono::steady_clock::now();
  out_ << "t_ms,distance_cm,action\n";
}
Logger::~Logger() { out_.close(); }

void Logger::write(int distance_cm, const std::string &action) {
  auto now = std::chrono::steady_clock::now();
  long ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - t0_).count();
  out_ << ms << "," << distance_cm << "," << action << "\n";
  out_.flush();
}
