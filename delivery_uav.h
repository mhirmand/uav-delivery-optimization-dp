#pragma once

#include <string>
#include <vector>
#include <fstream>

struct WayPoint {
  double x, y, penalty;
  WayPoint(double x_ = 0.0, double y_ = 0.0, double p_ = 0.0);
};

class DeliveryUAV {
public:
  DeliveryUAV(double speed, double wait_time);
  int solveCase(const std::string& input_file_name, const std::string& output_file_name);

private:
  double uav_speed_;
  double wait_time_;
  double solve(const std::vector<WayPoint>& waypoints, const std::vector<double>& prefix, std::vector<int>& path);

};