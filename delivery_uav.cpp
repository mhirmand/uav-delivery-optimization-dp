#include "delivery_uav.h"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>

WayPoint::WayPoint(double x_, double y_, double p_) : x(x_), y(y_), penalty(p_) {}

/**
 * @brief Constructs a DeliveryUAV instance with specified movement parameters
 *
 * Initializes the UAV's operational characteristics that remain constant
 * across all test cases. These parameters fundamentally affect all time calculations
 * in the path optimization algorithm.
 *
 * @param speed     Cruising speed of the UAV in meters/second (m/s)
 *                 - Must be > 0 (undefined behavior for zero/negative values)
 *                 - Typical commercial drone speeds: 10-25 m/s
 *                 - Affects all travel time calculations between waypoints
 *
 * @param wait_time Mandatory stop duration at each waypoint in seconds
 *                 - Must be ≥ 0 (0 = no stopping required)
 *                 - Includes package delivery and system checks
 *                 - Applied to ALL visited waypoints including terminal
 *
 * Example usage:
 * @code
 * DeliveryUAV heavyPayloadDrone(5.0, 15.0);  // Slow speed, long setup time
 * DeliveryUAV racingDrone(25.0, 2.5);        // Fast speed, quick stops
 * @endcode
 */
DeliveryUAV::DeliveryUAV(double speed, double wait_time)
  : uav_speed_(speed),      // Initialized first - critical for calculations
  wait_time_(wait_time)   // Directly affects all waypoint time costs
{
  // Note: While not explicitly validated here, these values should be:
  // - speed > 0 (prevents division-by-zero in time calculations)
  // - wait_time >= 0 (negative wait times are non-physical)
  // Caller responsibility to ensure valid parameters
}
int DeliveryUAV::solveCase(
  const std::string& input_file_name, 
  const std::string& output_file_name) 
{

	// open input and output files
  std::ifstream input_file(input_file_name);
  if (!input_file.is_open()) {
    std::cerr << "Error opening input file: " << input_file_name << '\n';
    return 1;
  }

  std::ofstream output_file(output_file_name);
  if (!output_file.is_open()) {
    std::cerr << "Error opening output file: " << output_file_name << '\n';
    input_file.close();
    return 1;
  }

  // get number of waypoints
  int n = 0;
  input_file >> n;
	if (n < 0) {
		std::cerr << "Invalid test case format. Expected a positive number of waypoints.\n";
		return 1;
	}

	// get start and terminal point coordinates
  double start_x, start_y, term_x, term_y;
  input_file >> start_x >> start_y >> term_x >> term_y;

	// initiate waypoints vector
  std::vector<WayPoint> waypoints;
  waypoints.reserve(n + 2);
  waypoints.emplace_back(start_x, start_y, 0.0); // Start point

  for (int i = 0; i < n; ++i) {
    double x, y, p;
    input_file >> x >> y >> p;
    waypoints.emplace_back(x, y, p);
  }
  waypoints.emplace_back(term_x, term_y, 0.0); // Terminal point

  // Precompute prefix sums of penalties
  std::vector<double> prefix(n + 2, 0.0);
  for (int i = 1; i <= n; ++i) {
    prefix[i] = prefix[i - 1] + waypoints[i].penalty;
  }
  prefix[n + 1] = prefix[n]; // Terminal has no penalty

  const double result = DeliveryUAV::solve(waypoints, prefix);
  output_file << std::fixed << std::setprecision(3) << result << '\n';

	// close input and output files
  input_file.close();
  output_file.close();

  return 0;
}

double DeliveryUAV::solve(
  const std::vector<WayPoint>& waypoints,
  const std::vector<double>& prefix)
{
  // Total points includes all waypoints except terminal in initial calculation
  const int total_points = waypoints.size() - 1;  // waypoints.size() = N + 2 (start + N + terminal)

  // DP array where dp[i] represents minimum time to reach waypoint[i]
  std::vector<double> dp(total_points + 1, std::numeric_limits<double>::infinity());
  dp[0] = 0.0;  // Base case: start point requires no initial time

  // Compute optimal path for each subsequent waypoint
  for (int i = 1; i <= total_points; ++i) {
    double min_time = std::numeric_limits<double>::max();

    // Consider all possible previous waypoints j that could precede i
    for (int j = 0; j < i; ++j) {
      // Calculate straight-line distance between waypoints j and i
      const double distance = std::hypot(waypoints[i].x - waypoints[j].x,
        waypoints[i].y - waypoints[j].y);

      // Sum of penalties for skipped waypoints between j and i (wp[j+1] to wp[i-1])
      const double sum_pen = prefix[i - 1] - prefix[j];

      // Calculate candidate time: time to reach j + travel time + penalties
      const double time_candidate = dp[j] + (distance / uav_speed_) + sum_pen;

      // Track minimum time across all possible j positions
      min_time = std::min(min_time, time_candidate);
    }

    // Add mandatory wait time at current waypoint (including terminal)
    dp[i] = min_time + wait_time_;
  }

  // Final result is the minimal time to reach terminal point (last element)
  return dp.back();
}