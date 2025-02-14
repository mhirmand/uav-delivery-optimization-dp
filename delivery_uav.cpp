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
 *                 - Must be >= 0 (0 = no stopping required)
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


/**
 * @brief Solves a single case defined in a given input file and writes solution to output file
 *
 * This function handles the complete workflow for a test case:
 * 1. File I/O initialization and validation
 * 2. Input data parsing and validation
 * 3. Data structure preparation
 * 4. Penalty precomputation
 * 5. Core algorithm execution
 * 6. Result formatting and output
 *
 * @param input_file_name Path to input file containing single test case data
 * @param output_file_name Path to output file for writing results
 * @return int Status code: 0 for success, 1 for errors
 *
 * @throws Does not throw exceptions but writes errors to cerr
 *
 * File Format Requirements:
 * - First line: Number of waypoints (N ≥ 0)
 * - Next 2 lines: Start coordinates and Terminal coordinates
 * - N lines: Waypoint data (X, Y, Penalty)
 */
int DeliveryUAV::solveCase(
  const std::string& input_file_name,
  const std::string& output_file_name)
{
  // ---------------------------
  // File Initialization Section
  // ---------------------------
  std::ifstream input_file(input_file_name);
  if (!input_file.is_open()) {
    std::cerr << "Error opening input file: " << input_file_name << '\n';
    return EXIT_FAILURE;
  }

  std::ofstream output_file(output_file_name);
  if (!output_file.is_open()) {
    std::cerr << "Error opening output file: " << output_file_name << '\n';
    input_file.close();  // Cleanup already opened input file
    return EXIT_FAILURE;
  }

  // -------------------------
  // Input Validation Section
  // -------------------------
  int n = 0;
  input_file >> n;
  if (n < 0) {
    std::cerr << "Invalid input format. Number of waypoints ("
      << n << ") must be non-negative.\n";
    return EXIT_FAILURE;
  }

  // ----------------------------------
  // Waypoint Data Loading Section
  // ----------------------------------
  double start_x, start_y, term_x, term_y;
  input_file >> start_x >> start_y >> term_x >> term_y;

  // Initialize waypoints with start and terminal points
  std::vector<WayPoint> waypoints;
  waypoints.reserve(n + 2);  // Preallocate for N + start + terminal
  waypoints.emplace_back(start_x, start_y, 0.0);  // Start point (index 0)

  // Load N waypoints between start and terminal
  for (int i = 0; i < n; ++i) {
    double x, y, p;
    input_file >> x >> y >> p;
    waypoints.emplace_back(x, y, p);  // Main waypoints (indices 1..N)
  }
  waypoints.emplace_back(term_x, term_y, 0.0);  // Terminal point (index N+1)

  // -------------------------------
  // Penalty Precomputation Section
  // -------------------------------
  std::vector<double> prefix(n + 2, 0.0);  // prefix[0] = 0 (start point)
  for (int i = 1; i <= n; ++i) {
    prefix[i] = prefix[i - 1] + waypoints[i].penalty;
  }
  prefix[n + 1] = prefix[n];  // Terminal inherits previous sum (no penalty)

  // --------------------------
  // Core Algorithm Execution
  // --------------------------
  std::vector<int> optimal_path;
  const double result = DeliveryUAV::solve(waypoints, prefix, optimal_path);

  // ----------------------
  // Result Output Section
  // ----------------------
  output_file << "Minimum UAV time: " << std::fixed << std::setprecision(3) << result << '\n';
  output_file << "Optimal waypoint indicies: ";
  for (int idx : optimal_path) {
    output_file << idx << " ";
  }
  output_file << "\n";

  // -------------------
  // Resource Cleanup
  // -------------------
  input_file.close();
  output_file.close();

  return EXIT_SUCCESS;
}


/**
 * @brief Computes the minimal time required for the UAV to complete the course
 *        using dynamic programming with penalty optimization.
 *
 * This function uses a bottom-up dynamic programming approach to determine
 * the optimal path that minimizes total time (travel + penalties + wait times).
 * The algorithm considers all possible previous waypoints when calculating
 * the minimum time to reach each subsequent waypoint.
 *
 * Time Complexity: O(N^2), where N is the number of waypoints
 *
 * @param waypoints Vector containing all waypoints in order:
 *                  [start, wp1, wp2..., terminal]
 * @param prefix    Prefix sum array where prefix[i] represents the sum of
 *                  penalties from waypoints[1] to waypoints[i]
 * @param path      Output vector storing indices of visited (optimal) waypoints in order
 * @return double   Minimal total time in seconds to complete the course,
 *                  rounded to 3 decimal places in the output
 */
double DeliveryUAV::solve(
  const std::vector<WayPoint>& waypoints,
  const std::vector<double>& prefix,
  std::vector<int>& path)
{
  // Total points includes all waypoints except terminal in initial calculation
  const int total_points = (int)waypoints.size() - 1;  // waypoints.size() = N + 2 (start + N + terminal)

  // DP array where dp[i] represents minimum time to reach waypoint[i]
  std::vector<double> dp(total_points + 1, std::numeric_limits<double>::infinity());
  dp[0] = 0.0;  // Base case: start point requires no initial time

  // vector to track best previous points for path reconstruction
  std::vector<int> prev_waypoint(total_points + 1, -1);

  // Compute optimal path for each subsequent waypoint
  for (int i = 1; i <= total_points; ++i) {
    double min_time = std::numeric_limits<double>::max();

    int bestPrev = -1;
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
      // min_time = std::min(min_time, time_candidate);
      if (time_candidate < min_time) {
        min_time = time_candidate;
        bestPrev = j;
      }
    }

    // Add mandatory wait time at current waypoint (including terminal)
    dp[i] = min_time + wait_time_;
    prev_waypoint[i] = bestPrev;
  }

  // Reconstruct path
  path.clear();
  int current = total_points; // Start from terminal
  while (current != 0) {
    path.push_back(current);
    current = prev_waypoint[current];
  }
  path.push_back(0); // Include start point
  std::reverse(path.begin(), path.end());

  // Final result is the minimal time to reach terminal point (last element)
  return dp.back();
}