#include "delivery_uav.h"
#include <string>
#include <iostream>

/**
 * Config: Structure to hold configurable parameters for the program.
 * - input_path: Path to the input data file.
 * - output_path: Path the output data file.
 * - uavSpeed: Speed of the UAV (default: 2.0 m/s).
 * - waitTime: Wait time at each waypoint (default: 10 s).
 */
struct Config {
  std::string input_path;
	std::string output_path;
	double uav_Speed = 2.0;
	double wait_Time = 10.0;
};

/**
 * parse_arguments: Parses command-line arguments.
 * - Validates input and extracts input/output paths, UAV speed, and wait time.
 * - Throws runtime_error for invalid or insufficient arguments.
 */
Config parse_arguments(int argc, char* argv[]) {
  Config cfg;
  if (argc < 2) {
    throw std::runtime_error("Usage: " + std::string(argv[0]) + " <input_path> <output_path> [uav_speed] [wait_time]");
  }
  cfg.input_path = argv[1];
  cfg.output_path = argv[2];

  if (argc > 3) cfg.uav_Speed = std::stoi(argv[3]);
  if (argc > 4) cfg.wait_Time = std::stoi(argv[4]);
  return cfg;
}

/**
 * main: Entry point for the application.
 * Workflow:
 * 1. Parses program configuration using command-line arguments.
 * 2. Loads input data using the input path.
 * 3. Initializes the UAV and finds the optimal path's time for the given case:
 * 4. Prints out the results in the output file.
 */

int main(int argc, char* argv[]) {

  Config cfg = parse_arguments(argc, argv);

  const std::string input_file = cfg.input_path;
  const std::string output_file = cfg.output_path;

  auto uav = DeliveryUAV(cfg.uav_Speed, cfg.wait_Time);
  return uav.solveCase("input.txt", "output.txt");
}