# C++ UAV Delivery Path Optimization

This repository contains a C++ implementation for the optimization of a delivery UAV path.

## Table of Contents

- [Overview](#overview)
- [Dataset](#dataset)
- [Architecture](#architecture)
- [Build Instructions](#build-instructions)
- [Command-Line Arguments](#command-line-arguments)
- [Results](#results)

## Problem Overview
A delivery UAV must navigate a large area, visiting ordered waypoints to deliver goods. The UAV starts at a given starting point (e.g., (0, 0)) and must end at a given end point (e.g., (100, 100)). Each waypoint has a penalty for being skipped to reflect 
the time needed for a human to handle the work later. The UAV must stop for a certain time at every visited waypoint to process the delivery. It might be advantageous to skip some waypoints and incur their penalty, rather than actually 
manoeuvring to them. Given a description of a course, the goal is to determine UAV's best possible path with the lowest time. The time spent includes:
- Travel time between waypoints
- Wait time at visited waypoints
- Penalties for skipped waypoints

### Key Constraints
1. **Waypoint Order**: Waypoints must be visited in sequential order. Skipping a waypoint incurs its penalty.
2. **No Backtracking**: Once a waypoint is skipped, it cannot be revisited.
3. **Straight-Line Movement**: The UAV moves in straight lines between waypoints and can turn instantly during stops.
4. **Unique Waypoints**: No two waypoints share the same coordinates.
5. **Waypoints not hit Accidentally**:The UAV it is not in danger of hitting a waypoint accidentally too soon by flying over it.

### Objective
Implement a C++ solution to compute the minimal total time and the optimal path for each test case. The solution should be able to solve problems with thousands of waypoints within a reasonable time.

## Algorithm and Time Complexity Analysis
The general solution strategy is to use dynamic programming to compute the minimal total time by evaluating all possible paths from the start to each waypoint, storing intermediate results to avoid redundant calculations, and backtracking to reconstruct the optimal path.

- **Without Dynamic Programming (DP)**:  
  A brute-force approach would evaluate all possible subsets of waypoints to visit or skip. For \( N \) waypoints, there are \( 2^N \) possible subsets, and evaluating each subset takes \( O(N) \) time. This results in a total complexity of \( O(N \times 2^N) \), which is **exponential** and impractical for \( N \geq 20 \).

- **With Dynamic Programming (DP)**:  
  The DP approach, which is implemented in this project, reduces the complexity to \( O(N^2) \) by avoiding redundant calculations. For each waypoint \( i \), it checks all previous waypoints \( j < i \) to compute the minimal time. This makes the algorithm feasible for \( N \leq 1000 \).

## Usage

### Build Instructions

The source .cpp and header .h may be used to build an executable without need to any external dependencies. 

### Command-Line Arguments

The 1st and 2nd arguments are the `path\to\input` and `path\to\output` in the run directory and are mandatory. The next two arguments listed below are optional. If not provided, the default values will be used. 
- `uav_speed`: speed of the UAV (default: 2 m/s)
- `wait_time`: wait time at each waypoint (default: 10 seconds)

Example: Run with default UAV speed and waipoint wait time:

```bash
./deliveryUAV /path/to/input /path/to/output 
```
Run with `uav_speed` = 20 and `wait_time` = 3
```bash
./deliveryUAV /path/to/input /path/to/output 20 3
```

### Input File Format
The input is a txt file containing a set of waypoint coordinates and their penalties in along with the coordinates of the start and end of the course. It must be in the following format:  
- **1st Line**: Integer N, the number of waypoints.
- **2nd Line**: X, Y of the starting point.
- **3rd Line**: Y, Y of the end point.
- **Next N Lines**: Each line contains three integers: X, Y, and P, where:
  - (X, Y) are the coordinates of the waypoint.
  - P is the penalty for skipping the waypoint.

### Output Format
For each test case, output:
1. The minimal total time (in seconds) rounded to 3 decimal places.
2. The optimal path as a sequence of visited waypoint indices (starting from 0 for the origin and ending at N+1 for the terminal point).

## Example Inputs and Solutions

Several example inputs and their solutions are provided in the `examples` folder. These examples span problems of small (a few waypoints), medium (less than ~100 waypoints), and large (over ~100 waypoints) size. 



