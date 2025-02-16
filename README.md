# C++ UAV Delivery Path Optimization

This repository contains a C++ implementation for the optimization of a delivery UAV path.

## Table of Contents

- [Overview](#Problem-Overview)
- [Algorithm and Time Complexity](#Algorithm-and-Time-Complexity)
- [Usage](#Usage)
- [Examples](#Examples)

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
5. **Waypoints not hit Accidentally**: The UAV it is not in danger of hitting a waypoint accidentally too soon by flying over it.

### Objective
Implement a C++ solution to compute the minimal total time and the optimal path for each test case. The solution should be able to solve problems with thousands of waypoints within a reasonable time.

## Algorithm and Time Complexity
The general solution strategy is to use dynamic programming to compute the minimal total time by evaluating all possible paths from the start to each waypoint, storing intermediate results to avoid redundant calculations, and backtracking to reconstruct the optimal path.

- **Without Dynamic Programming (DP)**:  
  A brute-force approach would evaluate all possible subsets of waypoints to visit or skip. For `N` waypoints, there are `2^N` possible subsets, and evaluating each subset takes `O(N)` time. This results in a total complexity of  `O(N * 2^N)`, which is **exponential** and impractical for large `N`.

- **With Dynamic Programming (DP)**:  
  The DP approach, which is implemented in this project, reduces the complexity to `O(N^2)` by avoiding redundant calculations. For each waypoint `i`, it checks all previous waypoints `j < i` to compute the minimal time. This makes the algorithm feasible for large `N`.

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
- **1st Line**: X, Y of the starting point.
- **2nd Line**: Y, Y of the end point.
- **3rd Line**: Integer N, the number of waypoints.
- **Next N Lines**: Each line contains three integers: X, Y, and P, where:
  - (X, Y) are the coordinates of the waypoint.
  - P is the penalty for skipping the waypoint.

### Output Format
For each test case, output:
1. The minimal total time (in seconds) rounded to 3 decimal places.
2. The optimal path as a sequence of visited waypoint indices. Start and end points are not included in the list. It assumed that waypoint indicies range from 1 to N.

## Examples

Several example inputs and their solutions (obtained using the default UAV speed and wait times) are provided in the `examples` folder. These examples span problems of small (a few waypoints), medium (less than ~100 waypoints), and large (over ~100 waypoints) size. 

For the small input #3 `small2.txt`, we have:
```bash
0 0
100 100
3
30 30 90
60 60 80
10 90 10
```
The starting point is `(0,0)` and the end point is `(100, 100)`. There are 3 waypoints to be visited by the UAV. The solution output is:

```bash
Execution Time: 1 ms
Minimum UAV time: 110.711
Optimal waypoint indicies: 
1 
2 
```
which shows that the optimal path includes:
- starting at (0, 0)
- visiting waypoints 1 and 2 (waiting for 10 s there and incure no penalty)
- traveling directly to the end point (100, 100) from waypoint #2 (i.e., skipping waypoint 3 by incuring its penalty of 10 s)

The total time optimal time is `110.711 s` which includes travel time, wait times, and penalties. The python script `plot_results.py` included in this repository can be used to visualize the optimal UAV path:

<div align="center">
<img src="https://github.com/user-attachments/assets/872bb38e-b121-4aad-a7e6-40e49b029e79" width="500" height="500">
</div>

