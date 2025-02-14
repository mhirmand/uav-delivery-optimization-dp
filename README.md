### UAV Delivery Path Optimization Problem

#### Overview
A delivery UAV must navigate a large area, visiting ordered waypoints to deliver goods. The UAV starts at a given starting point (e.g., (0, 0)) and must end at a given end point (e.g., (100, 100)). Each waypoint has a penalty for being skipped, and the UAV must stop for 10 seconds at every visited waypoint. The goal is to minimize the total time, which includes:
- Travel time between waypoints (UAV speed: 2 m/s)
- Wait time at visited waypoints
- Penalties for skipped waypoints

#### Key Constraints
1. **Waypoint Order**: Waypoints must be visited in sequential order. Skipping a waypoint incurs its penalty.
2. **No Backtracking**: Once a waypoint is skipped, it cannot be revisited.
3. **Straight-Line Movement**: The UAV moves in straight lines between waypoints and can turn instantly during stops.
4. **Unique Waypoints**: No two waypoints share the same coordinates.

#### Input Format
The input is a txt file containing the waypoint coordinates and their penalties in the following format.  
- **1st Line**: Integer N, the number of waypoints.
- **2nd Line**: X, Y of the starting point.
- **3rd Line**: Y, Y of the end point.
- **Next N Lines**: Each line contains three integers: X, Y, and P, where:
  - (X, Y) are the coordinates of the waypoint.
  - P is the penalty for skipping the waypoint.

#### Output Format
For each test case, output:
1. The minimal total time (in seconds) rounded to 3 decimal places.
2. The optimal path as a sequence of visited waypoint indices (starting from 0 for the origin and ending at N+1 for the terminal point).
