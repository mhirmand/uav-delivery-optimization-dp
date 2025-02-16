import matplotlib.pyplot as plt
import numpy as np

def visualize_uav_path(input_file, solution_file, output_image):
    # Read input file
    with open(input_file, 'r') as f:
        lines = f.read().splitlines()
    
    start = list(map(float, lines[0].split()))
    end = list(map(float, lines[1].split()))
    N = int(lines[2])
    waypoints = []
    for line in lines[3:3+N]:
        x, y, p = map(float, line.split())
        waypoints.append((x, y, p))
    
    # Read solution file
    with open(solution_file, 'r') as f:
        sol_lines = f.read().splitlines()
    
    path = []
    for line in sol_lines:
        stripped = line.strip()
        if stripped.isdigit():
            path.append(int(stripped))
    
    # Adjust path to include start (0) and end (N+1) if missing
    if 0 not in path:
        path = [0] + path
    if (N + 1) not in path:
        path.append(N + 1)
    
    # Generate coordinates for the optimal path
    optimal_coords = []
    for idx in path:
        if idx == 0:
            optimal_coords.append((start[0], start[1]))
        elif idx == N + 1:
            optimal_coords.append((end[0], end[1]))
        else:
            wp_idx = idx - 1  # Convert to 0-based index for waypoints list
            x, y, _ = waypoints[wp_idx]
            optimal_coords.append((x, y))
    
    # Create plot
    plt.figure(figsize=(10, 10))
    
    # Plot start and end points
    plt.plot(start[0], start[1], 'go', markersize=12, label='Start', zorder=5)
    plt.text(start[0], start[1] - 3, 'Start', ha='center', va='top', fontsize=12, color='green', zorder=6)
    plt.plot(end[0], end[1], 'ro', markersize=12, label='End', zorder=5)
    plt.text(end[0], end[1] + 3, 'End', ha='center', va='bottom', fontsize=12, color='red', zorder=6)
    
    # Plot all waypoints and annotate skipped ones
    for i, (x, y, p) in enumerate(waypoints):
        problem_idx = i + 1  # Problem's waypoint index (1-based)
        if problem_idx in path:
            plt.plot(x, y, 'bo', markersize=10, zorder=3)
            # Add waypoint index with a small offset
            plt.text(x, y + 3, str(problem_idx), color='blue', ha='center', va='bottom', fontsize=12, zorder=4)
        else:
            plt.plot(x, y, 'o', color='gray', markersize=10, zorder=2)
            # Add penalty in red with a small offset
            plt.text(x, y - 3, f'+{int(p)}', color='red', ha='center', va='top', fontsize=10, zorder=4)
    
    # Draw the optimal path lines with arrows
    x_coords = [p[0] for p in optimal_coords]
    y_coords = [p[1] for p in optimal_coords]
    for i in range(len(x_coords) - 1):
        dx = x_coords[i + 1] - x_coords[i]
        dy = y_coords[i + 1] - y_coords[i]
        plt.arrow(x_coords[i], y_coords[i], dx * 0.9, dy * 0.9, 
                  head_width=2, head_length=3, fc='blue', ec='blue', linestyle='--', linewidth=1, zorder=1)
    
    plt.xlim(0, 100)
    plt.ylim(0, 100)
    plt.xlabel('X (meters)', fontsize=12)
    plt.ylabel('Y (meters)', fontsize=12)
    plt.title('UAV Optimal Delivery Path Visualization', fontsize=14)
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend(loc='upper left', fontsize=10)
    plt.savefig(output_image, bbox_inches='tight')
    plt.close()

# Example usage:
visualize_uav_path('medium1.txt', 'medium1_sol.txt', 'uav_path_visualization.png')