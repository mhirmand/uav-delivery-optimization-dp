import matplotlib.pyplot as plt
import numpy as np
import sys
import os

def visualize_uav_path(input_file, solution_file, output_image):
    with open(input_file, 'r') as f:
        lines = f.read().splitlines()
    
    start = list(map(float, lines[0].split()))
    end = list(map(float, lines[1].split()))
    N = int(lines[2])
    waypoints = [tuple(map(float, line.split())) for line in lines[3:3+N]]
    
    with open(solution_file, 'r') as f:
        sol_lines = f.read().splitlines()
    
    path = [int(line.strip()) for line in sol_lines if line.strip().isdigit()]
    
    if 0 not in path:
        path.insert(0, 0)
    if (N + 1) not in path:
        path.append(N + 1)
    
    optimal_coords = [(start[0], start[1])] + [(waypoints[idx-1][0], waypoints[idx-1][1]) for idx in path if 1 <= idx <= N] + [(end[0], end[1])]
    
    plt.figure(figsize=(10, 10))
    plt.plot(start[0], start[1], 'go', markersize=16, label='Start', zorder=5)
    plt.text(start[0], start[1] - 3, 'Start', ha='center', va='top', fontsize=14, color='green', fontweight='bold', zorder=6)
    plt.plot(end[0], end[1], 'ro', markersize=16, label='End', zorder=5)
    plt.text(end[0], end[1] + 3, 'End', ha='center', va='bottom', fontsize=14, color='red', fontweight='bold', zorder=6)
    
    for i, (x, y, p) in enumerate(waypoints):
        if (i + 1) in path:
            plt.plot(x, y, 'g*', markersize=16, zorder=3)  # Green asterisk for optimal waypoints
            plt.text(x, y + 3, str(i + 1), color='blue', ha='center', va='bottom', fontsize=14, fontweight='bold', zorder=4)
        else:
            plt.plot(x, y, '*', color='gray', markersize=16, zorder=2)
            plt.text(x + 3, y, f'+{int(p)}', color='red', ha='center', va='center', fontsize=14, fontweight='bold', zorder=4)  # Penalty right on top
            plt.text(x, y - 3, f'{i+1}', color='black', ha='center', va='top', fontsize=14, fontweight='bold', zorder=4)
    
    x_coords, y_coords = zip(*optimal_coords)
    for i in range(len(x_coords) - 1):
        dx = (x_coords[i+1] - x_coords[i])
        dy = (y_coords[i+1] - y_coords[i])
        plt.arrow(x_coords[i] + 0.05 * dx, y_coords[i] + 0.05 * dy, dx * 0.85, dy * 0.85, 
                  head_width=1.5, head_length=2, fc='blue', ec='blue', linestyle='--', linewidth=1, zorder=1)
    
    plt.xlim(start[0], end[0])
    plt.ylim(start[1], end[1])
    plt.xlabel('X (meters)', fontsize=16, fontweight='bold')
    plt.ylabel('Y (meters)', fontsize=16, fontweight='bold')
    plt.title('UAV Optimal Delivery Path Visualization', fontsize=18, fontweight='bold')
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    # plt.legend(loc='lower left', fontsize=14)
    plt.savefig(output_image, bbox_inches='tight')
    
    plt.close()

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python plot_results.py <input_file> <solution_file>")
        sys.exit(1)
    
    input_file = sys.argv[1]
    solution_file = sys.argv[2]
    output_image = os.path.splitext(input_file)[0] + "_path.png"
    
    visualize_uav_path(input_file, solution_file, output_image)
    print(f"Plot saved as {output_image}")
