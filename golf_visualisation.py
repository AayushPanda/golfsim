import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import json
import subprocess
import argparse
import os
import sys
from matplotlib.patches import Circle

def run_simulation(friction=0.3, dt=0.05, max_time=15, 
                  speed_min=5, speed_max=30, speed_count=15,
                  angle_min=-45, angle_max=45, angle_count=15):
    """Run the C++ simulation with the given parameters."""
    # Build command
    cmd = [
        "./golf_simulation",
        "--friction", str(friction),
        "--dt", str(dt),
        "--max-time", str(max_time),
        "--speed-range", str(speed_min), str(speed_max),
        "--speed-count", str(speed_count),
        "--angle-range", str(angle_min), str(angle_max),
        "--angle-count", str(angle_count),
        "--output", "simulation_results.json"
    ]
    
    print("Running simulation with command:", " ".join(cmd))
    
    # Run the simulation
    try:
        subprocess.run(cmd, check=True)
        print("Simulation completed successfully")
    except subprocess.CalledProcessError as e:
        print(f"Error running simulation: {e}")
        sys.exit(1)
    
    # Load results
    try:
        with open("simulation_results.json", "r") as f:
            data = json.load(f)
        print("Loaded simulation results")
        return data
    except Exception as e:
        print(f"Error loading simulation results: {e}")
        sys.exit(1)

def create_visualization(simulation_data):
    """Create a 3D animation and plot of the simulation results."""
    # Extract data
    terrain_data = simulation_data["terrain"]
    trajectories_data = simulation_data["trajectories"]
    results_data = simulation_data["results"]
    course_info = simulation_data["course_info"]
    
    # Create meshgrid from terrain data
    X, Y = np.meshgrid(terrain_data["X"], terrain_data["Y"])
    Z = np.array(terrain_data["Z"])
    
    # Course dimensions
    course_width = course_info["width"]
    course_height = course_info["height"]
    
    # Start and hole positions
    start_x = course_info["start_x"]
    start_y = course_info["start_y"]
    hole_x = course_info["hole_x"]
    hole_y = course_info["hole_y"]
    hole_radius = course_info["hole_radius"]
    
    # Create figure with two subplots
    fig = plt.figure(figsize=(18, 9))
    
    # 3D golf course plot
    ax1 = fig.add_subplot(121, projection='3d')
    
    # Plot terrain as surface
    terrain = ax1.plot_surface(X, Y, Z, cmap=cm.terrain, alpha=0.8)
    
    # Set up the golf course plot
    ax1.set_xlim(0, course_width)
    ax1.set_ylim(0, course_height)
    z_min = np.min(Z) - 1
    z_max = np.max(Z) + 3
    ax1.set_zlim(z_min, z_max)
    ax1.set_title('Golf Ball Trajectories')
    ax1.set_xlabel('X Distance')
    ax1.set_ylabel('Y Distance')
    ax1.set_zlabel('Height')
    
    # Plot hole position
    hole_z = Z[int(hole_y * len(Z) / course_height)][int(hole_x * len(Z[0]) / course_width)]
    ax1.scatter([hole_x], [hole_y], [hole_z], color='black', s=100, marker='o')
    
    # Plot starting position
    start_z = Z[int(start_y * len(Z) / course_height)][int(start_x * len(Z[0]) / course_width)]
    ax1.scatter([start_x], [start_y], [start_z], color='red', s=100, marker='o')
    
    # Prepare lines for each trajectory
    lines = []
    for key, trajectory in trajectories_data.items():
        result = results_data[key]
        color = 'green' if result == 'hole' else 'red'
        line, = ax1.plot([], [], [], color=color, alpha=0.5)
        lines.append((line, np.array(trajectory)))
    
    # Create scatter plot of results (2D)
    ax2 = fig.add_subplot(122)
    
    # Organize results for visualization
    success_points = []
    failed_points = []
    
    for key, result in results_data.items():
        # Parse speed and angle from key (format: "speed_angle")
        speed, angle = map(float, key.split("_"))
        if result == "hole":
            success_points.append((speed, angle))
        else:
            failed_points.append((speed, angle))
    
    if success_points:
        success_speeds, success_angles = zip(*success_points)
        ax2.scatter(success_speeds, success_angles, color='green', label='In Hole', s=80)
    
    if failed_points:
        failed_speeds, failed_angles = zip(*failed_points)
        ax2.scatter(failed_speeds, failed_angles, color='red', label='Missed', alpha=0.3, s=50)
        
    ax2.set_title('Speed vs Angle for Successful Shots')
    ax2.set_xlabel('Speed (m/s)')
    ax2.set_ylabel('Angle (degrees)')
    ax2.legend()
    ax2.grid(True)
    
    # Animation function
    def init():
        for line, _ in lines:
            line.set_data([], [])
            line.set_3d_properties([])
        return [line for line, _ in lines]
    
    def animate(frame):
        for line, trajectory in lines:
            # Get path up to current frame
            if frame < len(trajectory):
                x_vals = trajectory[:frame+1, 0]
                y_vals = trajectory[:frame+1, 1]
                z_vals = trajectory[:frame+1, 2]
                line.set_data(x_vals, y_vals)
                line.set_3d_properties(z_vals)
        return [line for line, _ in lines]
    
    # Determine the maximum length of all trajectories
    max_frames = max(len(trajectory) for _, trajectory in lines)
    
    # Create animation
    anim = FuncAnimation(fig, animate, frames=min(max_frames, 200), 
                         init_func=init, interval=50, blit=True)
    
    plt.tight_layout()
    return fig, anim

def main():
    parser = argparse.ArgumentParser(description='Golf simulation visualization')
    parser.add_argument('--friction', type=float, default=0.3, help='Friction coefficient')
    parser.add_argument('--dt', type=float, default=0.05, help='Time step for simulation')
    parser.add_argument('--max-time', type=float, default=15, help='Maximum simulation time')
    parser.add_argument('--speed-min', type=float, default=5, help='Minimum speed')
    parser.add_argument('--speed-max', type=float, default=30, help='Maximum speed')
    parser.add_argument('--speed-count', type=int, default=15, help='Number of speeds to test')
    parser.add_argument('--angle-min', type=float, default=-45, help='Minimum angle')
    parser.add_argument('--angle-max', type=float, default=45, help='Maximum angle')
    parser.add_argument('--angle-count', type=int, default=15, help='Number of angles to test')
    parser.add_argument('--skip-simulation', action='store_true', help='Skip simulation and use existing JSON results')
    parser.add_argument('--output', default='golf_simulation_3d.mp4', help='Output file for animation')
    
    args = parser.parse_args()
    
    # Run simulation or load existing results
    if not args.skip_simulation:
        simulation_data = run_simulation(
            friction=args.friction,
            dt=args.dt,
            max_time=args.max_time,
            speed_min=args.speed_min,
            speed_max=args.speed_max,
            speed_count=args.speed_count,
            angle_min=args.angle_min,
            angle_max=args.angle_max,
            angle_count=args.angle_count
        )
    else:
        try:
            with open("simulation_results.json", "r") as f:
                simulation_data = json.load(f)
            print("Loaded existing simulation results")
        except Exception as e:
            print(f"Error loading existing simulation results: {e}")
            sys.exit(1)
    
    # Create visualization
    print("Creating visualization...")
    fig, anim = create_visualization(simulation_data)
    
    # Save animation
    print(f"Saving animation to {args.output}...")
    try:
        anim.save(args.output, writer='ffmpeg', fps=30)
        print(f"Animation saved as '{args.output}'")
    except Exception as e:
        print(f"Could not save animation: {e}")
    
    plt.show()
    print("Done!")

if __name__ == "__main__":
    main()