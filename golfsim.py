import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import Circle
import matplotlib.colors as mcolors

from scipy.interpolate import RegularGridInterpolator
from scipy.ndimage import gaussian_filter

class GolfSimulation3D:
    def __init__(self, friction=0.1, dt=0.05, max_time=30):
        """
        Initialize the 3D golf simulation with terrain elevation.
        
        Parameters:
        - friction: Coefficient of friction
        - dt: Time step for simulation
        - max_time: Maximum simulation time
        """
        # course parameters
        self.course_width = 100
        self.course_height = 50
        
        # ball starting position
        self.start_x = 10
        self.start_y = self.course_height / 2
        
        # hole position
        self.hole_x = 80
        self.hole_y = self.course_height / 2
        self.hole_radius = 1.0
        
        # physics parameters
        self.friction = friction
        self.dt = dt
        self.max_time = max_time
        self.gravity = 9.8  # m/s^2
        
        # launch parameters to test
        self.speeds = np.linspace(5, 30, 15)
        self.angles = np.linspace(-45, 45, 15)
        
        # results storage
        self.results = {}
        self.trajectories = {}
        
        # generate terrain
        self.create_terrain()
        
    def create_terrain(self):
        """Create terrain with hills, valleys, and slopes."""
        # grid for terrain
        x = np.linspace(0, self.course_width, 100)
        y = np.linspace(0, self.course_height, 50)
        self.X, self.Y = np.meshgrid(x, y)
        
        # elevation function with multiple terrain features
        self.Z = (
            # base elevation
            np.zeros_like(self.X) +
            
            # hill
            3 * np.exp(-0.01 * ((self.X - 40)**2 + (self.Y - 15)**2)) +
            
            # ridge
            2 * np.exp(-0.03 * (self.Y - 35)**2) +
            
            # valley
            -1.5 * np.exp(-0.02 * ((self.X - 60)**2 + (self.Y - 25)**2)) +
            
            # add some random small bumps
            0.2 * np.sin(0.2 * self.X) * np.cos(0.3 * self.Y)
        )
        
        # make starting point and hole are relatively flat
        start_mask = ((self.X - self.start_x)**2 + (self.Y - self.start_y)**2 < 5**2)
        self.Z[start_mask] = 0
        
        hole_mask = ((self.X - self.hole_x)**2 + (self.Y - self.hole_y)**2 < 3**2)
        self.Z[hole_mask] = 0
        
        # interpolation functions for getting height at any (x,y)
        from scipy.interpolate import RegularGridInterpolator
        x_points = np.linspace(0, self.course_width, 100)
        y_points = np.linspace(0, self.course_height, 50)
        self.elevation_func = RegularGridInterpolator((y_points, x_points), self.Z)
        
        # gradient functions for calculating slopes
        from scipy.ndimage import gaussian_filter
        Z_smooth = gaussian_filter(self.Z, sigma=1)
        
        gradient_y, gradient_x = np.gradient(Z_smooth)
        gradient_y /= y_points[1] - y_points[0]
        gradient_x /= x_points[1] - x_points[0]
        
        self.gradient_x_func = RegularGridInterpolator((y_points, x_points), gradient_x)
        self.gradient_y_func = RegularGridInterpolator((y_points, x_points), gradient_y)
    
    def get_elevation(self, x, y):
        """Get the elevation at a specific point."""
        if 0 <= x <= self.course_width and 0 <= y <= self.course_height:
            return float(self.elevation_func(np.array([y, x])))
        return 0
    
    def get_gradient(self, x, y):
        """Get the terrain gradient (slope) at a specific point."""
        if 0 <= x <= self.course_width and 0 <= y <= self.course_height:
            grad_x = float(self.gradient_x_func(np.array([y, x])))
            grad_y = float(self.gradient_y_func(np.array([y, x])))
            return grad_x, grad_y
        return 0, 0
        
    def run_simulation(self):
        """Run simulation for all speed/angle combinations."""
        for speed in self.speeds:
            for angle in self.angles:
                key = (speed, angle)
                
                # convert angle to radians
                angle_rad = np.deg2rad(angle)
                
                # initial velocity components (horizontal only)
                vx = speed * np.cos(angle_rad)
                vy = speed * np.sin(angle_rad)
                vz = 0  # no initial vertical velocity
                
                # simulate this ball's trajectory
                trajectory, result = self.simulate_ball(vx, vy, vz)
                
                # store results
                self.results[key] = result
                self.trajectories[key] = trajectory
    
    def simulate_ball(self, vx, vy, vz):
        """
        Simulate the trajectory of a single ball in 3D space with terrain.
        
        Parameters:
        - vx, vy, vz: Initial velocity components
        
        Returns:
        - trajectory: List of (x, y, z) positions
        - result: "hole" if ball lands in hole, "out" if out of bounds, "stopped" if stops before hole
        """
        # current position
        x, y = self.start_x, self.start_y
        z = self.get_elevation(x, y)
        
        # store trajectory
        trajectory = [(x, y, z)]
        
        # run simulation until max time
        for _ in np.arange(0, self.max_time, self.dt):
            # get terrain gradient at current position
            grad_x, grad_y = self.get_gradient(x, y)
            
            # gravity components due to slope
            g_x = self.gravity * grad_x
            g_y = self.gravity * grad_y
            g_z = -self.gravity  # vertical gravity always points down
            
            # apply terrain forces to velocity
            vx += g_x * self.dt
            vy += g_y * self.dt
            
            # in 3D, we use vz for vertical movement but reset it when on the ground
            vz += g_z * self.dt
            
            # update position
            x += vx * self.dt
            y += vy * self.dt
            z += vz * self.dt
            
            # check if ball hits the ground
            terrain_z = self.get_elevation(x, y)
            if z < terrain_z:
                # ball hits the ground
                z = terrain_z
                
                # simple bounce model (lose some energy)
                vz = -vz * 0.5
                
                # apply friction to horizontal velocity on contact
                speed_h = np.sqrt(vx**2 + vy**2)
                if speed_h > 0:
                    friction_decel = self.friction
                    vx -= friction_decel * vx / speed_h
                    vy -= friction_decel * vy / speed_h
            
            # check if ball is moving very slowly (effectively stopped)
            if np.sqrt(vx**2 + vy**2 + vz**2) < 0.1:
                trajectory.append((x, y, z))
                
                # check if the stopped ball is in the hole
                if np.sqrt((x - self.hole_x)**2 + (y - self.hole_y)**2) < self.hole_radius:
                    return trajectory, "hole"
                else:
                    return trajectory, "stopped"
                
            # check if ball is in the hole
            if np.sqrt((x - self.hole_x)**2 + (y - self.hole_y)**2) < self.hole_radius:
                trajectory.append((self.hole_x, self.hole_y, self.get_elevation(self.hole_x, self.hole_y)))
                return trajectory, "hole"
                
            # check if ball is out of bounds
            if x < 0 or x > self.course_width or y < 0 or y > self.course_height:
                trajectory.append((x, y, z))
                return trajectory, "out"
                
            # add current position to trajectory
            trajectory.append((x, y, z))
        
        # if we got here, the ball was still moving after max_time
        return trajectory, "timeout"
    
    def create_visualization(self):
        """Create a 3D animation and plot of the simulation results."""
        # create figure with two subplots
        fig = plt.figure(figsize=(18, 9))
        
        # 3D golf course plot
        ax1 = fig.add_subplot(121, projection='3d')
        
        # plot terrain as surface
        terrain = ax1.plot_surface(self.X, self.Y, self.Z, cmap=cm.terrain, alpha=0.8)
        
        # set up the golf course plot
        ax1.set_xlim(0, self.course_width)
        ax1.set_ylim(0, self.course_height)
        ax1.set_zlim(-2, 5)
        ax1.set_title('3D Golf Ball Trajectories')
        ax1.set_xlabel('X Distance (m)')
        ax1.set_ylabel('Y Distance (m)')
        ax1.set_zlabel('Height (m)')
        
        # plot hole position
        hole_z = self.get_elevation(self.hole_x, self.hole_y)
        ax1.scatter([self.hole_x], [self.hole_y], [hole_z], color='black', s=100, marker='o')
        
        # plot starting position
        start_z = self.get_elevation(self.start_x, self.start_y)
        ax1.scatter([self.start_x], [self.start_y], [start_z], color='red', s=100, marker='o')
        
        # prepare lines for each trajectory
        lines = []
        for key, trajectory in self.trajectories.items():
            result = self.results[key]
            color = 'green' if result == 'hole' else 'red'
            line, = ax1.plot([], [], [], color=color, alpha=0.5)
            lines.append((line, trajectory))
        
        # create scatter plot of results (2D)
        ax2 = fig.add_subplot(122)
        
        # organize results for visualization
        success_points = []
        failed_points = []
        
        for speed in self.speeds:
            for angle in self.angles:
                if self.results[(speed, angle)] == "hole":
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
        
        # animation function
        def init():
            for line, _ in lines:
                line.set_data([], [])
                line.set_3d_properties([])
            return [line for line, _ in lines]
        
        def animate(frame):
            for line, trajectory in lines:
                # get path up to current frame
                if frame < len(trajectory):
                    x_vals, y_vals, z_vals = zip(*trajectory[:frame+1])
                    line.set_data(x_vals, y_vals)
                    line.set_3d_properties(z_vals)
            return [line for line, _ in lines]
        
        # determine the maximum length of all trajectories
        max_frames = max(len(trajectory) for trajectory in self.trajectories.values())
        
        # create animation
        anim = FuncAnimation(fig, animate, frames=min(max_frames, 200), 
                             init_func=init, interval=50, blit=True)
        
        plt.tight_layout()
        return fig, anim
    
if __name__ == "__main__":
    
    # create and run simulation
    print("Init sim")
    simulation = GolfSimulation3D(friction=0.3, dt=0.05, max_time=15)
    
    print("Running sim...")
    simulation.run_simulation()
    
    # create visualization
    print("Creating visualization...")
    fig, anim = simulation.create_visualization()
    
    # save animation
    print("Saving animation...")
    try:
        anim.save('golf_simulation_3d.mp4', writer='ffmpeg', fps=30)
        print("Animation saved as 'golf_simulation_3d.mp4'")
    except Exception as e:
        print(f"Could not save animation: {e}")
    
    plt.show()
    
    print("Done!")