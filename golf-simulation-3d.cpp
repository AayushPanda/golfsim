#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class GolfSimulation3D {
private:

    // Course parameters
    double course_width = 100;
    double course_height = 100;
    
    // Ball starting position
    double start_x = 10;
    double start_y = 25;
    
    // Hole position
    double hole_x = 80;
    double hole_y = 40;
    double hole_radius = 1.0;
    
    // Physics parameters
    const double k = 0.5; // coeff of restitution for bounce
    double friction;
    double dt;
    double max_time;
    double gravity = 9.8; // m/s^2
    
    // Terrain data
    std::vector<std::vector<double>> terrain_elevation;
    std::vector<std::vector<double>> gradient_x;
    std::vector<std::vector<double>> gradient_y;
    int terrain_resolution_x = 100;
    int terrain_resolution_y = 50;
    
    // Results
    std::vector<std::pair<double, double>> speeds_angles_pairs;
    json results;

public:
    GolfSimulation3D(double frict, double time_step, double max_t) 
        : friction(frict), dt(time_step), max_time(max_t) {
        createTerrain();
    }

    void setSpeedsAndAngles(const std::vector<double>& speeds, const std::vector<double>& angles) {
        for (const auto& speed : speeds) {
            for (const auto& angle : angles) {
                speeds_angles_pairs.push_back(std::make_pair(speed, angle));
            }
        }
    }
    
    void createTerrain() {
        // Initialize terrain grid
        terrain_elevation.resize(terrain_resolution_y, std::vector<double>(terrain_resolution_x, 0.0));
        gradient_x.resize(terrain_resolution_y, std::vector<double>(terrain_resolution_x, 0.0));
        gradient_y.resize(terrain_resolution_y, std::vector<double>(terrain_resolution_x, 0.0));
        
        // Fill terrain with elevation data
        for (int y = 0; y < terrain_resolution_y; y++) {
            for (int x = 0; x < terrain_resolution_x; x++) {
                double x_pos = x * course_width / (terrain_resolution_x - 1);
                double y_pos = y * course_height / (terrain_resolution_y - 1);
                
                // Base elevation with features
                double elevation = 
                    // Hill
                    3 * exp(-0.01 * (pow(x_pos - 40, 2) + pow(y_pos - 15, 2))) +
                    
                    // Ridge
                    1.5 * exp(-0.03 * pow(y_pos - 35, 2)) +
                    
                    // Valley
                    -1.5 * exp(-0.02 * (pow(x_pos - 60, 2) + pow(y_pos - 25, 2))) +
                    
                    // Small bumps
                    0.2 * sin(0.2 * x_pos) * cos(0.3 * y_pos);
                
                terrain_elevation[y][x] = elevation;
            }
        }
        
        // Make starting point and hole area flat
        for (int y = 0; y < terrain_resolution_y; y++) {
            for (int x = 0; x < terrain_resolution_x; x++) {
                double x_pos = x * course_width / (terrain_resolution_x - 1);
                double y_pos = y * course_height / (terrain_resolution_y - 1);
                
                // Flatten start area
                if (pow(x_pos - start_x, 2) + pow(y_pos - start_y, 2) < 25) { // 5^2
                    terrain_elevation[y][x] = 0;
                }
                
                // Flatten hole area
                if (pow(x_pos - hole_x, 2) + pow(y_pos - hole_y, 2) < 9) { // 3^2
                    terrain_elevation[y][x] = 0;
                }
            }
        }
        
        // Calculate gradients using central differences
        for (int y = 1; y < terrain_resolution_y - 1; y++) {
            for (int x = 1; x < terrain_resolution_x - 1; x++) {
                double x_step = course_width / (terrain_resolution_x - 1);
                double y_step = course_height / (terrain_resolution_y - 1);
                
                gradient_x[y][x] = (terrain_elevation[y][x+1] - terrain_elevation[y][x-1]) / (2 * x_step);
                gradient_y[y][x] = (terrain_elevation[y+1][x] - terrain_elevation[y-1][x]) / (2 * y_step);
            }
        }
    }
    
    double getElevation(double x, double y) {
        if (x < 0 || x > course_width || y < 0 || y > course_height) {
            return 0;
        }
        
        // Convert x,y to grid indices
        double x_idx = x * (terrain_resolution_x - 1) / course_width;
        double y_idx = y * (terrain_resolution_y - 1) / course_height;
        
        // Bilinear interpolation
        int x0 = static_cast<int>(floor(x_idx));
        int y0 = static_cast<int>(floor(y_idx));
        int x1 = std::min(x0 + 1, terrain_resolution_x - 1);
        int y1 = std::min(y0 + 1, terrain_resolution_y - 1);
        
        double dx = x_idx - x0;
        double dy = y_idx - y0;
        
        double z00 = terrain_elevation[y0][x0];
        double z01 = terrain_elevation[y0][x1];
        double z10 = terrain_elevation[y1][x0];
        double z11 = terrain_elevation[y1][x1];
        
        double z0 = z00 * (1 - dx) + z01 * dx;
        double z1 = z10 * (1 - dx) + z11 * dx;
        
        return z0 * (1 - dy) + z1 * dy;
    }
    
    std::pair<double, double> getGradient(double x, double y) {
        if (x < 0 || x > course_width || y < 0 || y > course_height) {
            return {0, 0};
        }
        
        // Convert x,y to grid indices
        double x_idx = x * (terrain_resolution_x - 1) / course_width;
        double y_idx = y * (terrain_resolution_y - 1) / course_height;
        
        // Bilinear interpolation
        int x0 = static_cast<int>(floor(x_idx));
        int y0 = static_cast<int>(floor(y_idx));
        int x1 = std::min(x0 + 1, terrain_resolution_x - 1);
        int y1 = std::min(y0 + 1, terrain_resolution_y - 1);
        
        double dx = x_idx - x0;
        double dy = y_idx - y0;
        
        // Gradients at grid points
        double gx00 = gradient_x[y0][x0];
        double gx01 = gradient_x[y0][x1];
        double gx10 = gradient_x[y1][x0];
        double gx11 = gradient_x[y1][x1];
        
        double gy00 = gradient_y[y0][x0];
        double gy01 = gradient_y[y0][x1];
        double gy10 = gradient_y[y1][x0];
        double gy11 = gradient_y[y1][x1];
        
        // Interpolate
        double gx0 = gx00 * (1 - dx) + gx01 * dx;
        double gx1 = gx10 * (1 - dx) + gx11 * dx;
        double grad_x = gx0 * (1 - dy) + gx1 * dy;
        
        double gy0 = gy00 * (1 - dx) + gy01 * dx;
        double gy1 = gy10 * (1 - dx) + gy11 * dx;
        double grad_y = gy0 * (1 - dy) + gy1 * dy;
        
        return {grad_x, grad_y};
    }
    
    std::pair<std::vector<std::vector<double>>, std::string> simulateBall(double vx, double vy, double vz) {
        // Current position
        double x = start_x;
        double y = start_y;
        double z = getElevation(x, y);
        
        // Store trajectory: [x, y, z] points
        std::vector<std::vector<double>> trajectory;
        trajectory.push_back({x, y, z});
        
        // Run simulation until max time
        for (double t = 0; t < max_time; t += dt) {
            // Get terrain gradient at current position
            auto [grad_x, grad_y] = getGradient(x, y);
            
            // Gravity components due to slope
            double g_x = gravity * grad_x;
            double g_y = gravity * grad_y;
            double g_z = -gravity;  // Vertical gravity always points down
            
            // Apply terrain forces to velocity
            vx += g_x * dt;
            vy += g_y * dt;
            vz += g_z * dt;
            
            // Update position
            x += vx * dt;
            y += vy * dt;
            z += vz * dt;
            
            // Check if ball hits the ground
            double terrain_z = getElevation(x, y);
            if (z < terrain_z) {
                // Ball hits the ground
                z = terrain_z;
                
                // Simple bounce model (lose some energy)
                vz = -vz * k;
                
                // Apply friction to horizontal velocity on contact
                double speed_h = sqrt(vx*vx + vy*vy);
                if (speed_h > 0) {
                    double friction_decel = friction;
                    vx -= friction_decel * vx / speed_h;
                    vy -= friction_decel * vy / speed_h;
                }
            }
            
            // ball close to stopped
            if (sqrt(vx*vx + vy*vy + vz*vz) < 0.1) {
                trajectory.push_back({x, y, z});
                
                // Check if the stopped ball is in the hole
                if (sqrt(pow(x - hole_x, 2) + pow(y - hole_y, 2)) < hole_radius) {
                    return {trajectory, "hole"};
                } else {
                    return {trajectory, "stopped"};
                }
            }
            
            // Check if ball is in the hole
            if (sqrt(pow(x - hole_x, 2) + pow(y - hole_y, 2)) < hole_radius) {
                trajectory.push_back({hole_x, hole_y, getElevation(hole_x, hole_y)});
                return {trajectory, "hole"};
            }
            
            // Check if ball is out of bounds
            if (x < 0 || x > course_width || y < 0 || y > course_height) {
                trajectory.push_back({x, y, z});
                return {trajectory, "out"};
            }
            
            // Add current position to trajectory
            trajectory.push_back({x, y, z});
        }
        
        return {trajectory, "timeout"};
    }
    
    void runSimulation() {
        json trajectories_json = json::object();
        json results_json = json::object();
        json terrain_json = json::object();
        
        // terrain data
        json terrain_elevation_json = json::array();
        json x_coords = json::array();
        json y_coords = json::array();

        // coordinate arrays
        for (int x = 0; x < terrain_resolution_x; x++) {
            x_coords.push_back(x * course_width / (terrain_resolution_x - 1));
        }
        
        for (int y = 0; y < terrain_resolution_y; y++) {
            y_coords.push_back(y * course_height / (terrain_resolution_y - 1));
        }
        
        // elevation data
        for (int y = 0; y < terrain_resolution_y; y++) {
            json row = json::array();
            for (int x = 0; x < terrain_resolution_x; x++) {
                row.push_back(terrain_elevation[y][x]);
            }
            terrain_elevation_json.push_back(row);
        }
        
        terrain_json["X"] = x_coords;
        terrain_json["Y"] = y_coords;
        terrain_json["Z"] = terrain_elevation_json;

        // Run sim for all balls
        for (const auto& [speed, angle] : speeds_angles_pairs) {
            // Create sim key
            std::string key = std::to_string(speed) + "_" + std::to_string(angle);
            
            // Convert angle to radians
            double angle_rad = angle * M_PI / 180.0;
            
            // Initial velocity components
            double vx = speed * cos(angle_rad);
            double vy = speed * sin(angle_rad);
            double vz = 0;  // No initial vertical velocity
            
            // Simulate this ball
            auto [trajectory, result] = simulateBall(vx, vy, vz);
            
            // Store results
            results_json[key] = result;
            
            // Store trajectory
            json traj_json = json::array();
            for (const auto& point : trajectory) {
                traj_json.push_back({point[0], point[1], point[2]});
            }
            trajectories_json[key] = traj_json;
        }
        
        // Combine everything into final output
        json output;
        output["terrain"] = terrain_json;
        output["trajectories"] = trajectories_json;
        output["results"] = results_json;
        output["course_info"] = {
            {"width", course_width},
            {"height", course_height},
            {"start_x", start_x},
            {"start_y", start_y},
            {"hole_x", hole_x},
            {"hole_y", hole_y},
            {"hole_radius", hole_radius}
        };
        
        results = output;
    }
    
    void saveResultsToFile(const std::string& filename) {
        std::ofstream output_file(filename);
        if (output_file.is_open()) {
            output_file << results.dump(2);
            output_file.close();
        } else {
            std::cerr << "Error: Could not open file for writing: " << filename << std::endl;
        }
    }
};

// Parse cli args
void parseArgs(int argc, char* argv[], double& friction, double& dt, double& max_time, 
               std::vector<double>& speeds, std::vector<double>& angles, std::string& output_file) {
    
    // defaults
    friction = 0.3;
    dt = 0.05;
    max_time = 15.0;
    output_file = "simulation_results.json";
    
    // Default speed/angle ranges
    double speed_min = 5.0, speed_max = 30.0;
    int speed_count = 20;
    double angle_min = -45.0, angle_max = 45.0;
    int angle_count = 20;
    
    // lli args
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "--friction" && i + 1 < argc) {
            friction = std::stod(argv[++i]);
        } else if (arg == "--dt" && i + 1 < argc) {
            dt = std::stod(argv[++i]);
        } else if (arg == "--max-time" && i + 1 < argc) {
            max_time = std::stod(argv[++i]);
        } else if (arg == "--output" && i + 1 < argc) {
            output_file = argv[++i];
        } else if (arg == "--speed-range" && i + 2 < argc) {
            speed_min = std::stod(argv[++i]);
            speed_max = std::stod(argv[++i]);
        } else if (arg == "--speed-count" && i + 1 < argc) {
            speed_count = std::stoi(argv[++i]);
        } else if (arg == "--angle-range" && i + 2 < argc) {
            angle_min = std::stod(argv[++i]);
            angle_max = std::stod(argv[++i]);
        } else if (arg == "--angle-count" && i + 1 < argc) {
            angle_count = std::stoi(argv[++i]);
        }
    }
    
    // Generate speeds and angles arrays
    if (speed_count <= 1) {
        speeds = {speed_min};
    } else {
        double speed_step = (speed_max - speed_min) / (speed_count - 1);
        for (int i = 0; i < speed_count; i++) {
            speeds.push_back(speed_min + i * speed_step);
        }
    }
    
    if (angle_count <= 1) {
        angles = {angle_min};
    } else {
        double angle_step = (angle_max - angle_min) / (angle_count - 1);
        for (int i = 0; i < angle_count; i++) {
            angles.push_back(angle_min + i * angle_step);
        }
    }
}

int main(int argc, char* argv[]) {
    // Parse command line arguments
    double friction, dt, max_time;
    std::vector<double> speeds, angles;
    std::string output_file;
    
    parseArgs(argc, argv, friction, dt, max_time, speeds, angles, output_file);
    
    std::cout << "Starting simulation with parameters:" << std::endl;
    std::cout << "  Friction: " << friction << std::endl;
    std::cout << "  Time step: " << dt << std::endl;
    std::cout << "  Max time: " << max_time << std::endl;
    std::cout << "  Speeds: " << speeds.size() << " values from " << speeds.front() << " to " << speeds.back() << std::endl;
    std::cout << "  Angles: " << angles.size() << " values from " << angles.front() << " to " << angles.back() << std::endl;
    std::cout << "  Output file: " << output_file << std::endl;
    
    std::cout << "Initializing simulation..." << std::endl;
    GolfSimulation3D simulation(friction, dt, max_time);
    simulation.setSpeedsAndAngles(speeds, angles);
    
    std::cout << "Running simulation..." << std::endl;
    simulation.runSimulation();
    
    std::cout << "Saving results to " << output_file << std::endl;
    simulation.saveResultsToFile(output_file);
    
    std::cout << "Simulation complete!" << std::endl;
    return 0;
}