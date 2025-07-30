#pragma once

#include "VampOMPLInterfaces.h"
#include <vamp/collision/factory.hh>
#include <vector>
#include <array>
#include <cmath>
#include <map>
#include <sstream>

namespace vamp_ompl {

/**
 * @brief Creates an empty environment for basic testing
 */
class EmptyEnvironmentFactory : public EnvironmentFactory {
public:
    vamp::collision::Environment<float> createEnvironment() override
    {
        vamp::collision::Environment<float> environment;
        environment.sort();
        return environment;
    }
    
    std::string getEnvironmentName() const override
    {
        return "Empty Environment";
    }
    
    std::string getDescription() const override
    {
        return "Empty space with no obstacles - useful for testing basic motion";
    }
};

/**
 * @brief Creates a sphere cage environment
 */
class SphereCageEnvironmentFactory : public EnvironmentFactory {
public:
    /**
     * @brief Constructor with default working parameters
     */
    SphereCageEnvironmentFactory(float sphere_radius = 0.15f)
        : sphere_radius_(sphere_radius)
    {
    }
    
    vamp::collision::Environment<float> createEnvironment() override
    {
        vamp::collision::Environment<float> environment;

        // Create a cage of spheres arranged in two rings
        std::vector<std::array<float, 3>> sphere_positions = {
            {0.55, 0, 0.25}, {0.35, 0.35, 0.25}, {0, 0.55, 0.25},
            {-0.55, 0, 0.25}, {-0.35, -0.35, 0.25}, {0, -0.55, 0.25},
            {0.35, -0.35, 0.25}, {0.35, 0.35, 0.8}, {0, 0.55, 0.8},
            {-0.35, 0.35, 0.8}, {-0.55, 0, 0.8}, {-0.35, -0.35, 0.8},
            {0, -0.55, 0.8}, {0.35, -0.35, 0.8}
        };

        for (const auto &sphere : sphere_positions)
        {
            environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, sphere_radius_));
        }
        
        environment.sort();
        return environment;
    }
    
    std::string getEnvironmentName() const override
    {
        return "Sphere Cage";
    }
    
    std::string getDescription() const override
    {
        return "A cage made of spheres arranged in two rings - tests navigation in constrained space";
    }

private:
    float sphere_radius_;
};

/**
 * @brief Creates a table scene environment
 */
class TableSceneEnvironmentFactory : public EnvironmentFactory {
public:
    /**
     * @brief Constructor with default parameters for table scene
    */

    TableSceneEnvironmentFactory()
    {
    }
    
    vamp::collision::Environment<float> createEnvironment() override
    {
        vamp::collision::Environment<float> environment;
        
        // Define positions for the table surface and obstacles
        std::vector<std::array<float, 3>> box_positions = {
            // Table surface (1m x 1m table at Z=0.8m height)
            {-0.5, -0.5, 0.8}, {0.5, 0.5, 0.82},
            // Obstacles on table surface
            {0.2, 0.1, 0.82}, {0.3, 0.2, 0.92},
            {-0.2, 0.2, 0.82}, {-0.1, 0.3, 0.92},
            {0.0, -0.2, 0.82}, {0.1, -0.1, 0.92}
        };

        // Add table surface and obstacles
        for (size_t i = 0; i < box_positions.size(); i += 2) {
            auto min_corner = box_positions[i];
            auto max_corner = box_positions[i + 1];
            // Compute center and half extents
            std::array<float, 3> center = {
                0.5f * (min_corner[0] + max_corner[0]),
                0.5f * (min_corner[1] + max_corner[1]),
                0.5f * (min_corner[2] + max_corner[2])
            };
            std::array<float, 3> half_extents = {
                0.5f * std::abs(max_corner[0] - min_corner[0]),
                0.5f * std::abs(max_corner[1] - min_corner[1]),
                0.5f * std::abs(max_corner[2] - min_corner[2])
            };
            std::array<float, 3> euler_xyz = {0.0f, 0.0f, 0.0f}; // axis-aligned
            environment.cuboids.emplace_back(
                vamp::collision::factory::cuboid::array(center, euler_xyz, half_extents)
            );
        }
        
        environment.sort();
        return environment;
    }
    
    std::string getEnvironmentName() const override
    {
        return "Table Scene";
    }
    
    std::string getDescription() const override
    {
        return "Table with obstacles - simulates tabletop manipulation scenarios";
    }
};

/**
 * @brief Configuration structure for custom obstacles
 */
struct ObstacleConfig {
    std::string type;           // "sphere", "cuboid", "capsule"
    std::string name;           // Optional name for the obstacle
    std::array<float, 3> position;
    std::array<float, 3> orientation_euler_xyz = {0.0f, 0.0f, 0.0f}; // For cuboids and capsules
    float radius = 0.1f;        // For spheres and capsules
    std::array<float, 3> half_extents = {0.1f, 0.1f, 0.1f}; // For cuboids
    float length = 0.2f;        // For capsules
    
    ObstacleConfig() = default;
    
    ObstacleConfig(const std::string& obstacle_type, 
                   const std::array<float, 3>& pos,
                   float r = 0.1f)
        : type(obstacle_type), position(pos), radius(r) {}
        
    ObstacleConfig(const std::string& obstacle_type,
                   const std::array<float, 3>& pos,
                   const std::array<float, 3>& half_ext)
        : type(obstacle_type), position(pos), half_extents(half_ext) {}
};

/**
 * @brief Creates a custom environment from user-specified obstacles
 */
class CustomEnvironmentFactory : public EnvironmentFactory {
private:
    std::vector<ObstacleConfig> obstacles_;
    std::string environment_name_;
    std::string description_;
    
public:
    /**
     * @brief Constructor with obstacles list
     */
    CustomEnvironmentFactory(const std::vector<ObstacleConfig>& obstacles,
                           const std::string& name = "Custom Environment",
                           const std::string& desc = "User-defined custom environment")
        : obstacles_(obstacles), environment_name_(name), description_(desc)
    {
    }
    
    /**
     * @brief Add a sphere obstacle
     */
    void addSphere(const std::array<float, 3>& position, float radius, const std::string& name = "") {
        ObstacleConfig config("sphere", position, radius);
        config.name = name.empty() ? "sphere_" + std::to_string(obstacles_.size()) : name;
        obstacles_.push_back(config);
    }
    
    /**
     * @brief Add a cuboid obstacle
     */
    void addCuboid(const std::array<float, 3>& position, 
                   const std::array<float, 3>& half_extents,
                   const std::array<float, 3>& orientation_euler_xyz = {0.0f, 0.0f, 0.0f},
                   const std::string& name = "") {
        ObstacleConfig config("cuboid", position, half_extents);
        config.orientation_euler_xyz = orientation_euler_xyz;
        config.name = name.empty() ? "cuboid_" + std::to_string(obstacles_.size()) : name;
        obstacles_.push_back(config);
    }
    
    /**
     * @brief Add a capsule obstacle
     */
    void addCapsule(const std::array<float, 3>& position,
                    const std::array<float, 3>& orientation_euler_xyz,
                    float radius, float length,
                    const std::string& name = "") {
        ObstacleConfig config("capsule", position, radius);
        config.orientation_euler_xyz = orientation_euler_xyz;
        config.length = length;
        config.name = name.empty() ? "capsule_" + std::to_string(obstacles_.size()) : name;
        obstacles_.push_back(config);
    }
    
    vamp::collision::Environment<float> createEnvironment() override
    {
        vamp::collision::Environment<float> environment;
        
        for (const auto& obstacle : obstacles_) {
            if (obstacle.type == "sphere") {
                environment.spheres.emplace_back(
                    vamp::collision::factory::sphere::array(obstacle.position, obstacle.radius)
                );
                if (!obstacle.name.empty()) {
                    environment.spheres.back().name = obstacle.name;
                }
                
            } else if (obstacle.type == "cuboid") {
                environment.cuboids.emplace_back(
                    vamp::collision::factory::cuboid::array(
                        obstacle.position, 
                        obstacle.orientation_euler_xyz, 
                        obstacle.half_extents
                    )
                );
                if (!obstacle.name.empty()) {
                    environment.cuboids.back().name = obstacle.name;
                }
                
            } else if (obstacle.type == "capsule") {
                environment.capsules.emplace_back(
                    vamp::collision::factory::capsule::center::array(
                        obstacle.position,
                        obstacle.orientation_euler_xyz,
                        obstacle.radius,
                        obstacle.length
                    )
                );
                if (!obstacle.name.empty()) {
                    environment.capsules.back().name = obstacle.name;
                }
            } else {
                std::cerr << "Warning: Unknown obstacle type '" << obstacle.type 
                          << "'. Skipping obstacle." << std::endl;
            }
        }
        
        environment.sort();
        return environment;
    }
    
    std::string getEnvironmentName() const override
    {
        return environment_name_;
    }
    
    std::string getDescription() const override
    {
        std::string desc = description_ + " (" + std::to_string(obstacles_.size()) + " obstacles: ";
        std::map<std::string, int> type_counts;
        for (const auto& obstacle : obstacles_) {
            type_counts[obstacle.type]++;
        }
        bool first = true;
        for (const auto& [type, count] : type_counts) {
            if (!first) desc += ", ";
            desc += std::to_string(count) + " " + type + (count > 1 ? "s" : "");
            first = false;
        }
        desc += ")";
        return desc;
    }
    
    /**
     * @brief Get the obstacle configurations for serialization/debugging
     */
    const std::vector<ObstacleConfig>& getObstacles() const {
        return obstacles_;
    }
    
    /**
     * @brief Clear all obstacles
     */
    void clearObstacles() {
        obstacles_.clear();
    }
    
    /**
     * @brief Serialize obstacles to string format for visualization
     */
    std::string serializeObstacles() const {
        std::ostringstream oss;
        bool first = true;
        
        for (const auto& obstacle : obstacles_) {
            if (!first) oss << ";";
            first = false;
            
            bool first_prop = true;
            
            // Add type
            oss << "type=" << obstacle.type;
            first_prop = false;
            
            // Add position
            if (!first_prop) oss << ",";
            oss << "position_x=" << obstacle.position[0];
            oss << ",position_y=" << obstacle.position[1];
            oss << ",position_z=" << obstacle.position[2];
            
            // Add name if available
            if (!obstacle.name.empty()) {
                oss << ",name=" << obstacle.name;
            }
            
            // Add type-specific properties
            if (obstacle.type == "sphere") {
                oss << ",radius=" << obstacle.radius;
            } else if (obstacle.type == "cuboid") {
                oss << ",half_extents_x=" << obstacle.half_extents[0];
                oss << ",half_extents_y=" << obstacle.half_extents[1];
                oss << ",half_extents_z=" << obstacle.half_extents[2];
                oss << ",orientation_x=" << obstacle.orientation_euler_xyz[0];
                oss << ",orientation_y=" << obstacle.orientation_euler_xyz[1];
                oss << ",orientation_z=" << obstacle.orientation_euler_xyz[2];
            } else if (obstacle.type == "capsule") {
                oss << ",radius=" << obstacle.radius;
                oss << ",length=" << obstacle.length;
                oss << ",orientation_x=" << obstacle.orientation_euler_xyz[0];
                oss << ",orientation_y=" << obstacle.orientation_euler_xyz[1];
                oss << ",orientation_z=" << obstacle.orientation_euler_xyz[2];
            }
        }
        
        return oss.str();
    }
    
    /**
     * @brief Print obstacle configuration for debugging
     */
    void printConfiguration() const {
        std::cout << "\n--- Custom Environment Configuration ---" << std::endl;
        std::cout << "Name: " << environment_name_ << std::endl;
        std::cout << "Total obstacles: " << obstacles_.size() << std::endl;
        
        for (size_t i = 0; i < obstacles_.size(); ++i) {
            const auto& obs = obstacles_[i];
            std::cout << "  [" << i << "] " << obs.type;
            if (!obs.name.empty()) std::cout << " (" << obs.name << ")";
            std::cout << ": pos=[" << obs.position[0] << ", " << obs.position[1] << ", " << obs.position[2] << "]";
            
            if (obs.type == "sphere") {
                std::cout << ", radius=" << obs.radius;
            } else if (obs.type == "cuboid") {
                std::cout << ", half_extents=[" << obs.half_extents[0] << ", " << obs.half_extents[1] << ", " << obs.half_extents[2] << "]";
                std::cout << ", orientation=[" << obs.orientation_euler_xyz[0] << ", " << obs.orientation_euler_xyz[1] << ", " << obs.orientation_euler_xyz[2] << "]";
            } else if (obs.type == "capsule") {
                std::cout << ", radius=" << obs.radius << ", length=" << obs.length;
                std::cout << ", orientation=[" << obs.orientation_euler_xyz[0] << ", " << obs.orientation_euler_xyz[1] << ", " << obs.orientation_euler_xyz[2] << "]";
            }
            std::cout << std::endl;
        }
        std::cout << "----------------------------------------" << std::endl;
    }
    
    /**
     * @brief Create predefined custom environments for demos
     */
    static std::unique_ptr<CustomEnvironmentFactory> createSampleEnvironment(const std::string& sample_name = "mixed") {
        auto factory = std::make_unique<CustomEnvironmentFactory>(
            std::vector<ObstacleConfig>{}, 
            "Custom Sample Environment", 
            "Sample environment with mixed obstacles"
        );
        
        if (sample_name == "mixed") {
            // Create a mixed environment with different obstacle types
            factory->addSphere({0.3f, 0.3f, 0.5f}, 0.1f, "sphere_1");
            factory->addSphere({-0.3f, -0.3f, 0.7f}, 0.08f, "sphere_2");
            factory->addCuboid({0.0f, -0.4f, 0.6f}, {0.05f, 0.15f, 0.1f}, {0.0f, 0.0f, 0.785f}, "cuboid_1");
            factory->addCuboid({-0.2f, 0.2f, 0.4f}, {0.1f, 0.05f, 0.2f}, {0.0f, 0.0f, 0.0f}, "cuboid_2");
            factory->addCapsule({0.4f, -0.1f, 0.8f}, {1.57f, 0.0f, 0.0f}, 0.06f, 0.25f, "capsule_1");
            
        } else if (sample_name == "spheres_only") {
            // Only spheres for simpler testing
            factory->addSphere({0.2f, 0.2f, 0.5f}, 0.12f, "sphere_1");
            factory->addSphere({-0.2f, -0.2f, 0.7f}, 0.1f, "sphere_2");
            factory->addSphere({0.0f, -0.3f, 0.4f}, 0.08f, "sphere_3");
            factory->addSphere({0.3f, -0.1f, 0.8f}, 0.09f, "sphere_4");
            
        } else if (sample_name == "cuboids_only") {
            // Only cuboids for testing
            factory->addCuboid({0.15f, 0.15f, 0.5f}, {0.08f, 0.08f, 0.15f}, {0.0f, 0.0f, 0.0f}, "cuboid_1");
            factory->addCuboid({-0.15f, -0.15f, 0.7f}, {0.1f, 0.06f, 0.12f}, {0.0f, 0.0f, 0.785f}, "cuboid_2");
            factory->addCuboid({0.0f, -0.25f, 0.4f}, {0.06f, 0.12f, 0.08f}, {0.0f, 0.0f, 1.57f}, "cuboid_3");
        }
        
        return factory;
    }
};

} // namespace vamp_ompl