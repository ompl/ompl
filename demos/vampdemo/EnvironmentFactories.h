#pragma once

#include "VampOMPLInterfaces.h"
#include <vamp/collision/factory.hh>
#include <vector>
#include <array>
#include <cmath>

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
 * @brief Creates a sphere cage environment using exact working configuration from original demo
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
        
        // Use exact sphere positions from working original demo for Panda
        // This will work for all robots since it's a general cage
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
 * @brief Creates a table scene environment using exact working configuration from original demo
 */
class TableSceneEnvironmentFactory : public EnvironmentFactory {
public:
    /**
     * @brief Constructor - uses working Panda table scene configuration
     */
    TableSceneEnvironmentFactory()
    {
    }
    
    vamp::collision::Environment<float> createEnvironment() override
    {
        vamp::collision::Environment<float> environment;
        
        // Use exact box configuration from working original demo for Panda table scene
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

} // namespace vamp_ompl