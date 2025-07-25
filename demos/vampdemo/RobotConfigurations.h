#pragma once

#include "VampOMPLInterfaces.h"
#include <vamp/robots/panda.hh>
#include <vamp/robots/ur5.hh>
#include <vamp/robots/fetch.hh>

namespace vamp_ompl {

/**
 * @brief Panda robot configuration with joint limits and default poses
 */
class PandaConfig : public RobotConfig<vamp::robots::Panda> {
public:
    std::vector<std::pair<double, double>> getJointLimits() const override
    {
        // Panda joint limits from URDF specification
        return {
            {-2.9671, 2.9671},   // joint1
            {-1.8326, 1.8326},   // joint2  
            {-2.9671, 2.9671},   // joint3
            {-3.1416, 0.0873},   // joint4
            {-2.9671, 2.9671},   // joint5
            {-0.0873, 3.8223},   // joint6
            {-2.9671, 2.9671}    // joint7
        };
    }
    
    std::array<float, dimension> getStartConfigurationArray() const override
    {
        return {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
    }
    
    std::array<float, dimension> getGoalConfigurationArray() const override
    { 
        return {2.35, 1., 0., -0.8, 0, 2.5, 0.785};
    }
    
    std::string getRobotName() const override
    {
        return "Franka Emika Panda";
    }
};

/**
 * @brief Panda robot configuration for table scene environment
 * Uses different start/goal poses suitable for table manipulation
 */
class PandaTableConfig : public RobotConfig<vamp::robots::Panda> {
public:
    std::vector<std::pair<double, double>> getJointLimits() const override
    {
        return {
            {-2.9671, 2.9671},   // joint1
            {-1.8326, 1.8326},   // joint2  
            {-2.9671, 2.9671},   // joint3
            {-3.1416, 0.0873},   // joint4
            {-2.9671, 2.9671},   // joint5
            {-0.0873, 3.8223},   // joint6
            {-2.9671, 2.9671}    // joint7
        };
    }
    
    std::array<float, dimension> getStartConfigurationArray() const override
    {
        return {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
    }
    
    std::array<float, dimension> getGoalConfigurationArray() const override
    {
        return {0., -0.5599, 0., -2.97, 0., 0., 0.785};
    }
    
    std::string getRobotName() const override
    {
        return "Franka Emika Panda (Table Scene)";
    }
};

/**
 * @brief UR5 robot configuration with joint limits and default poses
 */
class UR5Config : public RobotConfig<vamp::robots::UR5> {
public:
    std::vector<std::pair<double, double>> getJointLimits() const override
    {
        // UR5 joint limits - all revolute joints with typical ±180° range
        return {
            {-3.14159265, 3.14159265},  // base_joint
            {-3.14159265, 3.14159265},  // shoulder_joint
            {-3.14159265, 3.14159265},  // elbow_joint
            {-3.14159265, 3.14159265},  // wrist_1_joint
            {-3.14159265, 3.14159265},  // wrist_2_joint
            {-3.14159265, 3.14159265}   // wrist_3_joint
        };
    }
    
    std::array<float, dimension> getStartConfigurationArray() const override
    {
        return {0., -1.57, 0., -1.57, 0., 0.};
    }
    
    std::array<float, dimension> getGoalConfigurationArray() const override
    {
        return {1.57, -0.785, 0., -2.356, 0., 1.57};
    }
    
    std::string getRobotName() const override
    {
        return "Universal Robots UR5";
    }
};

/**
 * @brief Fetch robot configuration with joint limits and default poses
 */
class FetchConfig : public RobotConfig<vamp::robots::Fetch> {
public:
    std::vector<std::pair<double, double>> getJointLimits() const override
    {
        // Fetch joint limits - mixed prismatic and revolute joints
        return {
            {0.0, 0.38615},        // torso_lift_joint (prismatic)
            {-1.6056, 1.6056},     // shoulder_pan_joint
            {-1.221, 1.518},       // shoulder_lift_joint
            {-3.14159, 3.14159},   // upperarm_roll_joint
            {-2.251, 2.251},       // elbow_flex_joint
            {-3.14159, 3.14159},   // forearm_roll_joint
            {-2.16, 2.16},         // wrist_flex_joint
            {-3.14159, 3.14159}    // wrist_roll_joint
        };
    }
    
    std::array<float, dimension> getStartConfigurationArray() const override
    {
        return {0., 0., 0., 0., 0., 0., 0., 0.};
    }
    
    std::array<float, dimension> getGoalConfigurationArray() const override
    {
        return {0.1, 1.57, 0.785, 0., -1.57, 0., 0., 0.};
    }
    
    std::string getRobotName() const override
    {
        return "Fetch Mobile Manipulator";
    }
};

} // namespace vamp_ompl