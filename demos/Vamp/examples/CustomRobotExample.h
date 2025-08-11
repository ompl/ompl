#pragma once

/**
 * @file CustomRobotExample.h
 * @brief Example of how to define and register a custom robot with VAMP
 * 
 * This file demonstrates the complete process of adding a new robot to VAMP
 * without modifying the core architecture. It shows how to:
 * 1. Define a custom robot with VAMP interface
 * 2. Implement vectorized forward kinematics
 * 3. Register the robot with the registry
 * 4. Use the robot in planning
 */

#include "VampRobotRegistry.h"
#include <vamp/vector.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/validity.hh>
#include <vamp/collision/capt.hh>
#include <array>
#include <string_view>

namespace vamp::robots {

/**
 * @brief Example 2-DOF Planar Robot
 * 
 * This robot demonstrates the minimal interface required for VAMP integration.
 * It's a 2-DOF planar manipulator with simple kinematics for educational purposes.
 * 
 */
struct PlanarArm2DOF {
    static constexpr auto name = "planar_arm_2dof";
    static constexpr auto dimension = 2;      // 2 joints
    static constexpr auto n_spheres = 3;      // 3 collision spheres
    static constexpr auto resolution = 16;    // Motion validation resolution
    
    // Joint limits: [-π, π] for both joints
    static constexpr std::array<float, dimension> s_a = {-3.14159f, -3.14159f}; // Lower limits
    static constexpr std::array<float, dimension> s_m = {6.28318f, 6.28318f};   // Ranges
    
    // Joint names for debugging
    static constexpr std::array<std::string_view, dimension> joint_names = {
        "shoulder_joint", "elbow_joint"
    };
    
    static constexpr const char *end_effector = "end_effector_frame";
    
    // Configuration types
    using Configuration = FloatVector<dimension>;
    using ConfigurationArray = std::array<FloatT, dimension>;
    
    template <std::size_t rake>
    using ConfigurationBlock = FloatVector<rake, dimension>;
    
    struct alignas(FloatVectorAlignment) ConfigurationBuffer
      : std::array<float, Configuration::num_scalars_rounded>
    {
    };
    
    // Collision sphere data structure for vectorized operations
    template <std::size_t rake>
    struct Spheres {
        FloatVector<rake, 1> x[n_spheres];  // X coordinates
        FloatVector<rake, 1> y[n_spheres];  // Y coordinates  
        FloatVector<rake, 1> z[n_spheres];  // Z coordinates (always 0 for planar)
        FloatVector<rake, 1> r[n_spheres];  // Radii
    };
    
    // Physical parameters (link lengths and sphere radii)
    static constexpr float link1_length = 0.3f;    // 30cm upper arm
    static constexpr float link2_length = 0.25f;   // 25cm forearm
    static constexpr float base_sphere_radius = 0.05f;     // 5cm base
    static constexpr float link1_sphere_radius = 0.04f;    // 4cm upper arm
    static constexpr float link2_sphere_radius = 0.03f;    // 3cm forearm
    
    // Required interface methods
    inline static void scale_configuration(Configuration &q) noexcept {
        Configuration s_a_vec(s_a.data());
        Configuration s_m_vec(s_m.data());
        q = q * s_m_vec + s_a_vec;
    }
    
    inline static void descale_configuration(Configuration &q) noexcept {
        Configuration s_a_vec(s_a.data());
        Configuration s_m_vec(s_m.data());
        q = (q - s_a_vec) / s_m_vec;
    }
    
    template <std::size_t rake>
    inline static void scale_configuration_block(ConfigurationBlock<rake> &q) noexcept {
        for (auto joint_idx = 0U; joint_idx < dimension; ++joint_idx) {
            q[joint_idx] = s_a[joint_idx] + (q[joint_idx] * s_m[joint_idx]);
        }
    }
    
    template <std::size_t rake>
    inline static void descale_configuration_block(ConfigurationBlock<rake> &q) noexcept {
        for (auto joint_idx = 0U; joint_idx < dimension; ++joint_idx) {
            q[joint_idx] = (q[joint_idx] - s_a[joint_idx]) / s_m[joint_idx];
        }
    }
    
    inline static auto space_measure() noexcept -> float {
        Configuration s_m_vec(s_m.data());
        return s_m_vec.l2_norm();
    }
    
    /**
     * @brief Vectorized forward kinematics with collision checking
     * 
     * This is the core function that VAMP calls for collision detection.
     * It computes the positions of all collision spheres for 'rake' configurations
     * simultaneously using SIMD operations.
     * 
     * @tparam rake Number of configurations to process simultaneously (SIMD width)
     * @param environment Collision environment to check against
     * @param q Block of joint configurations
     * @return true if all configurations are collision-free
     */
    template <std::size_t rake>
    inline static auto fkcc(
        const collision::Environment<FloatVector<rake>> &environment,
        const ConfigurationBlock<rake> &q) noexcept -> bool
    {
        Spheres<rake> spheres;
        
        // Joint angles
        auto q1 = q[0];  // Shoulder angle
        auto q2 = q[1];  // Elbow angle
        
        // Trigonometric functions
        auto c1 = q1.cos();
        auto s1 = q1.sin();
        auto c12 = (q1 + q2).cos();  // Combined angle
        auto s12 = (q1 + q2).sin();
        
        // Sphere 0: Base (always at origin)
        spheres.x[0] = FloatVector<rake>::fill(0.0f);
        spheres.y[0] = FloatVector<rake>::fill(0.0f);
        spheres.z[0] = FloatVector<rake>::fill(0.0f);
        spheres.r[0] = FloatVector<rake>::fill(base_sphere_radius);
        
        // Sphere 1: Upper arm midpoint
        spheres.x[1] = c1 * (link1_length * 0.5f);
        spheres.y[1] = s1 * (link1_length * 0.5f);
        spheres.z[1] = FloatVector<rake>::fill(0.0f);
        spheres.r[1] = FloatVector<rake>::fill(link1_sphere_radius);
        
        // Sphere 2: Forearm (end-effector)
        auto end_x = c1 * link1_length + c12 * link2_length;
        auto end_y = s1 * link1_length + s12 * link2_length;
        
        spheres.x[2] = end_x;
        spheres.y[2] = end_y;
        spheres.z[2] = FloatVector<rake>::fill(0.0f);
        spheres.r[2] = FloatVector<rake>::fill(link2_sphere_radius);
        
        // Check each sphere for environment collision (return false if any collision)
        for (size_t i = 0; i < n_spheres; ++i) {
            if (sphere_environment_in_collision(environment, spheres.x[i], spheres.y[i], spheres.z[i], spheres.r[i])) {
                return false;
            }
        }
        
        return true;  // No collisions detected
    }
    
    /**
     * @brief Forward kinematics with attachment support
     * 
     * For this simple robot, we don't implement attachments,
     * but the interface is required for VAMP compatibility.
     */
    template <std::size_t rake>
    inline static auto fkcc_attach(
        const collision::Environment<FloatVector<rake>> &environment,
        const ConfigurationBlock<rake> &q) noexcept -> bool
    {
        // For this simple robot, just call the standard fkcc
        return fkcc<rake>(environment, q);
    }
};

} // namespace vamp::robots

// Specializations for custom robots (must come after robot definitions)
namespace vamp_ompl {

template<>
inline RobotMetadata getRobotMetadata<vamp::robots::PlanarArm2DOF>() {
    return RobotMetadata{
        .name = "planar_arm_2dof",
        .description = "2-DOF Planar Arm Manipulator",
        .dimension = 2,
        .n_spheres = vamp::robots::PlanarArm2DOF::n_spheres,
        .radii_range = {0.01f, 1.0f},
        .joint_names = {"shoulder_joint", "elbow_joint"},
        .end_effector_frame = "end_effector_frame"
    };
}

// Register the custom robots automatically when this header is included

// Use anonymous namespace to ensure registration happens only once per translation unit
namespace {
    
/**
 * @brief Automatic registration of custom robots
 * 
 * These static objects will register the robots when the library is loaded.
 * The REGISTER_VAMP_ROBOT macro provides a cleaner interface.
 */
static RobotRegistrar<vamp::robots::PlanarArm2DOF> planar_arm_registrar("planar_arm_2dof");

} // anonymous namespace

} // namespace vamp_ompl

// Alternative registration approach using macros
// Users can use this in their own code:
// REGISTER_VAMP_ROBOT(MyCustomRobot, "my_robot_name");
//
// The macro expands to:
// namespace { 
//     static vamp_ompl::RobotRegistrar<MyCustomRobot> registrar_MyCustomRobot("my_robot_name"); 
// } 