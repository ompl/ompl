#pragma once

#include "VampOMPLInterfaces.h"
#include "VampValidators.h"
#include "OMPLPlanningContext.h"
#include "VampOMPLPlanner.h"

// Benchmarking functionality will be available when benchmarking headers are included
#include <functional>
#include <unordered_map>
#include <memory>
#include <any>
#include <mutex>
#include <vector>
#include <string>
#include <map>

// VAMP robot includes for built-in robots
#include <vamp/robots/panda.hh>
#include <vamp/robots/ur5.hh>
#include <vamp/robots/fetch.hh>

// Note: Custom robot specializations should be defined in their own headers,
// not here. This keeps the registry extensible without core modifications.

namespace vamp_ompl {

/**
 * @brief Robot metadata for registry
 */
struct RobotMetadata {
    std::string name;
    std::string description;
    size_t dimension;
    size_t n_spheres;
    std::pair<float, float> radii_range;
    std::vector<std::string> joint_names;
    std::string end_effector_frame;
};

/**
 * @brief Base class for robot operations
 * This interface provides a clean abstraction for robot operations
 */
class RobotHandler {
public:
    virtual ~RobotHandler() = default;
    
    /**
     * @brief Create robot configuration
     */
    virtual std::any createRobotConfig(
        const std::string& robot_name,
        std::vector<float> start_config,
        std::vector<float> goal_config) const = 0;
    
    /**
     * @brief Create VAMP-OMPL planner
     */
    virtual std::any createPlanner(
        std::any robot_config,
        std::unique_ptr<EnvironmentFactory> env_factory) const = 0;
    
    /**
     * @brief Initialize planner
     */
    virtual void initializePlanner(std::any& planner) const = 0;
    
    /**
     * @brief Execute planning
     */
    virtual PlanningResult executePlanning(
        std::any& planner,
        const PlanningConfig& planning_config) const = 0;
    
    /**
     * @brief Get robot metadata
     */
    virtual RobotMetadata getMetadata() const = 0;
    
    /**
     * @brief Validate joint configuration size
     */
    virtual bool validateConfigurationSize(const std::vector<float>& config) const = 0;
    
    /**
     * @brief Get joint limits for the robot
     */
    virtual std::vector<std::pair<double, double>> getJointLimits() const = 0;
    
    // ========== BENCHMARKING INTERFACE ==========
    
    /**
     * @brief Create benchmark manager (following same patterns as createPlanner)
     */
    virtual std::any createBenchmarkManager(
        const std::vector<float>& start_config,
        const std::vector<float>& goal_config,
        std::unique_ptr<EnvironmentFactory> env_factory) const = 0;
    
    /**
     * @brief Execute benchmark (following same patterns as executePlanning)
     */
    virtual std::map<std::string, std::string> executeBenchmark(
        std::any& benchmark_manager,
        const std::string& experiment_name,
        const std::vector<std::string>& planner_names,
        unsigned int runs,
        double timeout) const = 0;
};

/**
 * @brief Helper function to get robot metadata (can be specialized)
 */
template<typename Robot>
RobotMetadata getRobotMetadata() {
    return RobotMetadata{
        .name = std::string(Robot::name),
        .description = std::string(Robot::name) + " robot",
        .dimension = Robot::dimension,
        .n_spheres = Robot::n_spheres,
        .radii_range = {0.01f, 1.0f},
        .joint_names = {},
        .end_effector_frame = "end_effector_frame"
    };
}

// Specializations for built-in robots
template<>
inline RobotMetadata getRobotMetadata<vamp::robots::Panda>() {
    return RobotMetadata{
        .name = "panda",
        .description = "Franka Emika Panda 7-DOF manipulator",
        .dimension = 7,
        .n_spheres = vamp::robots::Panda::n_spheres,
        .radii_range = {0.01f, 1.19f},
        .joint_names = {"panda_joint1", "panda_joint2", "panda_joint3", 
                       "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"},
        .end_effector_frame = "panda_hand"
    };
}

template<>
inline RobotMetadata getRobotMetadata<vamp::robots::UR5>() {
    return RobotMetadata{
        .name = "ur5",
        .description = "Universal Robots UR5 6-DOF manipulator",
        .dimension = 6,
        .n_spheres = vamp::robots::UR5::n_spheres,
        .radii_range = {0.01f, 1.2f},
        .joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"},
        .end_effector_frame = "tool0"
    };
}

template<>
inline RobotMetadata getRobotMetadata<vamp::robots::Fetch>() {
    return RobotMetadata{
        .name = "fetch",
        .description = "Fetch Mobile Manipulator 8-DOF arm",
        .dimension = 8,
        .n_spheres = vamp::robots::Fetch::n_spheres,
        .radii_range = {0.01f, 1.5f},
        .joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint",
                       "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint", "gripper_axis"},
        .end_effector_frame = "gripper_link"
    };
}

// Note: Custom robot specializations should be defined in their respective 
// headers after including this registry header, not here.

/**
 * @brief Template robot handler for specific robot types
 * 
 * This class provides a cleaner implementation using 
 * std::any Type safety is maintained through the
 * template system and std::any_cast. Uses shared_ptr for copyable semantics.
 */
template<typename Robot>
class TypedRobotHandler : public RobotHandler {
public:
    std::any createRobotConfig(
        const std::string& robot_name,
        std::vector<float> start_config,
        std::vector<float> goal_config) const override {
        
        return std::make_shared<RobotConfiguration<Robot>>(
            robot_name, std::move(start_config), std::move(goal_config));
    }
    
    std::any createPlanner(
        std::any robot_config,
        std::unique_ptr<EnvironmentFactory> env_factory) const override {
        
        // Extract the typed robot config from std::any
        auto config_shared = std::any_cast<std::shared_ptr<RobotConfiguration<Robot>>>(robot_config);
        
        // Create a new RobotConfiguration for the planner (copy the configuration data)
        auto start_array = config_shared->getStartConfigurationArray();
        auto goal_array = config_shared->getGoalConfigurationArray();
        std::vector<float> start_vec(start_array.begin(), start_array.end());
        std::vector<float> goal_vec(goal_array.begin(), goal_array.end());
        
        auto config_unique = std::make_unique<RobotConfiguration<Robot>>(
            config_shared->getRobotName(),
            start_vec,
            goal_vec
        );
        
        return std::make_shared<VampOMPLPlanner<Robot>>(
            std::move(config_unique), std::move(env_factory));
    }
    
    void initializePlanner(std::any& planner) const override {
        auto typed_planner = std::any_cast<std::shared_ptr<VampOMPLPlanner<Robot>>>(planner);
        typed_planner->initialize();
    }
    
    PlanningResult executePlanning(
        std::any& planner,
        const PlanningConfig& planning_config) const override {
        
        auto typed_planner = std::any_cast<std::shared_ptr<VampOMPLPlanner<Robot>>>(planner);
        return typed_planner->plan(planning_config);
    }
    
    RobotMetadata getMetadata() const override {
        return getRobotMetadata<Robot>();
    }
    
    bool validateConfigurationSize(const std::vector<float>& config) const override {
        return config.size() == Robot::dimension;
    }
    
    std::vector<std::pair<double, double>> getJointLimits() const override {
        std::vector<std::pair<double, double>> limits;
        for (size_t i = 0; i < Robot::dimension; ++i) {
            double lower = Robot::s_a[i];
            double upper = Robot::s_a[i] + Robot::s_m[i];
            limits.emplace_back(lower, upper);
        }
        return limits;
    }
    
        std::any createBenchmarkManager(
        const std::vector<float>& /* start_config */,
        const std::vector<float>& /* goal_config */,
        std::unique_ptr<EnvironmentFactory> /* env_factory */) const override {

        // This will be implemented when benchmarking headers are included
        // For now, throw an error to indicate benchmarking headers are needed
        throw VampConfigurationError("Benchmarking functionality requires including benchmarking/VampBenchmarking.h");
    }

    std::map<std::string, std::string> executeBenchmark(
        std::any& /* benchmark_manager */,
        const std::string& /* experiment_name */,
        const std::vector<std::string>& /* planner_names */,
        unsigned int /* runs */,
        double /* timeout */) const override {

        // This will be implemented when benchmarking headers are included
        // For now, throw an error to indicate benchmarking headers are needed
        throw VampConfigurationError("Benchmarking functionality requires including benchmarking/VampBenchmarking.h");
    }
};

/**
 * @brief Robot Registry - singleton for robot management
 */
class RobotRegistry {
public:
    /**
     * @brief Get singleton instance
     */
    static RobotRegistry& getInstance() {
        static RobotRegistry instance;
        return instance;
    }
    
    /**
     * @brief Register a robot handler
     */
    template<typename Robot>
    void registerRobot(const std::string& name) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto handler = std::make_unique<TypedRobotHandler<Robot>>();
        robot_metadata_[name] = handler->getMetadata();
        handlers_[name] = std::move(handler);
    }
    
    /**
     * @brief Check if robot is registered
     */
    bool isRobotRegistered(const std::string& name) const {
        std::lock_guard<std::mutex> lock(mutex_);
        return handlers_.find(name) != handlers_.end();
    }
    
    /**
     * @brief Get robot metadata
     */
    RobotMetadata getRobotMetadata(const std::string& name) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = robot_metadata_.find(name);
        if (it == robot_metadata_.end()) {
            throw VampConfigurationError("Robot not registered: " + name);
        }
        return it->second;
    }
    
    /**
     * @brief Get list of registered robots
     */
    std::vector<std::string> getRegisteredRobots() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<std::string> robots;
        for (const auto& [name, handler] : handlers_) {
            robots.push_back(name);
        }
        return robots;
    }
    
    /**
     * @brief Create robot configuration
     */
    std::any createRobotConfig(
        const std::string& robot_name,
        std::vector<float> start_config,
        std::vector<float> goal_config) const {
        
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = handlers_.find(robot_name);
        if (it == handlers_.end()) {
            throw VampConfigurationError("Robot not registered: " + robot_name);
        }
        
        if (!it->second->validateConfigurationSize(start_config)) {
            throw VampConfigurationError("Invalid start configuration size for robot: " + robot_name);
        }
        
        if (!it->second->validateConfigurationSize(goal_config)) {
            throw VampConfigurationError("Invalid goal configuration size for robot: " + robot_name);
        }
        
        return it->second->createRobotConfig(robot_name, std::move(start_config), std::move(goal_config));
    }
    
    /**
     * @brief Create planner
     */
    std::any createPlanner(
        const std::string& robot_name,
        std::any robot_config,
        std::unique_ptr<EnvironmentFactory> env_factory) const {
        
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = handlers_.find(robot_name);
        if (it == handlers_.end()) {
            throw VampConfigurationError("Robot not registered: " + robot_name);
        }
        
        return it->second->createPlanner(std::move(robot_config), std::move(env_factory));
    }
    
    /**
     * @brief Initialize planner
     */
    void initializePlanner(const std::string& robot_name, std::any& planner) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = handlers_.find(robot_name);
        if (it == handlers_.end()) {
            throw VampConfigurationError("Robot not registered: " + robot_name);
        }
        
        it->second->initializePlanner(planner);
    }
    
    /**
     * @brief Execute planning
     */
    PlanningResult executePlanning(
        const std::string& robot_name,
        std::any& planner,
        const PlanningConfig& planning_config) const {
        
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = handlers_.find(robot_name);
        if (it == handlers_.end()) {
            throw VampConfigurationError("Robot not registered: " + robot_name);
        }
        
        return it->second->executePlanning(planner, planning_config);
    }
    
    /**
     * @brief Get joint limits for robot
     */
    std::vector<std::pair<double, double>> getJointLimits(const std::string& robot_name) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = handlers_.find(robot_name);
        if (it == handlers_.end()) {
            throw VampConfigurationError("Robot not registered: " + robot_name);
        }
        
        return it->second->getJointLimits();
    }
    
    // ========== BENCHMARKING EXTENSIONS ==========
    // Following the same patterns as above, but for benchmarking
    
    /**
     * @brief Create benchmark manager for specified robot (type-erased)
     * @param robot_name Robot identifier  
     * @param start_config Start configuration
     * @param goal_config Goal configuration
     * @param env_factory Environment factory
     * @return Type-erased benchmark manager
     * @throws VampConfigurationError if robot not found
     * 
     * This method extends the registry pattern to support benchmarking
     * while maintaining the same design principles and error handling.
     */
    std::any createBenchmarkManager(const std::string& robot_name,
                                   const std::vector<float>& start_config,
                                   const std::vector<float>& goal_config,
                                   std::unique_ptr<EnvironmentFactory> env_factory) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = handlers_.find(robot_name);
        if (it == handlers_.end()) {
            throw VampConfigurationError("Robot not registered: " + robot_name);
        }
        
        return it->second->createBenchmarkManager(start_config, goal_config, std::move(env_factory));
    }
    
    /**
     * @brief Execute benchmark with type erasure (following existing patterns)
     * @param robot_name Robot identifier
     * @param benchmark_manager Type-erased benchmark manager  
     * @param experiment_name Name for the experiment
     * @param planner_names List of planners to benchmark
     * @param runs Number of runs per planner
     * @param timeout Time limit per run
     * @return Benchmark results as string map for type erasure
     * @throws VampConfigurationError if robot not found
     */
    std::map<std::string, std::string> executeBenchmark(
        const std::string& robot_name,
        std::any& benchmark_manager,
        const std::string& experiment_name = "VAMP Benchmark",
        const std::vector<std::string>& planner_names = {"RRT-Connect", "BIT*"},
        unsigned int runs = 50,
        double timeout = 5.0) const {
        
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = handlers_.find(robot_name);
        if (it == handlers_.end()) {
            throw VampConfigurationError("Robot not registered: " + robot_name);
        }
        
        return it->second->executeBenchmark(
            benchmark_manager, experiment_name, planner_names, runs, timeout);
    }
    
    /**
     * @brief Check if benchmarking is available for robot
     * @param robot_name Robot identifier
     * @return true if benchmarking supported, false otherwise
     */
    bool isBenchmarkingAvailable(const std::string& robot_name) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = handlers_.find(robot_name);
        return it != handlers_.end(); // All registered robots support benchmarking
    }
    
    /**
     * @brief Get list of robots supporting benchmarking
     * @return Vector of robot names with benchmarking support
     */
    std::vector<std::string> getBenchmarkingEnabledRobots() const {
        return getRegisteredRobots(); // All robots support benchmarking
    }

private:
    RobotRegistry() {
        // Register built-in robots
        registerBuiltInRobots();
    }
    
    void registerBuiltInRobots() {
        // Register VAMP built-in robots
        registerRobot<vamp::robots::Panda>("panda");
        registerRobot<vamp::robots::UR5>("ur5");
        registerRobot<vamp::robots::Fetch>("fetch");
    }
    
    mutable std::mutex mutex_;
    std::unordered_map<std::string, std::unique_ptr<RobotHandler>> handlers_;
    std::unordered_map<std::string, RobotMetadata> robot_metadata_;
};

/**
 * @brief RAII robot registrar for automatic registration
 */
template<typename Robot>
class RobotRegistrar {
public:
    explicit RobotRegistrar(const std::string& name) {
        RobotRegistry::getInstance().registerRobot<Robot>(name);
    }
};

/**
 * @brief Macro for easy robot registration
 */
#define REGISTER_VAMP_ROBOT(RobotType, name) \
    namespace { \
        static vamp_ompl::RobotRegistrar<RobotType> registrar_##RobotType(name); \
    }

} // namespace vamp_ompl 