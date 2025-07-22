#include <vector>
#include <array>
#include <utility>
#include <iostream>
#include <chrono>
#include <memory>
#include <string>
#include <typeinfo>
#include <cxxabi.h>

#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>
#include <vamp/robots/panda.hh>
#include <vamp/robots/ur5.hh>
#include <vamp/robots/fetch.hh>

#include <ompl/base/MotionValidator.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/util/Exception.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Robot selection enum
enum class RobotType { PANDA, UR5, FETCH };

// Environment types
enum class EnvironmentType { SPHERE_CAGE, TABLE_SCENE, EMPTY };

// Planner types
enum class PlannerType { BITSTAR, RRTCONNECT, PRM };

// Template for different robot types
template<typename Robot>
class VampOMPLIntegration {
private:
    static constexpr std::size_t dimension = Robot::dimension;
    using Configuration = typename Robot::Configuration;
    static constexpr const std::size_t rake = vamp::FloatVectorWidth;
    using EnvironmentInput = vamp::collision::Environment<float>;
    using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

    RobotType robot_type_;
    EnvironmentType env_type_;
    PlannerType planner_type_;
    std::shared_ptr<ob::SpaceInformation> si_;
    std::shared_ptr<ob::ProblemDefinition> pdef_;
    EnvironmentVector env_v_;

    // Convert an OMPL state into a VAMP vector
    inline static auto ompl_to_vamp(const ob::State *state) -> Configuration
    {
        std::array<typename Configuration::S::ScalarT, Configuration::num_scalars> aligned_buffer;

        auto *as = state->as<ob::RealVectorStateSpace::StateType>();
        for (auto i = 0U; i < dimension; ++i)
        {
            aligned_buffer[i] = static_cast<float>(as->values[i]);
        }

        return Configuration(aligned_buffer.data());
    }

    // Convert a VAMP vector to an OMPL state
    inline static auto vamp_to_ompl(const Configuration &c, ob::State *state)
    {
        auto *as = state->as<ob::RealVectorStateSpace::StateType>();
        for (auto i = 0U; i < dimension; ++i)
        {
            as->values[i] = static_cast<double>(c[{i, 0}]);
        }
    }

    // State validator using VAMP
    struct VAMPStateValidator : public ob::StateValidityChecker
    {
        VAMPStateValidator(ob::SpaceInformation *si, const EnvironmentVector &env_v)
          : ob::StateValidityChecker(si), env_v(env_v)
        {
        }

        VAMPStateValidator(const ob::SpaceInformationPtr &si, const EnvironmentVector &env_v)
          : ob::StateValidityChecker(si), env_v(env_v)
        {
        }

        auto isValid(const ob::State *state) const -> bool override
        {
            auto configuration = VampOMPLIntegration<Robot>::ompl_to_vamp(state);
            return vamp::planning::validate_motion<Robot, rake, 1>(configuration, configuration, env_v);
        }

        const EnvironmentVector &env_v;
    };

    // Motion validator using VAMP
    struct VAMPMotionValidator : public ob::MotionValidator
    {
        VAMPMotionValidator(ob::SpaceInformation *si, const EnvironmentVector &env_v)
          : ob::MotionValidator(si), env_v(env_v)
        {
        }

        VAMPMotionValidator(const ob::SpaceInformationPtr &si, const EnvironmentVector &env_v)
          : ob::MotionValidator(si), env_v(env_v)
        {
        }

        auto checkMotion(const ob::State *s1, const ob::State *s2) const -> bool override
        {
            return vamp::planning::validate_motion<Robot, rake, Robot::resolution>(
                VampOMPLIntegration<Robot>::ompl_to_vamp(s1), VampOMPLIntegration<Robot>::ompl_to_vamp(s2), env_v);
        }

        auto checkMotion(const ob::State *, const ob::State *, std::pair<ob::State *, double> &) const
            -> bool override
        {
            throw ompl::Exception("Not implemented!");
        }

        const EnvironmentVector &env_v;
    };

    // Create sphere cage environment
    void createSphereCageEnvironment()
    {
        EnvironmentInput environment;
        
        // Create a cage of spheres around the robot
        std::vector<std::array<float, 3>> sphere_positions;
        
        if (robot_type_ == RobotType::PANDA) {
            // Sphere cage for Panda
            sphere_positions = {
                {0.55, 0, 0.25}, {0.35, 0.35, 0.25}, {0, 0.55, 0.25},
                {-0.55, 0, 0.25}, {-0.35, -0.35, 0.25}, {0, -0.55, 0.25},
                {0.35, -0.35, 0.25}, {0.35, 0.35, 0.8}, {0, 0.55, 0.8},
                {-0.35, 0.35, 0.8}, {-0.55, 0, 0.8}, {-0.35, -0.35, 0.8},
                {0, -0.55, 0.8}, {0.35, -0.35, 0.8}
            };
        } else if (robot_type_ == RobotType::UR5) {
            // Sphere cage for UR5
            sphere_positions = {
                {0.4, 0, 0.3}, {0.3, 0.3, 0.3}, {0, 0.4, 0.3},
                {-0.4, 0, 0.3}, {-0.3, -0.3, 0.3}, {0, -0.4, 0.3},
                {0.3, -0.3, 0.3}, {0.3, 0.3, 0.7}, {0, 0.4, 0.7},
                {-0.3, 0.3, 0.7}, {-0.4, 0, 0.7}, {-0.3, -0.3, 0.7},
                {0, -0.4, 0.7}, {0.3, -0.3, 0.7}
            };
        } else { // FETCH
            // Sphere cage for Fetch
            sphere_positions = {
                {0.6, 0, 0.4}, {0.4, 0.4, 0.4}, {0, 0.6, 0.4},
                {-0.6, 0, 0.4}, {-0.4, -0.4, 0.4}, {0, -0.6, 0.4},
                {0.4, -0.4, 0.4}, {0.4, 0.4, 1.0}, {0, 0.6, 1.0},
                {-0.4, 0.4, 1.0}, {-0.6, 0, 1.0}, {-0.4, -0.4, 1.0},
                {0, -0.6, 1.0}, {0.4, -0.4, 1.0}
            };
        }

        const float radius = 0.15;
        for (const auto &sphere : sphere_positions)
        {
            environment.spheres.emplace_back(vamp::collision::factory::sphere::array(sphere, radius));
        }

        environment.sort();
        env_v_ = EnvironmentVector(environment);
    }

    // Create table scene environment
    void createTableSceneEnvironment()
    {
        EnvironmentInput environment;
        
        // Create a table with some obstacles
        std::vector<std::array<float, 3>> box_positions;
        
        if (robot_type_ == RobotType::PANDA) {
            // Table scene for Panda - table at waist height to avoid base collision
            box_positions = {
                // Table surface (1m x 1m table at Z=0.8m height)
                {-0.5, -0.5, 0.8}, {0.5, 0.5, 0.82},
                // Obstacles on table surface
                {0.2, 0.1, 0.82}, {0.3, 0.2, 0.92},
                {-0.2, 0.2, 0.82}, {-0.1, 0.3, 0.92},
                {0.0, -0.2, 0.82}, {0.1, -0.1, 0.92}
            };
        } else if (robot_type_ == RobotType::UR5) {
            // Table scene for UR5
            box_positions = {
                // Table surface
                {0.0, 0.0, 0.0}, {0.0, 0.0, 0.02},
                // Obstacles on table
                {0.25, 0.15, 0.08}, {0.25, 0.15, 0.16},
                {-0.15, 0.25, 0.08}, {-0.15, 0.25, 0.16},
                {0.08, -0.2, 0.08}, {0.08, -0.2, 0.16}
            };
        } else { // FETCH
            // Table scene for Fetch
            box_positions = {
                // Table surface
                {0.0, 0.0, 0.0}, {0.0, 0.0, 0.02},
                // Obstacles on table
                {0.4, 0.25, 0.12}, {0.4, 0.25, 0.24},
                {-0.25, 0.4, 0.12}, {-0.25, 0.4, 0.24},
                {0.15, -0.3, 0.12}, {0.15, -0.3, 0.24}
            };
        }

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
        env_v_ = EnvironmentVector(environment);
    }

    // Create empty environment
    void createEmptyEnvironment()
    {
        EnvironmentInput environment;
        environment.sort();
        env_v_ = EnvironmentVector(environment);
    }

    // Setup OMPL state space and bounds
    void setupStateSpace()
    {
        auto space = std::make_shared<ob::RealVectorStateSpace>(dimension);
        ob::RealVectorBounds bounds(dimension);
        
        // Use hardcoded bounds based on robot type to avoid Configuration issues
        if (robot_type_ == RobotType::PANDA) {
            // Panda joint limits from URDF
            std::vector<std::pair<double, double>> panda_limits = {
                {-2.9671, 2.9671},   // joint1
                {-1.8326, 1.8326},   // joint2  
                {-2.9671, 2.9671},   // joint3
                {-3.1416, 0.0873},   // joint4
                {-2.9671, 2.9671},   // joint5
                {-0.0873, 3.8223},   // joint6
                {-2.9671, 2.9671}    // joint7
            };
            for (auto i = 0U; i < dimension; ++i) {
                bounds.setLow(i, panda_limits[i].first);
                bounds.setHigh(i, panda_limits[i].second);
            }
        } else if (robot_type_ == RobotType::UR5) {
            // UR5 joint limits
            for (auto i = 0U; i < dimension; ++i) {
                bounds.setLow(i, -3.14159265);
                bounds.setHigh(i, 3.14159265);
            }
        } else { // FETCH
            // Fetch joint limits - mixed joint types
            std::vector<std::pair<double, double>> fetch_limits = {
                {0.0, 0.38615},        // torso_lift_joint (prismatic)
                {-1.6056, 1.6056},     // shoulder_pan_joint
                {-1.221, 1.518},       // shoulder_lift_joint
                {-3.14159, 3.14159},   // upperarm_roll_joint
                {-2.251, 2.251},       // elbow_flex_joint
                {-3.14159, 3.14159},   // forearm_roll_joint
                {-2.16, 2.16},         // wrist_flex_joint
                {-3.14159, 3.14159}    // wrist_roll_joint
            };
            for (auto i = 0U; i < dimension; ++i) {
                bounds.setLow(i, fetch_limits[i].first);
                bounds.setHigh(i, fetch_limits[i].second);
            }
        }
        

        space->setBounds(bounds);
        // Create space information
        si_ = std::make_shared<ob::SpaceInformation>(space);
        si_->setStateValidityChecker(std::make_shared<VAMPStateValidator>(si_, env_v_));
        si_->setMotionValidator(std::make_shared<VAMPMotionValidator>(si_, env_v_));
        si_->setup();
    }

    // Get robot-specific start and goal configurations
    std::pair<std::array<float, dimension>, std::array<float, dimension>> getStartGoalConfigurations()
    {
        if (robot_type_ == RobotType::PANDA) {
            if (env_type_ == EnvironmentType::TABLE_SCENE) {
                // Use two validated SRDF poses for table scene
                static constexpr std::array<float, 7> start = {0., -0.785, 0., -2.356, 0., 1.571, 0.785}; // "ready" pose
                static constexpr std::array<float, 7> goal = {0., -0.5599, 0., -2.97, 0., 0., 0.785}; // "transport" pose
                
                std::array<float, dimension> start_padded{}, goal_padded{};
                std::copy(start.begin(), start.end(), start_padded.begin());
                std::copy(goal.begin(), goal.end(), goal_padded.begin());
                
                return {start_padded, goal_padded};
            } else {
                // Original start/goal for sphere cage and other environments
                static constexpr std::array<float, 7> start = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
                static constexpr std::array<float, 7> goal = {2.35, 1., 0., -0.8, 0, 2.5, 0.785};
                
                std::array<float, dimension> start_padded{}, goal_padded{};
                std::copy(start.begin(), start.end(), start_padded.begin());
                std::copy(goal.begin(), goal.end(), goal_padded.begin());
                
                return {start_padded, goal_padded};
            }
        } else if (robot_type_ == RobotType::UR5) {
            static constexpr std::array<float, 6> start = {0., -1.57, 0., -1.57, 0., 0.};
            static constexpr std::array<float, 6> goal = {1.57, -0.785, 0., -2.356, 0., 1.57};
            
            std::array<float, dimension> start_padded{}, goal_padded{};
            std::copy(start.begin(), start.end(), start_padded.begin());
            std::copy(goal.begin(), goal.end(), goal_padded.begin());
            
            return {start_padded, goal_padded};
        } else { // FETCH
            static constexpr std::array<float, 8> start = {0., 0., 0., 0., 0., 0., 0., 0.};
            static constexpr std::array<float, 8> goal = {0.1, 1.57, 0.785, 0., -1.57, 0., 0., 0.};
            
            std::array<float, dimension> start_padded{}, goal_padded{};
            std::copy(start.begin(), start.end(), start_padded.begin());
            std::copy(goal.begin(), goal.end(), goal_padded.begin());
            
            return {start_padded, goal_padded};
        }
    }

    // Setup problem definition
    void setupProblemDefinition()
    {
        auto [start_config, goal_config] = getStartGoalConfigurations();
        // Bounds check for start/goal arrays
        if (start_config.size() != dimension || goal_config.size() != dimension) {
            std::cerr << "[ERROR] Start/goal array size does not match robot dimension!\n";
            std::cerr << "[ERROR] start_config.size() = " << start_config.size() << ", goal_config.size() = " << goal_config.size() << ", dimension = " << dimension << std::endl;
            throw std::runtime_error("Start/goal array size mismatch");
        }
        auto space = si_->getStateSpace();
        ob::ScopedState<> start_ompl(space), goal_ompl(space);
        
        for (auto v : start_config) std::cout << v << " ";
        std::cout << std::endl;

        for (auto v : goal_config) std::cout << v << " ";
        std::cout << std::endl;

        for (auto i = 0U; i < dimension; ++i)
        {
            start_ompl[i] = start_config[i];
            goal_ompl[i] = goal_config[i];
        }

        pdef_ = std::make_shared<ob::ProblemDefinition>(si_);
        pdef_->setStartAndGoalStates(start_ompl, goal_ompl);

        // Set optimization objective
        auto obj = std::make_shared<ob::PathLengthOptimizationObjective>(si_);
        pdef_->setOptimizationObjective(obj);
    }

    // Create planner based on type
    std::shared_ptr<ob::Planner> createPlanner()
    {
        switch (planner_type_) {
            case PlannerType::BITSTAR:
                return std::make_shared<og::BITstar>(si_);
            case PlannerType::RRTCONNECT:
                return std::make_shared<og::RRTConnect>(si_);
            case PlannerType::PRM:
                return std::make_shared<og::PRM>(si_);
            default:
                return std::make_shared<og::BITstar>(si_);
        }
    }

public:
    VampOMPLIntegration(RobotType robot_type, EnvironmentType env_type, PlannerType planner_type)
        : robot_type_(robot_type), env_type_(env_type), planner_type_(planner_type)
    {
        switch (robot_type_) {
            case RobotType::PANDA: std::cout << "PANDA"; break;
            case RobotType::UR5: std::cout << "UR5"; break;
            case RobotType::FETCH: std::cout << "FETCH"; break;
        }
        std::cout << std::endl;
        // Print the C++ type of the Robot template parameter
        // int status;
        // char* realname = abi::__cxa_demangle(typeid(Robot).name(), 0, 0, &status);
        // if (realname) free(realname);

        // Create environment based on type
        switch (env_type_) {
            case EnvironmentType::SPHERE_CAGE:
                createSphereCageEnvironment();
                break;
            case EnvironmentType::TABLE_SCENE:
                createTableSceneEnvironment();
                break;
            case EnvironmentType::EMPTY:
                createEmptyEnvironment();
                break;
        }

        setupStateSpace();
        setupProblemDefinition();
    }

    // Run the planning demo
    bool runDemo(double planning_time = 5.0, double simplification_time = 1.0, bool optimize = false)
    {
        // Print C++ type and dimension again for confirmation
        // int status;
        // char* realname = abi::__cxa_demangle(typeid(Robot).name(), 0, 0, &status);
        // if (realname) free(realname);
        std::cout << "\n=== VAMP + OMPL Integration Demo ===" << std::endl;
        std::cout << "Robot: ";
        switch (robot_type_) {
            case RobotType::PANDA: std::cout << "Franka Emika Panda"; break;
            case RobotType::UR5: std::cout << "UR5"; break;
            case RobotType::FETCH: std::cout << "Fetch"; break;
        }
        std::cout << std::endl;
        
        std::cout << "Environment: ";
        switch (env_type_) {
            case EnvironmentType::SPHERE_CAGE: std::cout << "Sphere Cage"; break;
            case EnvironmentType::TABLE_SCENE: std::cout << "Table Scene"; break;
            case EnvironmentType::EMPTY: std::cout << "Empty"; break;
        }
        std::cout << std::endl;
        
        std::cout << "Planner: ";
        switch (planner_type_) {
            case PlannerType::BITSTAR: std::cout << "BIT*"; break;
            case PlannerType::RRTCONNECT: std::cout << "RRT-Connect"; break;
            case PlannerType::PRM: std::cout << "PRM"; break;
        }
        std::cout << std::endl;
        std::cout << "Planning time: " << planning_time << "s" << std::endl;
        std::cout << "Optimize: " << (optimize ? "Yes" : "No") << std::endl;

        // Create planner
        auto planner = createPlanner();
        planner->setProblemDefinition(pdef_);
        planner->setup();

        // Set optimization threshold if not optimizing
        if (!optimize) {
            auto obj = pdef_->getOptimizationObjective();
            obj->setCostThreshold(obj->infiniteCost());
        }

        // Solve the problem
        auto start_time = std::chrono::steady_clock::now();
        ob::PlannerStatus solved = planner->solve(planning_time);
        auto end_time = std::chrono::steady_clock::now();
        auto planning_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

        std::cout << "\nPlanning completed in " << planning_duration.count() << "ms" << std::endl;

        // Check solution
        if (solved == ob::PlannerStatus::EXACT_SOLUTION)
        {
            std::cout << "✓ Found solution! Simplifying path..." << std::endl;

            // Simplify the path
            const ob::PathPtr &path = pdef_->getSolutionPath();
            og::PathGeometric &path_geometric = static_cast<og::PathGeometric &>(*path);

            auto obj = pdef_->getOptimizationObjective();
            auto initial_cost = path_geometric.cost(obj);

            og::PathSimplifier simplifier(si_, pdef_->getGoal(), obj);
            auto simplify_start = std::chrono::steady_clock::now();
            bool simplified = simplifier.simplify(path_geometric, simplification_time);
            auto simplify_end = std::chrono::steady_clock::now();
            auto simplify_duration = std::chrono::duration_cast<std::chrono::milliseconds>(simplify_end - simplify_start);

            if (simplified) {
                auto simplified_cost = path_geometric.cost(obj);
                std::cout << "✓ Path simplified in " << simplify_duration.count() << "ms" << std::endl;
                std::cout << "Initial cost: " << initial_cost.value() << std::endl;
                std::cout << "Simplified cost: " << simplified_cost.value() << std::endl;
                std::cout << "Path length: " << path_geometric.getStateCount() << " states" << std::endl;
            } else {
                std::cout << "✗ Path simplification failed" << std::endl;
            }

            return true;
        }
        else
        {
            std::cout << "✗ No solution found" << std::endl;
            return false;
        }
    }

    // Get solution path for visualization
    const ob::PathPtr& getSolutionPath() const
    {
        return pdef_->getSolutionPath();
    }
};

// Forward declarations for separate robot functions
bool runPandaDemo(EnvironmentType env_type, PlannerType planner_type, const std::string& description);
bool runUR5Demo(EnvironmentType env_type, PlannerType planner_type, const std::string& description);
bool runFetchDemo(EnvironmentType env_type, PlannerType planner_type, const std::string& description);

// Main demo function
void runVampDemo()
{
    std::cout << "VAMP + OMPL Integration Demo" << std::endl;
    std::cout << "============================" << std::endl;
    std::cout << "This demo showcases the integration of VAMP (Vector-Accelerated Motion Planning)" << std::endl;
    std::cout << "with OMPL for high-performance motion planning." << std::endl;
    std::cout << std::endl;

    // Demo configurations
    std::vector<std::tuple<RobotType, EnvironmentType, PlannerType, std::string>> demos = {
        {RobotType::PANDA, EnvironmentType::SPHERE_CAGE, PlannerType::BITSTAR, "Panda in Sphere Cage with BIT*"},
        {RobotType::PANDA, EnvironmentType::TABLE_SCENE, PlannerType::RRTCONNECT, "Panda in Table Scene with RRT-Connect"},
        {RobotType::UR5, EnvironmentType::SPHERE_CAGE, PlannerType::PRM, "UR5 in Sphere Cage with PRM"},
        {RobotType::FETCH, EnvironmentType::EMPTY, PlannerType::BITSTAR, "Fetch in Empty Environment with BIT*"}
    };

    int success_count = 0;
    int total_count = 0;

    for (const auto& [robot_type, env_type, planner_type, description] : demos) {
        std::cout << "\n" << std::string(60, '-') << std::endl;
        std::cout << "Demo: " << description << std::endl;
        std::cout << std::string(60, '-') << std::endl;
        
        try {
            bool success = false;
            switch (robot_type) {
                case RobotType::PANDA:
                    success = runPandaDemo(env_type, planner_type, description);
                    break;
                case RobotType::UR5:
                    success = runUR5Demo(env_type, planner_type, description);
                    break;
                case RobotType::FETCH:
                    success = runFetchDemo(env_type, planner_type, description);
                    break;
            }
            if (success) success_count++;
            total_count++;
        } catch (const std::exception& e) {
            std::cout << "✗ Error: " << e.what() << std::endl;
            total_count++;
        }
    }

    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "Demo Summary" << std::endl;
    std::cout << std::string(60, '=') << std::endl;
    std::cout << "Successful demos: " << success_count << "/" << total_count << std::endl;
    std::cout << "Success rate: " << (100.0 * success_count / total_count) << "%" << std::endl;
    std::cout << std::endl;
}

// Separate robot demo functions
bool runPandaDemo(EnvironmentType env_type, PlannerType planner_type, const std::string& description)
{
    std::cout << "Running Panda demo: " << description << std::endl;
    VampOMPLIntegration<vamp::robots::Panda> demo(RobotType::PANDA, env_type, planner_type);
    return demo.runDemo(3.0, 0.5, false);
}

bool runUR5Demo(EnvironmentType env_type, PlannerType planner_type, const std::string& description)
{
    std::cout << "Running UR5 demo: " << description << std::endl;
    VampOMPLIntegration<vamp::robots::UR5> demo(RobotType::UR5, env_type, planner_type);
    return demo.runDemo(3.0, 0.5, false);
}

bool runFetchDemo(EnvironmentType env_type, PlannerType planner_type, const std::string& description)
{
    std::cout << "Running Fetch demo: " << description << std::endl;
    VampOMPLIntegration<vamp::robots::Fetch> demo(RobotType::FETCH, env_type, planner_type);
    return demo.runDemo(3.0, 0.5, false);
}

int main(int argc, char **argv)
{
    try {
        runVampDemo();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
} 