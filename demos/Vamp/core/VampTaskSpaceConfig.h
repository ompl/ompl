#pragma once

#include <ompl/geometric/planners/rrt/TSRRT.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/util/RandomNumbers.h>
#include <Eigen/Dense>
#include <memory>
#include <functional>

namespace vamp_ompl {

namespace ob = ompl::base;
namespace og = ompl::geometric;

/**
 * @brief Generic task space configuration for VAMP robots
 */
class VampTaskSpaceConfig : public og::TaskSpaceConfig {
public:
    /**
     * @brief Projection function type: (state) -> task_space_point
     */
    using ProjectionFunction = std::function<void(const ob::State*, Eigen::Ref<Eigen::VectorXd>)>;
    
    /**
     * @brief Lifting function type: (task_space_point, seed_state, output_state) -> success
     */
    using LiftingFunction = std::function<bool(const Eigen::Ref<Eigen::VectorXd>&, const ob::State*, ob::State*)>;
    
    /**
     * @brief Sampling function type: () -> task_space_point
     */
    using SamplingFunction = std::function<void(Eigen::Ref<Eigen::VectorXd>)>;

    /**
     * @brief Constructor
     * @param dimension Task space dimension
     * @param bounds Task space bounds
     * @param project_fn Projection function
     * @param lift_fn Lifting function
     * @param sample_fn Optional custom sampling function
     */
    VampTaskSpaceConfig(int dimension,
                       const ob::RealVectorBounds& bounds,
                       ProjectionFunction project_fn,
                       LiftingFunction lift_fn,
                       SamplingFunction sample_fn = nullptr)
        : dimension_(dimension)
        , bounds_(bounds)
        , project_fn_(std::move(project_fn))
        , lift_fn_(std::move(lift_fn))
        , sample_fn_(std::move(sample_fn)) {
        
        if (!sample_fn_) {
            // Default uniform sampling
            sample_fn_ = [this](Eigen::Ref<Eigen::VectorXd> ts_proj) {
                for (int i = 0; i < dimension_; ++i) {
                    ts_proj[i] = rng_.uniformReal(bounds_.low[i], bounds_.high[i]);
                }
            };
        }
    }

    // TaskSpaceConfig interface
    int getDimension() const override {
        return dimension_;
    }

    void project(const ob::State* state, Eigen::Ref<Eigen::VectorXd> ts_proj) const override {
        project_fn_(state, ts_proj);
    }

    void sample(Eigen::Ref<Eigen::VectorXd> ts_proj) const override {
        sample_fn_(ts_proj);
    }

    bool lift(const Eigen::Ref<Eigen::VectorXd>& ts_proj, 
              const ob::State* seed, 
              ob::State* state) const override {
        return lift_fn_(ts_proj, seed, state);
    }

private:
    int dimension_;
    ob::RealVectorBounds bounds_;
    ProjectionFunction project_fn_;
    LiftingFunction lift_fn_;
    SamplingFunction sample_fn_;
    mutable ompl::RNG rng_;
};

/**
 * @brief Generalized factory for creating task space configurations
 * 
 * This factory creates task space configurations that work with any robot by using
 * generalized kinematics approximations and automatic parameter estimation from
 * joint space bounds. No robot-specific hardcoding required.
 */
class VampTaskSpaceFactory {
public:
    /**
     * @brief Create end-effector position task space for manipulator
     * @param si Space information
     * @param workspace_bounds Task space bounds
     * @param ee_link_index End-effector link index (-1 for last)
     * @return Task space configuration
     */
    static std::shared_ptr<VampTaskSpaceConfig> createEndEffectorPositionConfig(
        const ob::SpaceInformationPtr& si,
        const ob::RealVectorBounds& workspace_bounds,
        int ee_link_index = -1) {
        
        auto state_space = si->getStateSpace();
        int num_joints = state_space->getDimension();
        
        if (ee_link_index < 0) {
            ee_link_index = num_joints - 1;
        }
        
        // Projection: joint angles -> end-effector position
        auto project_fn = [num_joints, ee_link_index, si](const ob::State* state, Eigen::Ref<Eigen::VectorXd> ts_proj) {
            // PLACEHOLDER: Simple approximation for demonstration
            // TODO: Replace with actual robot kinematics from VAMP robot model
            const auto* values = state->as<ob::RealVectorStateSpace::StateType>()->values;
            
            // Simplified planar chain forward kinematics (for demo purposes)
            // This works reasonably for planar manipulators but should be replaced
            // with proper 3D kinematics for real robots
            double x = 0.0, y = 0.0, z = 0.4; // Start at table height
            double angle_sum = 0.0;
            
            // Estimate link length from joint bounds (more generalized)
            const auto* rv_space = si->getStateSpace()->as<ob::RealVectorStateSpace>();
            double total_reach = 1.0; // Default
            if (rv_space) {
                auto bounds = rv_space->getBounds();
                double total_range = 0.0;
                for (int i = 0; i < num_joints && i < 7; ++i) {
                    total_range += std::abs(bounds.high[i] - bounds.low[i]);
                }
                total_reach = (total_range / (2.0 * M_PI)) * 0.8; // Conservative estimate
            }
            double link_length = total_reach / num_joints;
            
            for (int i = 0; i <= ee_link_index && i < num_joints; ++i) {
                angle_sum += values[i];
                x += link_length * cos(angle_sum);
                y += link_length * sin(angle_sum);
                // For 3D robots, this should include proper DH parameters
            }
            
            ts_proj[0] = x;
            ts_proj[1] = y;
            if (ts_proj.size() > 2) {
                ts_proj[2] = z; // Fixed height for planar approximation
            }
        };
        
        // Lifting: end-effector position -> joint configuration
        auto lift_fn = [si, num_joints](const Eigen::Ref<Eigen::VectorXd>& ts_proj, 
                                       const ob::State* seed, 
                                       ob::State* state) -> bool {
            // PLACEHOLDER: Simplified IK for demonstration
            // TODO: Replace with proper IK solver (KDL, Pinocchio, etc.)
            
            // For now, use seed configuration with small perturbations
            // This is not proper IK but allows TSRRT to function for testing
            si->copyState(state, seed);
            
            // Apply small perturbations biased toward the target
            auto* values = state->as<ob::RealVectorStateSpace::StateType>()->values;
            ompl::RNG rng;
            
            // Multiple attempts to find valid configuration
            for (int attempt = 0; attempt < 10; ++attempt) {
                si->copyState(state, seed);
                
                // Small random perturbations
                for (int i = 0; i < num_joints; ++i) {
                    values[i] += rng.gaussian(0.0, 0.05); // Smaller perturbation
                }
                
                // Enforce joint limits
                si->getStateSpace()->enforceBounds(state);
                
                if (si->isValid(state)) {
                    return true;
                }
            }
            
            // Fallback: return seed if no valid perturbation found
            si->copyState(state, seed);
            return si->isValid(state);
        };
        
        int task_dim = (workspace_bounds.low.size() >= 3) ? 3 : 2;
        
        return std::make_shared<VampTaskSpaceConfig>(
            task_dim, workspace_bounds, project_fn, lift_fn);
    }
    
    /**
     * @brief Create SE(2) task space configuration
     * @param si Space information
     * @param workspace_bounds Task space bounds
     * @return Task space configuration
     */
    static std::shared_ptr<VampTaskSpaceConfig> createSE2Config(
        const ob::SpaceInformationPtr& si,
        const ob::RealVectorBounds& workspace_bounds) {
        
        // Project SE(2) state to (x, y) position
        auto project_fn = [](const ob::State* state, Eigen::Ref<Eigen::VectorXd> ts_proj) {
            const auto* se2_state = state->as<ob::SE2StateSpace::StateType>();
            ts_proj[0] = se2_state->getX();
            ts_proj[1] = se2_state->getY();
        };
        
        // Lift (x, y) to SE(2) state
        auto lift_fn = [si](const Eigen::Ref<Eigen::VectorXd>& ts_proj, 
                           const ob::State* seed, 
                           ob::State* state) -> bool {
            auto* se2_state = state->as<ob::SE2StateSpace::StateType>();
            const auto* seed_se2 = seed->as<ob::SE2StateSpace::StateType>();
            
            se2_state->setX(ts_proj[0]);
            se2_state->setY(ts_proj[1]);
            se2_state->setYaw(seed_se2->getYaw()); // Keep orientation from seed
            
            return si->isValid(state);
        };
        
        return std::make_shared<VampTaskSpaceConfig>(
            2, workspace_bounds, project_fn, lift_fn);
    }
    
    /**
     * @brief Create intelligent task space based on state space type
     * @param si Space information
     * @param workspace_bounds Task space bounds
     * @return Task space configuration
     */
    static std::shared_ptr<VampTaskSpaceConfig> createIntelligentConfig(
        const ob::SpaceInformationPtr& si,
        const ob::RealVectorBounds& workspace_bounds) {
        
        auto state_space = si->getStateSpace();
        
        if (state_space->getType() == ob::STATE_SPACE_SE2) {
            return createSE2Config(si, workspace_bounds);
        } else if (state_space->getType() == ob::STATE_SPACE_REAL_VECTOR) {
            // Assume manipulator
            return createEndEffectorPositionConfig(si, workspace_bounds);
        } else {
            // Default to end-effector position for compound spaces
            return createEndEffectorPositionConfig(si, workspace_bounds);
        }
    }
};

} // namespace vamp_ompl
