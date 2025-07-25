#pragma once

#include "VampOMPLInterfaces.h"
#include <vamp/planning/validate.hh>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/util/Exception.h>

namespace vamp_ompl {

/**
 * @brief State validity checker using VAMP collision detection
 * 
 * This class is focused solely on checking whether individual states
 * are valid (collision-free) using VAMP's collision detection.
 */
template<typename Robot>
class VampStateValidator : public ob::StateValidityChecker {
public:
    using Configuration = typename Robot::Configuration;
    static constexpr std::size_t dimension = Robot::dimension;
    static constexpr std::size_t rake = vamp::FloatVectorWidth;
    using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

    /**
     * @brief Constructor taking space information and vectorized environment
     * @param si Space information pointer
     * @param env_v Vectorized VAMP environment for collision checking
     */
    VampStateValidator(const ob::SpaceInformationPtr &si, const EnvironmentVector &env_v)
        : ob::StateValidityChecker(si), env_v_(env_v)
    {
    }

    /**
     * @brief Check if a state is valid (collision-free)
     * @param state The state to check
     * @return true if state is valid, false otherwise
     */
    bool isValid(const ob::State *state) const override
    {
        auto configuration = ompl_to_vamp(state);
        // Use VAMP validation with single configuration (start == end for point check)
        return vamp::planning::validate_motion<Robot, rake, 1>(configuration, configuration, env_v_);
    }

private:
    const EnvironmentVector &env_v_;

    /**
     * @brief Convert OMPL state to VAMP configuration
     * @param state OMPL state
     * @return VAMP configuration
     */
    static Configuration ompl_to_vamp(const ob::State *state)
    {
        std::array<typename Configuration::S::ScalarT, Configuration::num_scalars> aligned_buffer;

        auto *as = state->as<ob::RealVectorStateSpace::StateType>();
        for (auto i = 0U; i < dimension; ++i)
        {
            aligned_buffer[i] = static_cast<float>(as->values[i]);
        }

        return Configuration(aligned_buffer.data());
    }
};

/**
 * @brief Motion validity checker using VAMP collision detection
 * 
 * This class is focused solely on checking whether motions between
 * two states are valid (collision-free) using VAMP's motion validation.
 */
template<typename Robot>
class VampMotionValidator : public ob::MotionValidator {
public:
    using Configuration = typename Robot::Configuration;
    static constexpr std::size_t dimension = Robot::dimension;
    static constexpr std::size_t rake = vamp::FloatVectorWidth;
    using EnvironmentVector = vamp::collision::Environment<vamp::FloatVector<rake>>;

    /**
     * @brief Constructor taking space information and vectorized environment
     * @param si Space information pointer  
     * @param env_v Vectorized VAMP environment for collision checking
     */
    VampMotionValidator(const ob::SpaceInformationPtr &si, const EnvironmentVector &env_v)
        : ob::MotionValidator(si), env_v_(env_v)
    {
    }

    /**
     * @brief Check if motion between two states is valid
     * @param s1 Start state
     * @param s2 End state
     * @return true if motion is valid, false otherwise
     */
    bool checkMotion(const ob::State *s1, const ob::State *s2) const override
    {
        return vamp::planning::validate_motion<Robot, rake, Robot::resolution>(
            ompl_to_vamp(s1), ompl_to_vamp(s2), env_v_);
    }

    /**
     * @brief Check motion with last valid state reporting (not implemented)
     * @param s1 Start state
     * @param s2 End state  
     * @param lastValid Last valid state along the path
     * @return true if motion is valid
     * @throws ompl::Exception Always throws as this method is not implemented
     */
    bool checkMotion(const ob::State *s1, const ob::State *s2, 
                     std::pair<ob::State *, double> &lastValid) const override
    {
        throw ompl::Exception("VampMotionValidator: checkMotion with lastValid not implemented");
    }

private:
    const EnvironmentVector &env_v_;

    /**
     * @brief Convert OMPL state to VAMP configuration
     * @param state OMPL state
     * @return VAMP configuration
     */
    static Configuration ompl_to_vamp(const ob::State *state)
    {
        std::array<typename Configuration::S::ScalarT, Configuration::num_scalars> aligned_buffer;

        auto *as = state->as<ob::RealVectorStateSpace::StateType>();
        for (auto i = 0U; i < dimension; ++i)
        {
            aligned_buffer[i] = static_cast<float>(as->values[i]);
        }

        return Configuration(aligned_buffer.data());
    }
};

} // namespace vamp_ompl