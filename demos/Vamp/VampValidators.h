/**
 * @file VampValidators.h
 * @brief VAMP-optimized state and motion validators for OMPL integration
 * 
 * This file implements the core validation components that bridge OMPL's validation
 * interface with VAMP's vectorized collision detection system. The validators are
 * designed to maximize SIMD utilization for significant performance improvements.
 * 
 * Key Performance Features:
 * - Vectorized collision checking: Process 8 configurations simultaneously
 * - SIMD-optimized memory layout: Structure-of-Arrays for cache efficiency
 * - "Rake" motion validation: Spatially distributed sampling for faster motion checks
 * - Zero-copy integration: Direct OMPL-to-VAMP configuration conversion
 * 
 * Validation Architecture:
 * - VampStateValidator: Point-in-space collision queries
 * - VampMotionValidator: Edge-based motion validation with vectorized sampling
 * 
 * The validators maintain OMPL's validation interface while leveraging VAMP's
 * advanced collision detection capabilities, providing seamless integration
 * with any OMPL-compatible planner.
 */
#pragma once

#include "VampOMPLInterfaces.h"
#include <vamp/planning/validate.hh>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/util/Exception.h>

namespace vamp_ompl {

/**
 * @brief SIMD-accelerated state validity checker for single configuration validation
 * 
 * This validator implements OMPL's StateValidityChecker interface using VAMP's
 * vectorized collision detection system. While OMPL requests single-state validation,
 * this class leverages VAMP's ability to check multiple configurations simultaneously
 * by using the same configuration in all SIMD lanes.
 * 
 * Performance Optimization:
 * The validator uses VAMP's validate_motion function with identical start and end
 * configurations, effectively performing a point collision check but within VAMP's
 * optimized SIMD framework. This approach maintains consistency with motion validation
 * while providing optimal performance for state queries.
 * 
 * Memory Layout:
 * Configurations are converted from OMPL's AOS (Array of Structs) format to VAMP's
 * SOA (Struct of Arrays) format to enable efficient SIMD operations.
 * 
 * @tparam Robot VAMP robot type providing collision checking capabilities
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
 * @brief SIMD-accelerated motion validator implementing the "rake" sampling approach
 * 
 * This validator implements OMPL's MotionValidator interface using VAMP's advanced
 * motion validation system. Unlike traditional motion validators that check states
 * sequentially along a path, this class uses VAMP's "rake" approach to check
 * multiple spatially distributed points simultaneously.
 * 
 * The "Rake" Concept:
 * Instead of: Check(t=0.1) → Check(t=0.2) → Check(t=0.3) → ... (sequential)
 * VAMP does: Check(t=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]) (parallel)
 * 
 * This spatial distribution allows SIMD instructions to process 8 collision checks
 * simultaneously, resulting in significant performance improvements for motion validation.
 * 
 * Resolution Control:
 * The validation resolution is controlled by Robot::resolution, which determines
 * how many intermediate points are sampled along each motion. Higher resolution
 * provides more thorough validation at the cost of computational overhead.
 * 
 * @tparam Robot VAMP robot type with resolution parameter
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