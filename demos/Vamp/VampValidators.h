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
 * @brief Common utility functions for OMPL-VAMP integration
 */
namespace conversion {
    // Thread-local buffer pool to avoid repeated allocations in hot path
    constexpr std::size_t MAX_SUPPORTED_ROBOT_DIMENSION = 16; // Support robots up to 16 DOF
    thread_local std::array<float, MAX_SUPPORTED_ROBOT_DIMENSION> threadLocalConversionBuffer;
    
    /**
     * @brief Convert OMPL state to VAMP configuration with zero-copy optimization
     * @tparam Robot VAMP robot type
     * @param omplState OMPL state pointer
     * @return VAMP configuration optimized for SIMD operations
     * 
     *  Note: This conversion transforms OMPL's Array-of-Structures (AOS) 
     * format to VAMP's Structure-of-Arrays (SOA) format, enabling SIMD operations.
     * 
     * Performance Insight: Uses thread-local buffer pool to avoid memory allocations 
     * in hot path. This is crucial for real-time performance as memory allocation
     * in collision checking can cause significant latency spikes.
     * 
     * Memory Layout Transformation:
     * OMPL AOS: [joint1, joint2, joint3, ...] (sequential memory, SIMD-inefficient)
     * VAMP SOA: [[joint1_lane0...7], [joint2_lane0...7], ...] (SIMD-optimized)
     */
    template<typename Robot>
    static typename Robot::Configuration ompl_to_vamp(const ob::State *omplState)
    {
        using Configuration = typename Robot::Configuration;
        static constexpr std::size_t robotDimension = Robot::dimension;
        
        // Ensure we don't exceed buffer size
        static_assert(robotDimension <= MAX_SUPPORTED_ROBOT_DIMENSION, 
                     "Robot dimension exceeds maximum supported dimension");
        
        auto *realVectorState = omplState->as<ob::RealVectorStateSpace::StateType>();
        for (auto jointIndex = 0U; jointIndex < robotDimension; ++jointIndex)
        {
            threadLocalConversionBuffer[jointIndex] = static_cast<float>(realVectorState->values[jointIndex]);
        }

        return Configuration(threadLocalConversionBuffer.data());
    }
}

/**
 * @brief SIMD-accelerated state validity checker for single configuration validation
 * 
 * This validator implements OMPL's StateValidityChecker interface using VAMP's
 * vectorized collision detection system. While OMPL requests single-state validation,
 * this class leverages VAMP's ability to check multiple configurations simultaneously
 * by using the same configuration in all SIMD lanes.
 * 
 * Vectorization Strategy:
 * Traditional approach: Check one configuration at a time (scalar operations)
 * VAMP approach: Fill all SIMD lanes with the same configuration, leveraging
 * SIMD instructions even for single-point queries. This maintains consistency
 * with the vectorized motion validator and optimizes instruction pipeline usage.
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
    using RobotConfiguration = typename Robot::Configuration;
    static constexpr std::size_t robotDimension = Robot::dimension;
    static constexpr std::size_t simdLaneWidth = vamp::FloatVectorWidth;
    using VectorizedEnvironment = vamp::collision::Environment<vamp::FloatVector<simdLaneWidth>>;

    /**
     * @brief Constructor taking space information and vectorized environment
     * @param spaceInformation Space information pointer
     * @param vectorizedEnvironment Vectorized VAMP environment for collision checking
     */
    VampStateValidator(const ob::SpaceInformationPtr &spaceInformation, const VectorizedEnvironment &vectorizedEnvironment)
        : ob::StateValidityChecker(spaceInformation), m_vectorizedEnvironment(vectorizedEnvironment)
    {
    }

    /**
     * @brief Check if a state is valid (collision-free) using vectorized collision detection
     * @param omplState The state to check
     * @return true if state is valid, false otherwise
     * 
     * Implementation Note: Uses VAMP's motion validation with identical start/end
     * configurations to perform point collision checking within the vectorized framework.
     * This maintains algorithmic consistency across all validation operations.
     */
    bool isValid(const ob::State *omplState) const override
    {
        auto robotConfiguration = conversion::ompl_to_vamp<Robot>(omplState);
        // Use VAMP validation with single configuration (start == end for point check)
        return vamp::planning::validate_motion<Robot, simdLaneWidth, 1>(
            robotConfiguration, robotConfiguration, m_vectorizedEnvironment);
    }

private:
    const VectorizedEnvironment &m_vectorizedEnvironment;
};

/**
 * @brief SIMD-accelerated motion validator implementing the "rake" sampling approach
 * 
 * This validator implements OMPL's MotionValidator interface using VAMP's advanced
 * motion validation system. Unlike traditional motion validators that check states
 * sequentially along a path, this class uses VAMP's "rake" approach to check
 * multiple spatially distributed points simultaneously.
 * 
 * The "Rake" Vectorization Concept:
 * Traditional sequential validation:
 *   Check(t=0.1) → Check(t=0.2) → Check(t=0.3) → ... (sequential, cache-inefficient)
 * 
 * VAMP's "rake" parallel validation:
 *   Check(t=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]) (parallel SIMD operations)
 * 
 * Performance Benefits:
 * 1. SIMD Utilization: 8 collision checks processed simultaneously
 * 2. Cache Efficiency: Spatial locality in collision geometry access
 * 3. Instruction Pipeline: Better CPU pipeline utilization
 * 4. Memory Bandwidth: Amortized memory access costs across multiple checks
 * 
 * Resolution Control:
 * The validation resolution is controlled by Robot::resolution, which determines
 * how many intermediate sampling points are distributed along each motion edge.
 * Higher resolution provides more thorough validation at the cost of computational overhead.
 * 
 * Sampling Distribution:
 * Points are uniformly distributed along the motion edge, with each SIMD lane
 * processing a different temporal sample. This spatial distribution pattern
 * is optimized for typical robot motion characteristics.
 * 
 * @tparam Robot VAMP robot type with resolution parameter
 */
template<typename Robot>
class VampMotionValidator : public ob::MotionValidator {
public:
    using RobotConfiguration = typename Robot::Configuration;
    static constexpr std::size_t robotDimension = Robot::dimension;
    static constexpr std::size_t simdLaneWidth = vamp::FloatVectorWidth;
    static constexpr std::size_t motionSamplingResolution = Robot::resolution;
    using VectorizedEnvironment = vamp::collision::Environment<vamp::FloatVector<simdLaneWidth>>;

    /**
     * @brief Constructor taking space information and vectorized environment
     * @param spaceInformation Space information pointer  
     * @param vectorizedEnvironment Vectorized VAMP environment for collision checking
     */
    VampMotionValidator(const ob::SpaceInformationPtr &spaceInformation, const VectorizedEnvironment &vectorizedEnvironment)
        : ob::MotionValidator(spaceInformation), m_vectorizedEnvironment(vectorizedEnvironment)
    {
    }

    /**
     * @brief Check if motion between two states is valid using vectorized "rake" sampling
     * @param startState Start state
     * @param endState End state
     * @return true if motion is valid, false otherwise
     * 
     *  Note: This method demonstrates how vectorized collision detection
     * can be applied to motion validation. Instead of checking intermediate points
     * sequentially, VAMP distributes them across SIMD lanes for parallel processing.
     * 
     * The sampling resolution is compile-time configurable per robot type, allowing
     * different robots to use different validation granularities based on their
     * kinematic properties and typical motion characteristics.
     */
    bool checkMotion(const ob::State *startState, const ob::State *endState) const override
    {
        return vamp::planning::validate_motion<Robot, simdLaneWidth, motionSamplingResolution>(
            conversion::ompl_to_vamp<Robot>(startState), 
            conversion::ompl_to_vamp<Robot>(endState), 
            m_vectorizedEnvironment);
    }

    /**
     * @brief Check motion with last valid state reporting (not implemented)
     * @param startState Start state
     * @param endState End state  
     * @param lastValidState Last valid state along the path
     * @return true if motion is valid
     * @throws ompl::Exception Always throws as this method is not implemented
     * 
     *  Note: This advanced validation mode requires bisection search
     * to identify the exact collision point along a motion edge. While useful for
     * some planners, it conflicts with VAMP's batch-oriented validation approach
     * and would require significant algorithmic restructuring to implement efficiently.
     */
    bool checkMotion(const ob::State *startState, const ob::State *endState, 
                     std::pair<ob::State *, double> &lastValidState) const override
    {
        throw ompl::Exception("VampMotionValidator: checkMotion with lastValidState not implemented - "
                             "conflicts with vectorized validation approach");
    }

private:
    const VectorizedEnvironment &m_vectorizedEnvironment;
};

} // namespace vamp_ompl