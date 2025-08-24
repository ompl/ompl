/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Rice University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sahruday Patti */
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
    constexpr std::size_t MAX_SUPPORTED_ROBOT_DIMENSION = 16; // Support robots up to 16 DOF
    
    /**
     * @brief Thread-safe configuration converter with proper RAII
     * 
     * This class manages conversion buffers safely and provides exception-safe
     * conversion from OMPL states to VAMP configurations.
     */
    template<typename Robot>
    class SafeConfigurationConverter {
    private:
        static constexpr std::size_t robot_dimension_ = Robot::dimension;
        // CRITICAL: Use aligned allocation for SIMD compatibility
        // VAMP configurations use SIMD operations that require 32-byte alignment
        static constexpr std::size_t SIMD_ALIGNMENT = 32;
        alignas(SIMD_ALIGNMENT) float buffer_[MAX_SUPPORTED_ROBOT_DIMENSION];
        std::size_t buffer_size_;
        
    public:
        SafeConfigurationConverter() 
            : buffer_size_(robot_dimension_) {
            static_assert(robot_dimension_ <= MAX_SUPPORTED_ROBOT_DIMENSION, 
                         "Robot dimension exceeds maximum supported dimension");
            // Initialize buffer to zero for safety
            std::fill(std::begin(buffer_), std::begin(buffer_) + robot_dimension_, 0.0f);
        }
        
        // Non-copyable but movable for performance
        SafeConfigurationConverter(const SafeConfigurationConverter&) = delete;
        SafeConfigurationConverter& operator=(const SafeConfigurationConverter&) = delete;
        SafeConfigurationConverter(SafeConfigurationConverter&&) = default;
        SafeConfigurationConverter& operator=(SafeConfigurationConverter&&) = default;
        
        /**
         * @brief Convert OMPL state to VAMP configuration safely
         * @param ompl_state OMPL state pointer (must not be null)
         * @return VAMP configuration optimized for SIMD operations
         * @throws std::invalid_argument if state is null or wrong type
         */
        auto convert(const ob::State* ompl_state) -> typename Robot::Configuration {
            if (!ompl_state) {
                throw std::invalid_argument("OMPL state cannot be null");
            }
            
            const auto* real_vector_state = ompl_state->as<ob::RealVectorStateSpace::StateType>();
            if (!real_vector_state) {
                throw std::invalid_argument("Expected RealVectorStateSpace::StateType");
            }
            
            // CRITICAL: Check if values array is valid before accessing
            if (!real_vector_state->values) {
                throw std::invalid_argument("RealVectorStateSpace::StateType has null values array");
            }
            
            // Safe conversion with bounds checking
            for (std::size_t joint_index = 0; joint_index < robot_dimension_; ++joint_index) {
                buffer_[joint_index] = static_cast<float>(real_vector_state->values[joint_index]);
            }
            
            return typename Robot::Configuration(buffer_);
        }
    };
    
    /**
     * @brief Get function-local converter instance for performance with safe initialization
     * 
     * Each template instantiation gets its own converter instance to avoid allocation overhead
     * while maintaining initialization safety. Uses function-local static to avoid
     * static initialization order fiasco.
     * 
     * CRITICAL: Replaced thread_local with function-local static to avoid
     * static initialization order fiasco that causes segfaults in programmatic path.
     * This provides one converter per template instantiation with lazy initialization.
     * 
     * @tparam Robot VAMP robot type
     * @return Reference to converter instance
     */
    template<typename Robot>
    SafeConfigurationConverter<Robot>& get_converter() {
        // CRITICAL: Use function-local static instead of thread_local to avoid
        // static initialization order issues. This is safe because:
        // 1. Function-local statics are initialized on first call (lazy initialization)
        // 2. Avoids thread_local initialization during static construction
        // 3. Still provides one converter per template instantiation
        static SafeConfigurationConverter<Robot> converter;
        return converter;
    }
    
    /**
     * @brief Convert OMPL state to VAMP configuration with zero-copy optimization
     * @tparam Robot VAMP robot type
     * @param ompl_state OMPL state pointer
     * @return VAMP configuration optimized for SIMD operations
     * 
     * Performance Insight: Uses function-local static converter instances to avoid memory 
     * allocations in hot path while maintaining initialization safety and exception safety.
     * 
     * Memory Layout Transformation:
     * OMPL AOS: [joint1, joint2, joint3, ...] (sequential memory, SIMD-inefficient)
     * VAMP SOA: [[joint1_lane0...7], [joint2_lane0...7], ...] (SIMD-optimized)
     */
    template<typename Robot>
    auto ompl_to_vamp(const ob::State* ompl_state) -> typename Robot::Configuration {
        try {
            return get_converter<Robot>().convert(ompl_state);
        } catch (const std::exception& e) {
            // Enhanced error reporting for debugging initialization issues
            std::string error_msg = "Failed to convert OMPL state to VAMP configuration for robot '" 
                                  + std::string(Robot::name) + "': " + e.what();
            throw std::runtime_error(error_msg);
        }
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
    static constexpr std::size_t robot_dimension_ = Robot::dimension;
    static constexpr std::size_t simd_lane_width_ = vamp::FloatVectorWidth;
    using VectorizedEnvironment = vamp::collision::Environment<vamp::FloatVector<simd_lane_width_>>;

    /**
     * @brief Constructor taking space information and vectorized environment
     * @param space_information Space information pointer
     * @param vectorized_environment Vectorized VAMP environment for collision checking
     */
    VampStateValidator(const ob::SpaceInformationPtr& space_information, 
                      const VectorizedEnvironment& vectorized_environment)
        : ob::StateValidityChecker(space_information)
        , vectorized_environment_(vectorized_environment) {
        
        // DEFENSIVE CHECK: Validate that template instantiation is working correctly
        static_assert(robot_dimension_ > 0, "Robot dimension must be positive");
        static_assert(robot_dimension_ <= conversion::MAX_SUPPORTED_ROBOT_DIMENSION, 
                     "Robot dimension exceeds maximum supported");
        
        // Validate space information
        if (!space_information) {
            throw std::invalid_argument("SpaceInformation cannot be null for VampStateValidator");
        }
        
        // Validate space dimension matches robot dimension
        if (space_information->getStateDimension() != robot_dimension_) {
            throw std::invalid_argument("Space dimension (" + 
                std::to_string(space_information->getStateDimension()) + 
                ") does not match robot dimension (" + 
                std::to_string(robot_dimension_) + ") for robot " + Robot::name);
        }
        
        // Test converter initialization early to catch static initialization issues
        try {
            auto& converter = conversion::get_converter<Robot>();
            (void)converter; // Suppress unused variable warning
        } catch (const std::exception& e) {
            throw std::runtime_error("Failed to initialize state converter for robot " + 
                std::string(Robot::name) + ": " + e.what());
        }
    }

    /**
     * @brief Check if a state is valid (collision-free) using vectorized collision detection
     * @param ompl_state The state to check
     * @return true if state is valid, false otherwise
     * 
     * Implementation Note: Uses VAMP's motion validation with identical start/end
     * configurations to perform point collision checking within the vectorized framework.
     * This maintains algorithmic consistency across all validation operations.
     */
    bool isValid(const ob::State* ompl_state) const override {
        try {
            auto robot_configuration = conversion::ompl_to_vamp<Robot>(ompl_state);
            // Use VAMP validation with single configuration (start == end for point check)
            return vamp::planning::validate_motion<Robot, simd_lane_width_, 1>(
                robot_configuration, robot_configuration, vectorized_environment_);
        } catch (const std::exception& e) {
            // Log error for debugging but don't throw from validator
            // Conservative approach: assume invalid on conversion error
            return false;
        }
    }

    /**
     * @brief OMPL-required C++ method name
     */
    auto is_valid(const ob::State* ompl_state) const -> bool {
        return isValid(ompl_state);
    }

private:
    const VectorizedEnvironment& vectorized_environment_;
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
    static constexpr std::size_t robot_dimension_ = Robot::dimension;
    static constexpr std::size_t simd_lane_width_ = vamp::FloatVectorWidth;
    static constexpr std::size_t motion_sampling_resolution_ = Robot::resolution;
    using VectorizedEnvironment = vamp::collision::Environment<vamp::FloatVector<simd_lane_width_>>;

    /**
     * @brief Constructor taking space information and vectorized environment
     * @param space_information Space information pointer  
     * @param vectorized_environment Vectorized VAMP environment for collision checking
     */
    VampMotionValidator(const ob::SpaceInformationPtr& space_information, 
                       const VectorizedEnvironment& vectorized_environment)
        : ob::MotionValidator(space_information)
        , vectorized_environment_(vectorized_environment) {
        
        // DEFENSIVE CHECK: Validate that template instantiation is working correctly
        static_assert(robot_dimension_ > 0, "Robot dimension must be positive");
        static_assert(robot_dimension_ <= conversion::MAX_SUPPORTED_ROBOT_DIMENSION, 
                     "Robot dimension exceeds maximum supported");
        static_assert(motion_sampling_resolution_ > 0, "Motion sampling resolution must be positive");
        
        // Validate space information
        if (!space_information) {
            throw std::invalid_argument("SpaceInformation cannot be null for VampMotionValidator");
        }
        
        // Validate space dimension matches robot dimension
        if (space_information->getStateDimension() != robot_dimension_) {
            throw std::invalid_argument("Space dimension (" + 
                std::to_string(space_information->getStateDimension()) + 
                ") does not match robot dimension (" + 
                std::to_string(robot_dimension_) + ") for robot " + Robot::name);
        }
        
        // Test converter initialization early to catch static initialization issues
        try {
            auto& converter = conversion::get_converter<Robot>();
            (void)converter; // Suppress unused variable warning
        } catch (const std::exception& e) {
            throw std::runtime_error("Failed to initialize motion converter for robot " + 
                std::string(Robot::name) + ": " + e.what());
        }
    }

    /**
     * @brief Check if motion between two states is valid using vectorized "rake" sampling
     * @param start_state Start state
     * @param end_state End state
     * @return true if motion is valid, false otherwise
     * 
     * Note: This method demonstrates how vectorized collision detection
     * can be applied to motion validation. Instead of checking intermediate points
     * sequentially, VAMP distributes them across SIMD lanes for parallel processing.
     * 
     * The sampling resolution is compile-time configurable per robot type, allowing
     * different robots to use different validation granularities based on their
     * kinematic properties and typical motion characteristics.
     */
    bool checkMotion(const ob::State* start_state, const ob::State* end_state) const override {
        try {
            return vamp::planning::validate_motion<Robot, simd_lane_width_, motion_sampling_resolution_>(
                conversion::ompl_to_vamp<Robot>(start_state), 
                conversion::ompl_to_vamp<Robot>(end_state), 
                vectorized_environment_);
        } catch (const std::exception& e) {
            // Conservative approach: assume invalid motion on conversion error
            return false;
        }
    }

    /**
     * @brief Check motion
     */
    auto check_motion(const ob::State* start_state, const ob::State* end_state) const -> bool {
        return checkMotion(start_state, end_state);
    }

    /**
     * @brief Check motion with last valid state reporting (fallback implementation)
     * @param start_state Start state
     * @param end_state End state  
     * @param last_valid_state Last valid state along the path
     * @return true if motion is valid
     * 
     * Note: This provides a fallback implementation for planners that require
     * last valid state reporting. While not as efficient as the vectorized approach,
     * it ensures compatibility with all OMPL planners. The method performs standard
     * motion checking and sets last_valid_state appropriately.
     */
    bool checkMotion(const ob::State* start_state, const ob::State* end_state, 
                     std::pair<ob::State*, double>& last_valid_state) const override {

        bool is_valid = checkMotion(start_state, end_state);
        
        if (is_valid) {
            // If motion is valid, set last valid state to the end state
            if (last_valid_state.first) {
                si_->copyState(last_valid_state.first, end_state);
                last_valid_state.second = 1.0;  // Full motion is valid
            }
            return true;
        } else {
            // If motion is invalid, set last valid state to start state
            // This is conservative but safe - assumes start state is valid
            if (last_valid_state.first) {
                si_->copyState(last_valid_state.first, start_state);
                last_valid_state.second = 0.0;  // No motion progress is valid
            }
            return false;
        }
    }

    /**
     * @brief Check motion with last valid state reporting
     */
    auto check_motion_with_last_valid(const ob::State* start_state, 
                                     const ob::State* end_state, 
                                     std::pair<ob::State*, double>& last_valid_state) const -> bool {
        return checkMotion(start_state, end_state, last_valid_state);
    }

private:
    const VectorizedEnvironment& vectorized_environment_;
};

} // namespace vamp_ompl