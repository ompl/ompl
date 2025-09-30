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

#pragma once

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>

#include <vamp/collision/environment.hh>
#include <vamp/planning/validate.hh>

#include <memory>
#include <array>

namespace ompl { namespace vamp {

/**
 * @brief VAMP-aware StateSpace for robot motion planning
 * 
 * This is a custom OMPL StateSpace that integrates VAMP's collision detection.
 * It extends RealVectorStateSpace and automatically configures joint limits
 * from the VAMP robot definition.
 * 
 * @tparam Robot VAMP robot type (e.g., vamp::robots::Panda)
 */
template<typename Robot>
class VAMPStateSpace : public base::RealVectorStateSpace {
public:
    using Configuration = typename Robot::Configuration;
    static constexpr std::size_t dimension = Robot::dimension;
    static constexpr std::size_t simd_width = ::vamp::FloatVectorWidth;
    using VectorizedEnvironment = ::vamp::collision::Environment<::vamp::FloatVector<simd_width>>;

    /**
     * @brief VAMP StateValidityChecker
     */
    class StateValidityChecker : public base::StateValidityChecker {
    private:
        const VectorizedEnvironment& environment_;

    public:
        StateValidityChecker(const base::SpaceInformationPtr& si,
                           const VectorizedEnvironment& env)
            : base::StateValidityChecker(si), environment_(env) {}

        bool isValid(const base::State* state) const override {
            auto config = convertOMPLToVAMP(state);
            return ::vamp::planning::validate_motion<Robot, simd_width, 1>(
                config, config, environment_);
        }

    private:
        Configuration convertOMPLToVAMP(const base::State* state) const {
            alignas(Configuration::S::Alignment) 
            std::array<typename Configuration::S::ScalarT, Configuration::num_scalars> buffer;
            
            auto* real_state = state->as<StateType>();
            for (std::size_t i = 0; i < dimension; ++i) {
                buffer[i] = static_cast<float>(real_state->values[i]);
            }
            
            return Configuration(buffer.data());
        }
    };

    /**
     * @brief VAMP MotionValidator
     */
    class MotionValidator : public base::MotionValidator {
    private:
        const VectorizedEnvironment& environment_;

    public:
        MotionValidator(const base::SpaceInformationPtr& si,
                       const VectorizedEnvironment& env)
            : base::MotionValidator(si), environment_(env) {}

        bool checkMotion(const base::State* s1, const base::State* s2) const override {
            auto config1 = convertOMPLToVAMP(s1);
            auto config2 = convertOMPLToVAMP(s2);
            return ::vamp::planning::validate_motion<Robot, simd_width, Robot::resolution>(
                config1, config2, environment_);
        }

        bool checkMotion(const base::State* s1, const base::State* s2,
                        std::pair<base::State*, double>& lastValid) const override {
            bool valid = checkMotion(s1, s2);
            if (!valid && lastValid.first) {
                si_->copyState(lastValid.first, s1);
                lastValid.second = 0.0;
            }
            return valid;
        }

    private:
        Configuration convertOMPLToVAMP(const base::State* state) const {
            alignas(Configuration::S::Alignment) 
            std::array<typename Configuration::S::ScalarT, Configuration::num_scalars> buffer;
            
            auto* real_state = state->as<StateType>();
            for (std::size_t i = 0; i < dimension; ++i) {
                buffer[i] = static_cast<float>(real_state->values[i]);
            }
            
            return Configuration(buffer.data());
        }
    };

private:
    VectorizedEnvironment vectorized_environment_;

public:
    /**
     * @brief Constructor with VAMP collision environment
     * @param environment VAMP collision environment
     */
    explicit VAMPStateSpace(const ::vamp::collision::Environment<float>& environment)
        : RealVectorStateSpace(dimension), vectorized_environment_(environment) {
        
        // Set robot joint limits
        base::RealVectorBounds bounds(dimension);
        for (std::size_t i = 0; i < dimension; ++i) {
            bounds.setLow(i, Robot::s_a[i]);
            bounds.setHigh(i, Robot::s_a[i] + Robot::s_m[i]);
        }
        setBounds(bounds);
        
        setName("VAMP_" + std::string(Robot::name));
    }

    /**
     * @brief Allocate the default state validity checker for this space
     * @param si Space information
     * @return VAMP state validity checker
     */
    base::StateValidityCheckerPtr allocDefaultStateValidityChecker(
        const base::SpaceInformationPtr& si) const {
        return std::make_shared<StateValidityChecker>(si, vectorized_environment_);
    }

    /**
     * @brief Allocate the default motion validator for this space
     * @param si Space information
     * @return VAMP motion validator
     */
    base::MotionValidatorPtr allocDefaultMotionValidator(
        const base::SpaceInformationPtr& si) const {
        return std::make_shared<MotionValidator>(si, vectorized_environment_);
    }
};

}} // namespace ompl::vamp 