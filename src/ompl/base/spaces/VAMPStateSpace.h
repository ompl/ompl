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

#ifndef OMPL_BASE_SPACES_VAMP_STATE_SPACE_
#define OMPL_BASE_SPACES_VAMP_STATE_SPACE_

// OMPL includes
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>

// VAMP includes
#include <vamp/collision/environment.hh>
#include <vamp/collision/factory.hh>
#include <vamp/planning/validate.hh>
#include <vamp/robots/panda.hh>
#include <vamp/robots/ur5.hh>
#include <vamp/robots/fetch.hh>
#include <vamp/robots/baxter.hh>

// Standard library
#include <memory>
#include <array>
#include <string>

namespace ompl { namespace geometric {

    /**
       @anchor gVAMP
       @par Short Description
       VAMPStateSpace provides SIMD-accelerated collision detection for manipulator motion planning by
       integrating the VAMP (Vector Accelerated Motion Planning) library into OMPL. VAMP leverages
       modern CPU vectorization (AVX2, NEON) to perform collision checking orders of
       magnitude faster than traditional methods. The state space automatically configures joint limits
       from robot definitions and provides optimized state validity checking and motion validation.

       VAMPStateSpace supports both primitive collision objects (spheres, boxes, capsules) and point
       cloud environments through VAMP's Collision-Affording Point Tree (CAPT) data structure, enabling
       efficient planning in sensor-derived environments.

       @par Supported Robots
       Pre-defined robot models include: Panda (Franka Emika), UR5 (Universal Robots), Fetch, and Baxter.

       @par Usage Example
       @code
       // Create VAMP environment with sphere obstacles
       vamp::collision::Environment<float> environment;
       environment.spheres.emplace_back(0.3, 0.0, 0.5, 0.2);  // x, y, z, radius

       // Create state space for Panda robot
       auto space = std::make_shared<ompl::geometric::VAMPStateSpace<vamp::robots::Panda>>(environment);

       // Use with SimpleSetup
       ompl::geometric::SimpleSetup ss(space);
       auto si = ss.getSpaceInformation();
       ss.setStateValidityChecker(space->allocDefaultStateValidityChecker(si));
       si->setMotionValidator(space->allocDefaultMotionValidator(si));

       // Set start/goal and plan using any OMPL geometric planner
       @endcode

       @par External Documentation
       The following paper describes VAMP's vectorization methodology and performance characteristics:

       Wil Thomason, Zachary Kingston, and Lydia E. Kavraki,
       "VAMP: Motions in Microseconds via Vectorized Sampling-Based Planning,"
       arXiv preprint arXiv:2309.14545, 2024.
       <a href="https://arxiv.org/abs/2309.14545">arXiv:2309.14545</a>

       VAMP library: <a href="https://github.com/KavrakiLab/vamp">https://github.com/KavrakiLab/vamp</a>
    */

    /** \brief A RealVectorStateSpace with VAMP's SIMD-accelerated collision detection.
     *
     * This state space extends RealVectorStateSpace to provide high-performance collision
     * detection for manipulator robots using SIMD vectorization. Joint limits are automatically
     * configured from the robot definition, and custom validators leverage VAMP's optimized
     * collision checking algorithms.
     *
     * @tparam Robot VAMP robot type (e.g., vamp::robots::Panda, vamp::robots::UR5)
     */
template<typename Robot>
class VAMPStateSpace : public base::RealVectorStateSpace {
public:
    using Configuration = typename Robot::Configuration;
    static constexpr std::size_t dimension = Robot::dimension;
    static constexpr std::size_t simd_width = vamp::FloatVectorWidth;
    using VectorizedEnvironment = vamp::collision::Environment<vamp::FloatVector<simd_width>>;

    /** \brief State validity checker using VAMP's vectorized collision detection.
     *
     * This checker uses VAMP's SIMD-optimized collision detection to verify that a
     * robot configuration is collision-free. The implementation automatically converts
     * OMPL states to VAMP's aligned configuration format and performs vectorized
     * sphere-based collision checking against the environment.
     */
    class StateValidityChecker : public base::StateValidityChecker {
    private:
        const VectorizedEnvironment& environment_;
        const VAMPStateSpace* space_;

    public:
        StateValidityChecker(const base::SpaceInformationPtr& si,
                           const VectorizedEnvironment& env,
                           const VAMPStateSpace* space)
            : base::StateValidityChecker(si), environment_(env), space_(space) {}

        bool isValid(const base::State* state) const override {
            auto config = space_->convertOMPLToVAMP(state);
            return vamp::planning::validate_motion<Robot, simd_width, 1>(
                config, config, environment_);
        }
    };

    /** \brief Motion validator using VAMP's vectorized continuous collision detection.
     *
     * This validator uses VAMP's SIMD-optimized swept-sphere collision detection to verify
     * that a motion between two configurations is collision-free. The motion is discretized
     * according to the robot's resolution parameter (defined in the robot template), and
     * each intermediate configuration is checked using vectorized collision detection.
     */
    class MotionValidator : public base::MotionValidator {
    private:
        const VectorizedEnvironment& environment_;
        const VAMPStateSpace* space_;

    public:
        MotionValidator(const base::SpaceInformationPtr& si,
                       const VectorizedEnvironment& env,
                       const VAMPStateSpace* space)
            : base::MotionValidator(si), environment_(env), space_(space) {}

        bool checkMotion(const base::State* s1, const base::State* s2) const override {
            auto config1 = space_->convertOMPLToVAMP(s1);
            auto config2 = space_->convertOMPLToVAMP(s2);
            return vamp::planning::validate_motion<Robot, simd_width, Robot::resolution>(
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
    };

private:
    VectorizedEnvironment vectorized_environment_;

    /** \brief Convert OMPL state to VAMP configuration.
     *
     * This helper method converts an OMPL RealVectorState to a VAMP robot configuration,
     * ensuring proper memory alignment and type conversion (double to float).
     *
     * @param state OMPL state to convert
     * @return VAMP robot configuration
     */
    Configuration convertOMPLToVAMP(const base::State* state) const {
        alignas(Configuration::S::Alignment) 
        std::array<typename Configuration::S::ScalarT, Configuration::num_scalars> buffer;
        
        auto* real_state = state->as<StateType>();
        for (std::size_t i = 0; i < dimension; ++i) {
            buffer[i] = static_cast<float>(real_state->values[i]);
        }
        
        return Configuration(buffer.data());
    }

public:
    /** \brief Construct a VAMP state space for the specified robot.
     *
     * The constructor automatically:
     * - Sets the state space dimension to match the robot's DOF
     * - Configures joint limits from the robot definition
     * - Vectorizes the collision environment for SIMD operations
     * - Sets the state space name to "VAMP_<robotname>"
     *
     * @param environment VAMP collision environment containing obstacles (spheres, boxes, capsules, or point clouds)
     */
    explicit VAMPStateSpace(const vamp::collision::Environment<float>& environment)
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

    /** \brief Allocate the VAMP state validity checker.
     *
     * This method provides VAMP's vectorized collision checker instead of the standard
     * validity checker. The returned checker automatically uses SIMD instructions for
     * collision detection.
     *
     * @param si The space information to associate with the checker
     * @return A shared pointer to the VAMP state validity checker
     */
    base::StateValidityCheckerPtr allocDefaultStateValidityChecker(
        const base::SpaceInformationPtr& si) const {
        return std::make_shared<StateValidityChecker>(si, vectorized_environment_, this);
    }

    /** \brief Allocate the VAMP motion validator.
     *
     * This method provides VAMP's vectorized continuous collision checker instead of the
     * standard motion validator. The returned validator uses swept-sphere collision detection
     * with SIMD acceleration.
     *
     * @param si The space information to associate with the validator
     * @return A shared pointer to the VAMP motion validator
     */
    base::MotionValidatorPtr allocDefaultMotionValidator(
        const base::SpaceInformationPtr& si) const {
        return std::make_shared<MotionValidator>(si, vectorized_environment_, this);
    }
};

}} // namespace ompl::geometric

#endif 