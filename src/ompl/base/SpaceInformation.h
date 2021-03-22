/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Ioan Sucan */

#ifndef OMPL_BASE_SPACE_INFORMATION_
#define OMPL_BASE_SPACE_INFORMATION_

#include "ompl/base/State.h"
#include "ompl/base/StateValidityChecker.h"
#include "ompl/base/MotionValidator.h"
#include "ompl/base/StateSpace.h"
#include "ompl/base/ValidStateSampler.h"

#include "ompl/util/ClassForward.h"
#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"

#include <functional>
#include <utility>
#include <cstdlib>
#include <vector>
#include <iostream>

/** \brief Main namespace. Contains everything in this library */
namespace ompl
{
    /** \brief This namespace contains sampling based planning
        routines shared by both planning under geometric constraints
        (geometric) and planning under differential constraints
        (dynamic) */
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::SpaceInformation */
        OMPL_CLASS_FORWARD(SpaceInformation);
        /// @endcond

        /** \class ompl::base::SpaceInformationPtr
            \brief A shared pointer wrapper for ompl::base::SpaceInformation */

        /** \brief If no state validity checking class is specified
            (StateValidityChecker), a std::function can be specified
            instead */
        using StateValidityCheckerFn = std::function<bool(const State *)>;

        /** \brief The base class for space information. This contains
            all the information about the space planning is done in.
            setup() needs to be called as well, before use */
        class SpaceInformation
        {
        public:
            // non-copyable
            SpaceInformation(const SpaceInformation &) = delete;
            SpaceInformation &operator=(const SpaceInformation &) = delete;

            /** \brief Constructor. Sets the instance of the state space to plan with. */
            SpaceInformation(StateSpacePtr space);

            virtual ~SpaceInformation() = default;

            /** \brief Check if a given state is valid or not */
            bool isValid(const State *state) const
            {
                return stateValidityChecker_->isValid(state);
            }

            /** \brief Return the instance of the used state space */
            const StateSpacePtr &getStateSpace() const
            {
                return stateSpace_;
            }

            /** @name Topology-specific state operations (as in the state space)
                @{ */

            /** \brief Check if two states are the same */
            bool equalStates(const State *state1, const State *state2) const
            {
                return stateSpace_->equalStates(state1, state2);
            }

            /** \brief Check if a state is inside the bounding box */
            bool satisfiesBounds(const State *state) const
            {
                return stateSpace_->satisfiesBounds(state);
            }

            /** \brief Compute the distance between two states */
            double distance(const State *state1, const State *state2) const
            {
                return stateSpace_->distance(state1, state2);
            }

            /** \brief Bring the state within the bounds of the state space */
            void enforceBounds(State *state) const
            {
                stateSpace_->enforceBounds(state);
            }

            /** \brief Print a state to a stream */
            void printState(const State *state, std::ostream &out = std::cout) const
            {
                stateSpace_->printState(state, out);
            }

            /** @} */

            /** @name Configuration of state validity checking
                @{ */

            /** \brief Set the instance of the state validity checker
                to use. Parallel implementations of planners assume
                this validity checker is thread safe. */
            void setStateValidityChecker(const StateValidityCheckerPtr &svc)
            {
                stateValidityChecker_ = svc;
                setup_ = false;
            }

            /** \brief If no state validity checking class is
                specified (StateValidityChecker), a function can
                be specified instead. This version however incurs a
                small additional overhead when calling the function,
                since there is one more level of indirection */
            void setStateValidityChecker(const StateValidityCheckerFn &svc);

            /** \brief Return the instance of the used state validity checker */
            const StateValidityCheckerPtr &getStateValidityChecker() const
            {
                return stateValidityChecker_;
            }

            /** \brief Set the instance of the motion validity checker
                to use. Parallel implementations of planners assume
                this validity checker is thread safe.  */
            void setMotionValidator(const MotionValidatorPtr &mv)
            {
                motionValidator_ = mv;
                setup_ = false;
            }

            /** \brief Return the instance of the used state validity checker */
            const MotionValidatorPtr &getMotionValidator() const
            {
                return motionValidator_;
            }

            /** \brief Return the non-const instance of the used state validity checker */
            MotionValidatorPtr& getMotionValidator()
            {
                return motionValidator_;
            }

            /** \brief Set the resolution at which state validity
                needs to be verified in order for a motion between two
                states to be considered valid. This value is specified
                as a fraction of the space's extent. This call is only
                applicable if a ompl::base::DiscreteMotionValidator is
                used. See \ref stateValidation. */
            void setStateValidityCheckingResolution(double resolution)
            {
                stateSpace_->setLongestValidSegmentFraction(resolution);
                setup_ = false;
            }

            /** \brief Get the resolution at which state validity is
                verified. This call is only applicable if a
                ompl::base::DiscreteMotionValidator is used. See \ref
                stateValidation. */
            double getStateValidityCheckingResolution() const
            {
                return stateSpace_->getLongestValidSegmentFraction();
            }

            /** @}*/

            /** \brief Return the dimension of the state space */
            unsigned int getStateDimension() const
            {
                return stateSpace_->getDimension();
            }

            /** \brief Get a measure of the space (this can be thought of as a generalization of volume) */
            double getSpaceMeasure() const
            {
                return stateSpace_->getMeasure();
            }

            /** @name State memory management
                @{ */

            /** \brief Allocate memory for a state */
            State *allocState() const
            {
                return stateSpace_->allocState();
            }

            /** \brief Allocate memory for each element of the array \e states */
            void allocStates(std::vector<State *> &states) const
            {
                for (auto &state : states)
                    state = stateSpace_->allocState();
            }

            /** \brief Free the memory of a state */
            void freeState(State *state) const
            {
                stateSpace_->freeState(state);
            }

            /** \brief Free the memory of an array of states */
            void freeStates(std::vector<State *> &states) const
            {
                for (auto &state : states)
                    stateSpace_->freeState(state);
            }

            /** \brief Copy a state to another */
            void copyState(State *destination, const State *source) const
            {
                stateSpace_->copyState(destination, source);
            }

            /** \brief Clone a state */
            State *cloneState(const State *source) const
            {
                return stateSpace_->cloneState(source);
            }

            /**  @} */

            /** @name Sampling of valid states
                @{ */

            /** \brief Allocate a uniform state sampler for the state space */
            StateSamplerPtr allocStateSampler() const
            {
                return stateSpace_->allocStateSampler();
            }

            /** \brief Allocate an instance of a valid state sampler for this space. If setValidStateSamplerAllocator()
               was previously called,
                the specified allocator is used to produce the state sampler.  Otherwise, a
               ompl::base::UniformValidStateSampler() is
                allocated. */
            ValidStateSamplerPtr allocValidStateSampler() const;

            /** \brief Set the allocator to use for a valid state sampler. This replaces the default uniform valid state
                sampler. This call can be made at any time, but it should not be changed while
               ompl::base::Planner::solve() is executing */
            void setValidStateSamplerAllocator(const ValidStateSamplerAllocator &vssa);

            /** \brief Clear the allocator used for the valid state sampler. This will revert to using the uniform valid
             * state sampler (the default). */
            void clearValidStateSamplerAllocator();

            /** @}*/

            /** @name Primitives typically used by motion planners
                @{ */

            /** \brief Get the maximum extent of the space we are
                planning in. This is the maximum distance that could
                be reported between any two given states */
            double getMaximumExtent() const
            {
                return stateSpace_->getMaximumExtent();
            }

            /** \brief Find a valid state near a given one. If the given state is valid, it will be returned itself.
             *  The two passed state pointers need not point to different memory. Returns true on success.
             *  \param state the location at which to store the valid state, if one is found. This location may be
             * modified even if no valid state is found.
             *  \param near a state that may be invalid near which we would like to find a valid state
             *  \param distance the maximum allowed distance between \e state and \e near
             *  \param attempts the algorithm works by sampling states near state \e near. This parameter defines the
             * maximum number of sampling attempts
             */
            bool searchValidNearby(State *state, const State *near, double distance, unsigned int attempts) const;

            /** \brief Find a valid state near a given one. If the given state is valid, it will be returned itself.
             *  The two passed state pointers need not point to different memory. Returns true on success.
             *  \param sampler the valid state sampler to use when attemting to find a valid sample.
             *  \param state the location at which to store the valid state, if one is found. This location may be
             * modified even if no valid state is found.
             *  \param near a state that may be invalid near which we would like to find a valid state
             *  \param distance the maximum allowed distance between \e state and \e near
             */
            bool searchValidNearby(const ValidStateSamplerPtr &sampler, State *state, const State *near,
                                   double distance) const;

            /** \brief Produce a valid motion starting at \e start by randomly bouncing off of invalid states. The start
             * state \e start is not included in the computed motion (\e states). Returns the number of elements written
             * to \e states (less or equal to \e steps).
             *  \param sss the state space sampler to use
             *  \param start the state at which to start bouncing
             *  \param steps the number of bouncing steps to take
             *  \param states the location at which generated states will be stored
             *  \param alloc flag indicating whether memory should be allocated for \e states */
            unsigned int randomBounceMotion(const StateSamplerPtr &sss, const State *start, unsigned int steps,
                                            std::vector<State *> &states, bool alloc) const;

            /** \brief Incrementally check if the path between two motions is valid. Also compute the last state that
               was
                valid and the time of that state. The time is used to parametrize the motion from s1 to s2, s1 being at
               t =
                0 and s2 being at t = 1. This function assumes s1 is valid.
                \param s1 start state of the motion to be checked (assumed to be valid)
                \param s2 final state of the motion to be checked
                \param lastValid first: storage for the last valid state (may be nullptr); this need not be different
               from \e s1 or \e s2. second: the time (between 0 and 1) of  the last valid state, on the motion from \e
               s1 to \e s2 */
            virtual bool checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const
            {
                return motionValidator_->checkMotion(s1, s2, lastValid);
            }

            /** \brief Check if the path between two states (from \e s1 to \e s2) is valid, using the MotionValidator.
             * This function assumes \e s1 is valid. */
            virtual bool checkMotion(const State *s1, const State *s2) const
            {
                return motionValidator_->checkMotion(s1, s2);
            }

            /** \brief Incrementally check if a sequence of states is valid. Given a vector of states, this routine only
                checks the first \e count elements and marks the index of the first invalid state
                \param states the array of states to be checked
                \param count the number of states to be checked in the array (0 to \e count)
                \param firstInvalidStateIndex location to store the first invalid state index. Unmodified if the
               function returns true */
            bool checkMotion(const std::vector<State *> &states, unsigned int count,
                             unsigned int &firstInvalidStateIndex) const;

            /** \brief Check if a sequence of states is valid using subdivision. */
            bool checkMotion(const std::vector<State *> &states, unsigned int count) const;

            /** \brief Get \e count states that make up a motion between \e s1 and \e s2. Returns the number of states
               that were added to \e states. These states are not checked for validity.
                If \e states.size() >= count or \e alloc is true, the returned value is equal to \e count (or \e count +
               2, if \e endpoints is true).
                Otherwise, fewer states can be returned.
                \param s1 the start state of the considered motion
                \param s2 the end state of the considered motion
                \param states the computed set of states along the specified motion
                \param count the number of intermediate states to compute
                \param endpoints flag indicating whether \e s1 and \e s2 are to be included in states
                \param alloc flag indicating whether memory is to be allocated automatically
            */
            virtual unsigned int getMotionStates(const State *s1, const State *s2, std::vector<State *> &states,
                                                 unsigned int count, bool endpoints, bool alloc) const;

            /** \brief Get the total number of motion segments checked by the MotionValidator so far */
            unsigned int getCheckedMotionCount() const
            {
                return motionValidator_->getCheckedMotionCount();
            }

            /** @}*/

            /** @name Routines for inferring information about the state space
                @{ */

            /** \brief Estimate probability of sampling a valid state. setup() is assumed to have been called. */
            double probabilityOfValidState(unsigned int attempts) const;

            /** \brief Estimate the length of a valid motion. setup() is assumed to have been called.*/
            double averageValidMotionLength(unsigned int attempts) const;

            /** \brief Estimate the number of samples that can be drawn per second, using the sampler returned by
             * allocStateSampler() */
            void samplesPerSecond(double &uniform, double &near, double &gaussian, unsigned int attempts) const;

            /** \brief Print information about the current instance of the state space */
            virtual void printSettings(std::ostream &out = std::cout) const;

            /** \brief Print properties of the current instance of the state space */
            virtual void printProperties(std::ostream &out = std::cout) const;

            /** \brief Get the combined parameters for the classes that the space information manages */
            ParamSet &params()
            {
                return params_;
            }

            /** \brief Get the combined parameters for the classes that the space information manages */
            const ParamSet &params() const
            {
                return params_;
            }

            /** \brief Perform additional setup tasks (run once,
                before use). If state validity checking resolution has
                not been set, estimateMaxResolution() is called to
                estimate it. */
            virtual void setup();

            /** \brief Return true if setup was called */
            bool isSetup() const;

        protected:
            /** \brief Set default motion validator for the state space */
            void setDefaultMotionValidator();

            /** \brief The state space planning is to be performed in */
            StateSpacePtr stateSpace_;

            /** \brief The instance of the state validity checker used for determining the validity of states in the
             * planning process */
            StateValidityCheckerPtr stateValidityChecker_;

            /** \brief The instance of the motion validator to use when determining the validity of motions in the
             * planning process */
            MotionValidatorPtr motionValidator_;

            /** \brief Flag indicating whether setup() has been called on this instance */
            bool setup_;

            /** \brief The optional valid state sampler allocator */
            ValidStateSamplerAllocator vssa_;

            /** \brief Combined parameters for the contained classes */
            ParamSet params_;
        };
    }
}

#endif
