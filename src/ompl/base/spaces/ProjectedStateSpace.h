/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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

/* Author: Zachary Kingston */

#ifndef OMPL_BASE_SPACES_PROJECTED_STATE_SPACE_
#define OMPL_BASE_SPACES_PROJECTED_STATE_SPACE_

#include "ompl/base/MotionValidator.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/Constraint.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include "ompl/geometric/PathGeometric.h"

#include <eigen3/Eigen/Core>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::ProjectedStateSpace. */
        OMPL_CLASS_FORWARD(ProjectedStateSpace);
        /// @endcond

        /** \class ompl::base::ProjectedStateSpacePtr
         * \brief A boost shared pointer wrapper for
         * ompl::base::ProjectedStateSpace. */

        /** \brief StateSampler for use on an atlas. */
        class ProjectedStateSampler : public StateSampler
        {
        public:
            /** \brief Create a sampler for the specified space information.
             * \note The underlying state space must be an AtlasStateSpace. */
            ProjectedStateSampler(const SpaceInformation *si);

            /** \brief Create a sampler for the specified \a atlas space. */
            ProjectedStateSampler(const ProjectedStateSpace &ss);

            /** \brief Sample a state uniformly from the charted regions of the
             * manifold. Return sample in \a state. */
            virtual void sampleUniform(State *state);

            /** \brief Sample a state uniformly from the ball with center \a
             * near and radius \a distance. Return sample in \a state.
             * \note rho_s_ is a good choice for \a distance. */
            virtual void sampleUniformNear(State *state, const State *near, const double distance);

            /** \brief Sample a state uniformly from a normal distribution with
                given \a mean and \a stdDev. Return sample in \a state. */
            virtual void sampleGaussian(State *state, const State *mean, const double stdDev);

        private:
            /** \brief Space on which to sample. */
            const ProjectedStateSpace &ss_;

            /** \brief Underlying space's sampler. */
            const StateSamplerPtr sampler_;
        };

        /** \brief ValidStateSampler for use on an atlas. */
        class ProjectedValidStateSampler : public ValidStateSampler
        {
        public:
            /** \brief Create a valid state sampler for the specifed space
             * information \a si. */
            ProjectedValidStateSampler(const SpaceInformation *si);

            /** \brief Sample a valid state uniformly from the charted regions
             * of the manifold. Return sample in \a state. */
            virtual bool sample(State *state);

            /** \brief Sample a valid state uniformly from the ball with center
             * \a near and radius \a distance. Return sample in \a state.
             * \note rho_s_ is a good choice for \a distance. */
            virtual bool sampleNear(State *state, const State *near, const double distance);

        private:
            /** \brief Underlying ordinary atlas state sampler. */
            ProjectedStateSampler sampler_;
        };

        /** \brief Atlas-specific implementation of checkMotion(). */
        class ProjectedMotionValidator : public MotionValidator
        {
        public:
            /** \brief Constructor. */
            ProjectedMotionValidator(SpaceInformation *si);

            /** \brief Constructor. */
            ProjectedMotionValidator(const SpaceInformationPtr &si);

            /** \brief Return whether we can step from \a s1 to \a s2 along the
             * manifold without collision. */
            bool checkMotion(const State *s1, const State *s2) const;

            /** \brief Return whether we can step from \a s1 to \a s2 along the
             * manifold without collision. If not, return the last valid state
             * and its interpolation parameter in \a lastValid.
             * \note The interpolation parameter will not likely reproduce the
             * last valid state if used in interpolation since the distance
             * between the last valid state and \a s2 is estimated using the
             * ambient metric. */
            bool checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const;

        private:
            /** \brief Space in which we check motion. */
            const ProjectedStateSpace &ss_;
        };

        /** \brief State space encapsulating a planner-agnostic algorithm for
         * planning on a constraint manifold. */
        class ProjectedStateSpace : public StateSpace
        {
        public:
            /** \brief Construct an atlas with the specified dimensions. */
            ProjectedStateSpace(const StateSpacePtr space, const ConstraintPtr constraint);

            /** \brief Destructor. */
            virtual ~ProjectedStateSpace(void);

            /** @name Setup and tuning of atlas parameters
             * @{ */

            /** \brief Final setup for the space. */
            void setup(void);

            /** \brief Check that the space referred to by the space information
             * \a si is, in fact, an AtlasStateSpace. */
            static void checkSpace(const SpaceInformation *si);

            /** \brief Reset the space (except for anchor charts). */
            void clear(void);

            /** \brief Associate \a si with this space. Requires that \a si was
             * constructed from this AtlasStateSpace. */
            void setSpaceInformation(const SpaceInformationPtr &si);

            /** \brief Set \a delta, the step size for traversing the manifold
             * and collision checking. Default 0.02. */
            void setDelta(const double delta);

            /** \brief Get delta. */
            double getDelta(void) const;

            /** \brief Returns the dimension of the ambient space. */
            unsigned int getAmbientDimension() const;

            /** \brief Returns the dimension of the manifold. */
            unsigned int getManifoldDimension() const;

            ConstraintPtr getConstraint() const;

            StateSpacePtr getAmbientStateSpace() const;

            /** \brief Traverse the manifold from \a from toward \a to. Returns
             * true if we reached \a to, and false if we stopped early for any
             * reason, such as a collision or traveling too far. No collision
             * checking is performed if \a interpolate is true. If \a stateList
             * is not nullptr, the sequence of intermediates is saved to it,
             * including a copy of \a from, as well as the final state, which is
             * a copy of \a to if we reached \a to. Caller is responsible for
             * freeing states returned in \a stateList. */
            bool traverseManifold(const StateType *from, const StateType *to, const bool interpolate = false,
                                  std::vector<StateType *> *stateList = nullptr) const;

            /** \brief Find the state between \a from and \a to at time \a t,
             * where \a t = 0 is \a from, and \a t = 1 is the final state
             * reached by followManifold(\a from, \a to, true, ...), which may
             * not be \a to. State returned in \a state. */
            void interpolate(const State *from, const State *to, const double t, State *state) const;

            /** \brief Like interpolate(...), but uses the information about
             * intermediate states already supplied in \a stateList from a
             * previous call to followManifold(..., true, \a stateList). The
             * 'from' and 'to' states are the first and last elements \a
             * stateList. Assumes \a stateList contains at least two
             * elements. */
            void piecewiseInterpolate(const std::vector<StateType *> &stateList, const double t, State *state) const;

            /** \brief Whether interpolation is symmetric. (Yes.) */
            bool hasSymmetricInterpolate(void) const;

            /** \brief Return an instance of the AtlasStateSampler. */
            StateSamplerPtr allocDefaultStateSampler(void) const;

            /** @} */

            /** @name Visualization and debug
             * @{ */

            /** \brief Write a mesh of a path on the atlas to stream. Insert
             * additional vertices to project the edges along the manifold if \a
             * asIs == true. */
            void dumpPath(ompl::geometric::PathGeometric &path, std::ostream &out, const bool asIs = false) const;

            unsigned int getDimension() const;
            double getMaximumExtent() const;
            double getMeasure() const;
            void enforceBounds(State *state) const;
            bool satisfiesBounds(const State *state) const;
            void copyState(State *destination, const State *source) const;
            double distance(const State *state1, const State *state2) const;
            bool equalStates(const State *state1, const State *state2) const;
            State *allocState() const;
            void freeState(State *state) const;

            /** @} */

        protected:
            /** \brief SpaceInformation associated with this space. */
            SpaceInformation *si_;

            /** \brief Ambient state space associated with this space. */
            StateSpacePtr ss_;

            /** \brief Constraint function that defines the manifold. */
            ConstraintPtr constraint_;

        private:
            /** \brief Ambient space dimension. */
            const unsigned int n_;

            /** \brief Manifold dimension. */
            const unsigned int k_;

            /** \brief Step size when traversing the manifold and collision checking. */
            double delta_;

            /** \brief Whether setup() has been called. */
            bool setup_;
        };
    }
}

#endif
