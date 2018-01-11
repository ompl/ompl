/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Rice University
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

#ifndef OMPL_BASE_SPACES_CONSTRAINED_STATE_SPACE_
#define OMPL_BASE_SPACES_CONSTRAINED_STATE_SPACE_

#include "ompl/base/Constraint.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/MotionValidator.h"
#include "ompl/base/spaces/WrapperStateSpace.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/geometric/PathGeometric.h"

#include <eigen3/Eigen/Core>

namespace ompl
{
    namespace magic
    {
        static const double CONSTRAINED_STATE_SPACE_DELTA = 0.01;
    }

    namespace base
    {
        class ConstrainedStateSpace;

        /** \brief StateSampler for use on an atlas. */
        class ConstrainedStateSampler : public WrapperStateSampler
        {
        public:
            ConstrainedStateSampler(const ConstrainedStateSpace *space, StateSamplerPtr sampler);

            /** \brief Sample a state uniformly from the charted regions of the
             * manifold. Return sample in \a state. */
            void sampleUniform(State *state);

            /** \brief Sample a state uniformly from the ball with center \a
             * near and radius \a distance. Return sample in \a state.
             * \note rho_s_ is a good choice for \a distance. */
            void sampleUniformNear(State *state, const State *near, double distance) override;

            /** \brief Sample a state uniformly from a normal distribution with
                given \a mean and \a stdDev. Return sample in \a state. */
            void sampleGaussian(State *state, const State *mean, double stdDev) override;

        protected:
            const ConstraintPtr constraint_;
        };

        /** \brief ValidStateSampler for use on an atlas. */
        class ConstrainedValidStateSampler : public ValidStateSampler
        {
        public:
            /** \brief Create a valid state sampler for the specifed space
             * information \a si. */
            ConstrainedValidStateSampler(const SpaceInformation *si);

            /** \brief Sample a valid state uniformly from the charted regions
             * of the manifold. Return sample in \a state. */
            bool sample(State *state) override;

            /** \brief Sample a valid state uniformly from the ball with center
             * \a near and radius \a distance. Return sample in \a state.
             * \note rho_s_ is a good choice for \a distance. */
            bool sampleNear(State *state, const State *near, double distance) override;

        private:
            /** \brief Underlying ordinary atlas state sampler. */
            ConstrainedStateSampler sampler_;

            /** \brief Constraint function. */
            const ConstraintPtr constraint_;
        };

        /** \brief Atlas-specific implementation of checkMotion(). */
        class ConstrainedMotionValidator : public MotionValidator
        {
        public:
            /** \brief Constructor. */
            ConstrainedMotionValidator(SpaceInformation *si);

            /** \brief Constructor. */
            ConstrainedMotionValidator(const SpaceInformationPtr &si);

            /** \brief Return whether we can step from \a s1 to \a s2 along the
             * manifold without collision. */
            bool checkMotion(const State *s1, const State *s2) const override;

            /** \brief Return whether we can step from \a s1 to \a s2 along the
             * manifold without collision. If not, return the last valid state
             * and its interpolation parameter in \a lastValid.
             * \note The interpolation parameter will not likely reproduce the
             * last valid state if used in interpolation since the distance
             * between the last valid state and \a s2 is estimated using the
             * ambient metric. */
            bool checkMotion(const State *s1, const State *s2, std::pair<State *, double> &lastValid) const override;

        private:
            /** \brief Space in which we check motion. */
            const ConstrainedStateSpace &ss_;
        };
        /// @endcond

        class ConstrainedStateSpace : public WrapperStateSpace
        {
        public:
            /** \brief A state in an atlas represented as a real vector in
             * ambient space and a chart that it belongs to. */
            class StateType : public WrapperStateSpace::StateType
            {
            public:
                /** \brief Construct state of size \a n. */
                StateType(State *state, unsigned int n) : WrapperStateSpace::StateType(state), n_(n)
                {
                }

                void setValues(double *location)
                {
                    values = location;
                }

                /** \brief View this state as a vector. */
                Eigen::Map<Eigen::VectorXd> vectorView() const
                {
                    return Eigen::Map<Eigen::VectorXd>(values, n_);
                }

                /** \brief View this state as a const vector. */
                Eigen::Map<const Eigen::VectorXd> constVectorView() const
                {
                    return Eigen::Map<const Eigen::VectorXd>(values, n_);
                }

                double *values;

            protected:
                const unsigned int n_;
            };

            /** \brief Construct an atlas with the specified dimensions. */
            ConstrainedStateSpace(const StateSpacePtr ambientSpace, const ConstraintPtr constraint);

            bool isMetricSpace() const override
            {
                return false;
            }

            /** \brief Check that the space referred to by the space information
             * \a si is, in fact, an AtlasStateSpace. */
            static void checkSpace(const SpaceInformation *si);

            /** \brief Sets the space information for this state space. Required for collision checking in manifold
             * traversal. */
            void setSpaceInformation(SpaceInformation *si);

            /** \brief Final setup for the space. */
            void setup() override;

            /** \brief Clear any allocated memory from the state space. */
            void clear();

            /** \brief Traverse the manifold from \a from toward \a to. Returns
             * true if we reached \a to, and false if we stopped early for any
             * reason, such as a collision or traveling too far. No collision
             * checking is performed if \a interpolate is true. If \a stateList
             * is not nullptr, the sequence of intermediates is saved to it,
             * including a copy of \a from, as well as the final state, which is
             * a copy of \a to if we reached \a to. Caller is responsible for
             * freeing states returned in \a stateList. */
            virtual bool traverseManifold(const State *from, const State *to, bool interpolate = false,
                                          std::vector<State *> *stateList = nullptr) const = 0;

            /** \brief Find the state between \a from and \a to at time \a t,
             * where \a t = 0 is \a from, and \a t = 1 is the final state
             * reached by traverseManifold(\a from, \a to, true, ...), which may
             * not be \a to. State returned in \a state. */
            void interpolate(const State *from, const State *to, double t, State *state) const;

            /** \brief Like interpolate(...), but uses the information about
             * intermediate states already supplied in \a stateList from a
             * previous call to followManifold(..., true, \a stateList). The
             * 'from' and 'to' states are the first and last elements \a
             * stateList. Assumes \a stateList contains at least two
             * elements. */
            virtual State *piecewiseInterpolate(const std::vector<State *> &stateList, double t) const;

            /** \brief Allocate a new state in this space. */
            virtual State *allocState() const override;

            /** \brief Allocate the default state sampler for this space. */
            StateSamplerPtr allocDefaultStateSampler() const override
            {
                return StateSamplerPtr(new ConstrainedStateSampler(this, space_->allocDefaultStateSampler()));
            }

            /** \brief Allocate the previously set state sampler for this space. */
            StateSamplerPtr allocStateSampler() const override
            {
                return StateSamplerPtr(new ConstrainedStateSampler(this, space_->allocStateSampler()));
            }

            /** \brief Set \a delta, the step size for traversing the manifold
             * and collision checking. Default 0.02. */
            void setDelta(const double delta)
            {
                if (delta <= 0)
                    throw ompl::Exception("ompl::base::ConstrainedStateSpace::setDelta(): "
                                          "delta must be positive.");
                delta_ = delta;

                /* if (setup_) */
                /*     setLongestValidSegmentFraction(delta_ / getMaximumExtent()); */
            }

            /** \brief Get delta. */
            double getDelta() const
            {
                return delta_;
            }

            /** \brief Returns the dimension of the ambient space. */
            unsigned int getAmbientDimension() const
            {
                return n_;
            }

            /** \brief Returns the dimension of the manifold. */
            unsigned int getManifoldDimension() const
            {
                return k_;
            }

            /** \brief Returns the constraint that defines the underlying manifold. */
            const ConstraintPtr getConstraint() const
            {
                return constraint_;
            }

            void setCaching(bool caching)
            {
                caching_ = caching;
            }

            bool getCaching() const
            {
                return caching_;
            }

            bool checkPath(geometric::PathGeometric &path, std::vector<unsigned int> *indices = nullptr) const;

            /** @} */

        protected:
            /** \brief SpaceInformation associated with this space. Required
             * for early collision checking in manifold traversal. */
            SpaceInformation *si_;

            /** \brief Constraint function that defines the manifold. */
            const ConstraintPtr constraint_;

            /** \brief Ambient space dimension. */
            const unsigned int n_;

            /** \brief Manifold dimension. */
            const unsigned int k_;

            /** \brief Step size when traversing the manifold and collision checking. */
            double delta_;

            /** \brief Whether setup() has been called. */
            bool setup_;

            bool caching_;
        };
    }
}

#endif
