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

#include "ompl/base/MotionValidator.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/Constraint.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/geometric/PathGeometric.h"

#include <eigen3/Eigen/Core>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::ConstrainedStateSpace. */
        OMPL_CLASS_FORWARD(ConstrainedStateSpace);

        /** \brief StateSampler for use on an atlas. */
        class ConstrainedStateSampler : public RealVectorStateSampler
        {
        public:
            /** \brief Create a sampler for the specified space information.
             * \note The underlying state space must be an AtlasStateSpace. */
            ConstrainedStateSampler(const SpaceInformation *si);

            /** \brief Create a sampler for the specified \a atlas space. */
            ConstrainedStateSampler(const ConstrainedStateSpace &ss);

            /** \brief Sample a state uniformly from the charted regions of the
             * manifold. Return sample in \a state. */
            void sampleUniform(State *state) override;

            /** \brief Sample a state uniformly from the ball with center \a
             * near and radius \a distance. Return sample in \a state.
             * \note rho_s_ is a good choice for \a distance. */
            void sampleUniformNear(State *state, const State *near, double distance) override;

            /** \brief Sample a state uniformly from a normal distribution with
                given \a mean and \a stdDev. Return sample in \a state. */
            void sampleGaussian(State *state, const State *mean, double stdDev) override;

        private:
            /** \brief Space on which to sample. */
            const ConstrainedStateSpace &ss_;
        };

        /** \brief ValidStateSampler for use on an atlas. */
        class ConstrainedValidStateSampler : public ValidStateSampler
        {
        public:
            /** \brief Create a valid state sampler for the specifed space
             * information \a si. */
            ConstrainedValidStateSampler(const SpaceInformation *si);

            /** \brief Destructor. */
            ~ConstrainedValidStateSampler();

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

            /** \brief Scratch state used in sampling. */
            State *scratch_;
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

        class ConstrainedStateSpace : public RealVectorStateSpace
        {
        public:
            /** \brief A state in an atlas represented as a real vector in
             * ambient space and a chart that it belongs to. */
            class StateType : public RealVectorStateSpace::StateType
            {
            public:
                /** \brief Construct state of size \a n. */
                StateType(const unsigned int n) : n_(n)
                {
                    // Do what RealVectorStateSpace::allocState() would have done.
                    values = new double[n_];
                }

                /** \brief Destructor. */
                virtual ~StateType(void)
                {
                    // Do what RealVectorStateSpace::freeState() would have done.
                    delete[] values;
                }

                /** \brief View this state as a vector. */
                Eigen::Map<Eigen::VectorXd> vectorView(void) const
                {
                    return Eigen::Map<Eigen::VectorXd>(values, n_);
                }

                /** \brief View this state as a const vector. */
                Eigen::Map<const Eigen::VectorXd> constVectorView(void) const
                {
                    return Eigen::Map<const Eigen::VectorXd>(values, n_);
                }

            protected:
                /** \brief Dimension of the real vector. */
                const unsigned int n_;
            };

            /** \brief Construct an atlas with the specified dimensions. */
            ConstrainedStateSpace(const StateSpacePtr ambientSpace, const ConstraintPtr constraint)
              : RealVectorStateSpace(ambientSpace->getDimension())
              , si_(nullptr)
              , ss_(std::move(ambientSpace))
              , constraint_(std::move(constraint))
              , n_(ambientSpace->getDimension())
              , k_(constraint_->getManifoldDimension())
              , delta_(0.02)
              , setup_(false)
            {
            }

            /** \brief Check that the space referred to by the space information
             * \a si is, in fact, an AtlasStateSpace. */
            static void checkSpace(const SpaceInformation *si);

            void setSpaceInformation(const SpaceInformationPtr &si);

            bool isMetricSpace() const override
            {
                return false;
            }

            /** \brief Final setup for the space. */
            void setup() override
            {
                if (setup_)
                    return;

                if (si_ == nullptr)
                    throw ompl::Exception("ompl::base::ConstrainedStateSpace::setup(): "
                                         "Must associate a SpaceInformation object to the ConstrainedStateSpace via "
                                         "setStateInformation() before use.");

                setup_ = true;
                setDelta(delta_);  // This makes some setup-related calls

                RealVectorStateSpace::setup();
            }

            void clear()
            {
            }

            /** \brief Set \a delta, the step size for traversing the manifold
             * and collision checking. Default 0.02. */
            void setDelta(const double delta)
            {
                if (delta <= 0)
                    throw ompl::Exception("ompl::base::ConstrainedStateSpace::setDelta(): "
                                          "delta must be positive.");
                delta_ = delta;

                if (setup_)
                {
                    setLongestValidSegmentFraction(delta_ / getMaximumExtent());
                }
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
             * reached by followManifold(\a from, \a to, true, ...), which may
             * not be \a to. State returned in \a state. */
            void interpolate(const State *from, const State *to, double t, State *state) const override;

            /** \brief Like interpolate(...), but uses the information about
             * intermediate states already supplied in \a stateList from a
             * previous call to followManifold(..., true, \a stateList). The
             * 'from' and 'to' states are the first and last elements \a
             * stateList. Assumes \a stateList contains at least two
             * elements. */
            unsigned int piecewiseInterpolate(const std::vector<State *> &stateList, double t, State *state) const;

            StateSamplerPtr allocDefaultStateSampler() const override
            {
                return StateSamplerPtr(new ConstrainedStateSampler(*this));
            }

            /** \brief Whether interpolation is symmetric. (Yes.) */
            bool hasSymmetricInterpolate() const override
            {
                return true;
            }

            void copyState(State *destination, const State *source) const override
            {
                StateType *adest = destination->as<StateType>();
                const StateType *asrc = source->as<StateType>();
                adest->vectorView() = asrc->constVectorView();
            }

            /** \brief Allocate a new state in this space. */
            State *allocState() const override
            {
                return new StateType(n_);
            }

            /** \brief Free \a state. Assumes \a state is of type
             * AtlasStateSpace::StateType. state. */
            void freeState(State *state) const override
            {
                StateType *const astate = state->as<StateType>();
                delete astate;
            }

            /** @} */

            /** @name Visualization and debug
             * @{ */

            /** @} */

        protected:
            /** \brief SpaceInformation associated with this space. */
            SpaceInformation *si_;

            /** \brief Ambient state space associated with this space. */
            const StateSpacePtr ss_;

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
        };
    }
}

#endif
