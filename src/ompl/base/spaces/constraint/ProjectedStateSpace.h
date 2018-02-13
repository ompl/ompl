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

#ifndef OMPL_BASE_SPACES_PROJECTED_STATE_SPACE_
#define OMPL_BASE_SPACES_PROJECTED_STATE_SPACE_

#include "ompl/base/MotionValidator.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/ValidStateSampler.h"
#include "ompl/base/Constraint.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/base/spaces/constraint/ConstrainedStateSpace.h"

#include <eigen3/Eigen/Core>

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        OMPL_CLASS_FORWARD(ProjectedStateSpace);
        /// @endcond

        /** \brief StateSampler for use on an atlas. */
        class ProjectedStateSampler : public WrapperStateSampler
        {
        public:
            ProjectedStateSampler(const ProjectedStateSpace *space, StateSamplerPtr sampler);

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

        protected:
            const ConstraintPtr constraint_;
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
            bool sample(State *state) override;

            /** \brief Sample a valid state uniformly from the ball with center
             * \a near and radius \a distance. Return sample in \a state.
             * \note rho_s_ is a good choice for \a distance. */
            bool sampleNear(State *state, const State *near, double distance) override;

        private:
            /** \brief Underlying ordinary atlas state sampler. */
            ProjectedStateSampler sampler_;

            /** \brief Constraint function. */
            const ConstraintPtr constraint_;
        };

        /** \brief State space encapsulating a planner-agnostic algorithm for
         * planning on a constraint manifold. */
        class ProjectedStateSpace : public ConstrainedStateSpace
        {
        public:
            /** \brief Construct an atlas with the specified dimensions. */
            ProjectedStateSpace(const StateSpacePtr ambientSpace, const ConstraintPtr constraint)
              : ConstrainedStateSpace(ambientSpace, constraint)
            {
                setName("Projected" + space_->getName());
            }

            /** \brief Destructor. */
            virtual ~ProjectedStateSpace() = default;

            /** \brief Check that the space referred to by the space information
             * \a si is, in fact, an AtlasStateSpace. */
            static void checkSpace(const SpaceInformation *si);

            /** \brief Allocate the default state sampler for this space. */
            StateSamplerPtr allocDefaultStateSampler() const override
            {
                return std::make_shared<ProjectedStateSampler>(this, space_->allocDefaultStateSampler());
            }

            /** \brief Allocate the previously set state sampler for this space. */
            StateSamplerPtr allocStateSampler() const override
            {
                return std::make_shared<ProjectedStateSampler>(this, space_->allocStateSampler());
            }

            /** \brief Traverse the manifold from \a from toward \a to. Returns
             * true if we reached \a to, and false if we stopped early for any
             * reason, such as a collision or traveling too far. No collision
             * checking is performed if \a interpolate is true. If \a stateList
             * is not nullptr, the sequence of intermediates is saved to it,
             * including a copy of \a from, as well as the final state, which is
             * a copy of \a to if we reached \a to. Caller is responsible for
             * freeing states returned in \a stateList. */
            bool traverseManifold(const State *from, const State *to, bool interpolate = false,
                                  std::vector<State *> *stateList = nullptr) const override;
        };
    }
}

#endif
