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
#include "ompl/base/spaces/constraint/ConstrainedStateSpace.h"

namespace ompl
{
    namespace base
    {
        /// @cond IGNORE
        /** \brief Forward declaration of ompl::base::ProjectedStateSpace */
        OMPL_CLASS_FORWARD(ProjectedStateSpace);
        /// @endcond

        /** \brief StateSampler for use for a projection-based state space. */
        class ProjectedStateSampler : public WrapperStateSampler
        {
        public:
            /** \brief Constructor. */
            ProjectedStateSampler(const ProjectedStateSpace *space, StateSamplerPtr sampler);

            /** \brief Sample a state uniformly in ambient space and project to
             * the manifold. Return sample in \a state. */
            void sampleUniform(State *state) override;

            /** \brief Sample a state uniformly from the ball with center \a
             * near and radius \a distance in ambient space and project to the
             * manifold. Return sample in \a state. */
            void sampleUniformNear(State *state, const State *near, double distance) override;

            /** \brief Sample a state uniformly from a normal distribution with
                given \a mean and \a stdDev in ambient space and project to the
                manifold. Return sample in \a state. */
            void sampleGaussian(State *state, const State *mean, double stdDev) override;

        protected:
            /** \brief Constraint. */
            const ConstraintPtr constraint_;
        };

        /**
           @anchor gProject
           @par Short Description
           ProjectedStateSpace implements a projection-based methodology for constrained sampling-based planning, where
           points in ambient space are \e projected onto the constraint manifold via a projection operator, which in
           this case is implemented as a Newton's method within the Constraint.

           @par External Documentation
           For more information on constrained sampling-based planning using projection-based methods, see the following review paper.
           The section on projection-based methods cites most of the relevant literature.

           Z. Kingston, M. Moll, and L. E. Kavraki, “Sampling-Based Methods for Motion Planning with Constraints,”
           Annual Review of Control, Robotics, and Autonomous Systems, 2018. DOI:
           <a href="http://dx.doi.org/10.1146/annurev-control-060117-105226">10.1146/annurev-control-060117-105226</a>
           <a href="http://kavrakilab.org/publications/kingston2018sampling-based-methods-for-motion-planning.pdf">[PDF]</a>.
        */

        /** \brief ConstrainedStateSpace encapsulating a projection-based
         * methodology for planning with constraints. */
        class ProjectedStateSpace : public ConstrainedStateSpace
        {
        public:
            /** \brief Construct an atlas with the specified dimensions. */
            ProjectedStateSpace(const StateSpacePtr &ambientSpace, const ConstraintPtr &constraint)
              : ConstrainedStateSpace(ambientSpace, constraint)
            {
                setName("Projected" + space_->getName());
            }

            /** \brief Destructor. */
            ~ProjectedStateSpace() override = default;

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
             * checking is performed if \a interpolate is true. If \a geodesic
             * is not nullptr, the sequence of intermediates is saved to it,
             * including a copy of \a from, as well as the final state, which is
             * a copy of \a to if we reached \a to. Caller is responsible for
             * freeing states returned in \a geodesic.*/
            bool discreteGeodesic(const State *from, const State *to, bool interpolate = false,
                                  std::vector<State *> *geodesic = nullptr) const override;
        };
    }
}

#endif
