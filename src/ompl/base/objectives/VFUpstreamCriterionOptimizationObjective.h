/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Caleb Voss and Wilson Beebe
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
*   * Neither the name of the Willow Garage nor the names of its
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

/* Authors: Caleb Voss, Wilson Beebe */

#ifndef OMPL_BASE_OBJECTIVES_VF_UPSTREAM_CRITERION_OPTIMIZATION_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_VF_UPSTREAM_CRITERION_OPTIMIZATION_OBJECTIVE_

#include <utility>

#include "ompl/base/OptimizationObjective.h"
#include "ompl/geometric/planners/rrt/VFRRT.h"

namespace ompl
{
    namespace base
    {
        /**
         * Optimization objective that computes the upstream criterion between two states.
         */
        class VFUpstreamCriterionOptimizationObjective : public ompl::base::OptimizationObjective
        {
        public:
            /** Constructor. */
            VFUpstreamCriterionOptimizationObjective(const ompl::base::SpaceInformationPtr &si,
                                                     geometric::VFRRT::VectorField vf)
              : ompl::base::OptimizationObjective(si), vf_(std::move(vf))
            {
                description_ = "Upstream Criterion";
            }

            /** Assume we can always do better. */
            bool isSatisfied(ompl::base::Cost) const override
            {
                return false;
            }

            /** \brief Returns a cost with a value of 0. */
            Cost stateCost(const State *) const override
            {
                return Cost(0.);
            }

            /** Compute upstream criterion between two states. */
            ompl::base::Cost motionCost(const State *s1, const State *s2) const override
            {
                const base::StateSpacePtr &space = si_->getStateSpace();
                // Per equation 1 in the paper, Riemann approximation on the left
                unsigned int vfdim = space->getValueLocations().size();
                Eigen::VectorXd qprime(vfdim);
                unsigned int numSegments = space->validSegmentCount(s1, s2);
                std::vector<ompl::base::State *> interp;

                for (unsigned int i = 0; i < vfdim; i++)
                    qprime[i] = *space->getValueAddressAtIndex(s2, i) - *space->getValueAddressAtIndex(s1, i);
                qprime.normalize();
                si_->getMotionStates(s1, s2, interp, numSegments - 1, true, true);
                double cost = 0;
                for (unsigned int i = 0; i < interp.size() - 1; i++)
                {
                    Eigen::VectorXd f = vf_(interp[i]);
                    cost += si_->distance(interp[i], interp[i + 1]) * (f.norm() - f.dot(qprime));
                    si_->freeState(interp[i]);
                }
                si_->freeState(interp[interp.size() - 1]);
                return ompl::base::Cost(cost);
            }

            bool isSymmetric() const override
            {
                return false;
            }

        protected:
            /** VectorField associated with the space. */
            geometric::VFRRT::VectorField vf_;
        };
    }
}

#endif
