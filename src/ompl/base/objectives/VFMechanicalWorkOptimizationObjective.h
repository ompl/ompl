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

#ifndef OMPL_BASE_OBJECTIVES_VF_MECHANICAL_WORK_OPTIMIZATION_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_VF_MECHANICAL_WORK_OPTIMIZATION_OBJECTIVE_

#include <utility>

#include "ompl/base/objectives/MechanicalWorkOptimizationObjective.h"
#include "ompl/geometric/planners/rrt/VFRRT.h"

namespace ompl
{
    namespace base
    {
        /**
         * Optimization objective that computes mechanical work between two states by following a vector field.
         */
        class VFMechanicalWorkOptimizationObjective : public ompl::base::MechanicalWorkOptimizationObjective
        {
        public:
            /** Constructor. */
            VFMechanicalWorkOptimizationObjective(const ompl::base::SpaceInformationPtr &si,
                                                  geometric::VFRRT::VectorField vf)
              : ompl::base::MechanicalWorkOptimizationObjective(si), vf_(std::move(vf))
            {
            }

            /** Assume we can always do better. */
            bool isSatisfied(ompl::base::Cost) const override
            {
                return false;
            }

            /** Compute mechanical work between two states. */
            ompl::base::Cost motionCost(const ompl::base::State *s1, const ompl::base::State *s2) const override
            {
                const base::StateSpacePtr &space = si_->getStateSpace();
                // Per equation 7 in the paper
                Eigen::VectorXd f = vf_(s2);
                unsigned int vfdim = f.size();
                Eigen::VectorXd qprime(vfdim);

                for (unsigned int i = 0; i < vfdim; i++)
                    qprime[i] = *space->getValueAddressAtIndex(s2, i) - *space->getValueAddressAtIndex(s1, i);

                // Don't included negative work
                double positiveCostAccrued = std::max(-(f.dot(qprime)), 0.);
                return ompl::base::Cost(positiveCostAccrued + pathLengthWeight_ * si_->distance(s1, s2));
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
