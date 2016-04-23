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

#ifndef V_F_UPSTREAM_CRITERION_OPTIMIZATION_OBJECTIVE_
#define V_F_UPSTREAM_CRITERION_OPTIMIZATION_OBJECTIVE_

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
            VFUpstreamCriterionOptimizationObjective(const ompl::base::SpaceInformationPtr &si, const geometric::VFRRT::VectorField *vf)
                : ompl::base::OptimizationObjective(si), vf(*vf), sstate1(si_), sstate2(si_), d(sstate1.reals().size()), qprime(d)
            {
                description_ = "Upstream Criterion";
            }
    
            /** Assume we can always do better. */
            bool isSatisfied(ompl::base::Cost c) const
            {
                return false;
            }

            /** \brief Returns a cost with a value of 0. */
            virtual Cost stateCost(const State *s) const
            {
                return Cost(0.);
            }

            /** Compute upstream criterion between two states. */
            ompl::base::Cost motionCost(const State *s1, const State *s2) const
            {
                // Per equation 1 in the paper, Riemann approximation on the left
                sstate1 = s1;
                sstate2 = s2;
                for (int i = 0; i < d; i++)
                {
                    qprime[i] = sstate2[i] - sstate1[i];
                }
                int segments = std::ceil(si_->distance(s1,s2) / si_->getStateValidityCheckingResolution());
                si_->getMotionStates(s1, s2, interp, segments-1, true, true);
                qprime.normalize();
                double cost = 0;
                for (int i = 0; i < segments; i++)
                {
                    Eigen::VectorXd f = vf(interp[i]);
                    cost += si_->distance(interp[i],interp[i+1])*(f.norm() - f.dot(qprime));
                    si_->freeState(interp[i]);
                }
                si_->freeState(interp[interp.size()-1]);
                interp.clear();
                return ompl::base::Cost(cost);
            }
    
            bool isSymmetric(void) const
            {
                return false;
            }

        private:
    
            /** VectorField associated with the space. */
            const geometric::VFRRT::VectorField &vf;
    
            /** Variables used in computation that we keep around to save on allocations. */
            mutable ompl::base::ScopedState<> sstate1;
            mutable ompl::base::ScopedState<> sstate2;
            const int d;
            mutable Eigen::VectorXd qprime;
            mutable std::vector<ompl::base::State*> interp;
    
        };

    }
}

#endif
