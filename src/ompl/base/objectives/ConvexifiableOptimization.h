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

/* Author: Bryce Willey */

#ifndef OMPL_BASE_OBJECTIVES_CONVEXIFIABLE_OPTIMIZATION_
#define OMPL_BASE_OBJECTIVES_CONVEXIFIABLE_OPTIMIZATION_

#include "ompl/base/OptimizationObjective.h"
#include "ompl/trajopt/modeling.h"

namespace ompl
{
    namespace base
    {
        /** \brief An optimization objective that can be turned into an analytical convex
         *  approximation.
         *
         *  In the context of the non-convex optimization problem:
         *    minimize f(x)
         *    subject to
         *       g_i(x) <= 0
         *      h_i(x) = 0
         *
         *  This optimization can function either as a cost (f(x) above), or as a constraint, 
         *  either an equality constraint (h(x)) or an inequality constraint (g(x)).
         *
         *  Subclasses handle the differences between a cost or a constraint.
         */
        OMPL_CLASS_FORWARD(ConvexifiableOptimization);

        class ConvexifiableOptimization : public OptimizationObjective
        {
        public:
            ConvexifiableOptimization(const SpaceInformationPtr &si) : OptimizationObjective(si) {}

            virtual void addToProblem(sco::OptProbPtr problem) = 0;
        };

        OMPL_CLASS_FORWARD(MultiConvexifiableOptimization);

        /** \brief A composed version of a convexifiable optimization. */
        class MultiConvexifiableOptimization : public ConvexifiableOptimization
        {
        public:

            MultiConvexifiableOptimization(const SpaceInformationPtr &si) : ConvexifiableOptimization(si) {}

            ompl::base::Cost stateCost(const ompl::base::State* state) const {
                Cost c = identityCost();
                for (ConvexifiableOptimizationPtr opt : components_) {
                    c = Cost(c.value() + opt->stateCost(state).value());
                }
                return c;
            }

            ompl::base::Cost motionCost(const State *s1, const State *s2) const {
                Cost c = identityCost();
                for (ConvexifiableOptimizationPtr opt : components_) {
                    c = Cost(c.value() + opt->motionCost(s1, s2).value());
                }
                return c;
            }

            /** \brief Adds a new objective for this multiobjective. A weight must also be specified for specifying
             * importance of this objective in planning. */
            void addObjective(const ConvexifiableOptimizationPtr &objective)
            {
                components_.push_back(objective);
            }

            void addToProblem(sco::OptProbPtr problem)
            {
                for (ConvexifiableOptimizationPtr c : components_) {
                    c->addToProblem(problem);
                }
            }
        private:
            std::vector<ConvexifiableOptimizationPtr> components_;
        };
    }
}

#endif
