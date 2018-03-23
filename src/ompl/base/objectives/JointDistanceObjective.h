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

/* Authors: John Schulman, Bryce Willey */

#ifndef OMPL_BASE_OBJECTIVES_JOINT_DISTANCE_OBJECTIVE_
#define OMPL_BASE_OBJECTIVES_JOINT_DISTANCE_OBJECTIVE_

#include "ompl/base/objectives/ConvexifiableObjective.h"
#include "ompl/trajopt/sco_fwd.h"
#include "ompl/trajopt/modeling.h"
#include "ompl/trajopt/typedefs.h"

namespace ompl
{
    namespace base
    {
        /**
         * \brief Encourages minimum length paths by using the sum of squared displacements.
         */
        class JointDistCost : public sco::Cost
        {
        public:
            JointDistCost(const sco::VarArray& traj);
            /**
             * Turns this cost into a convex cost.
             */
            sco::ConvexObjectivePtr convex(const std::vector<double>& x, sco::Model* model);

            /**
             * Gets the cost value of a specific trajectory.
             */
            double value(const std::vector<double>&);
        private:
            sco::VarArray vars_;
            sco::QuadExpr expr_;
        };

        class JointDistanceObjective : public ConvexifiableObjective
        {
        public:
            JointDistanceObjective(const SpaceInformationPtr &si);

            /** \brief Returns identity cost. */
            Cost stateCost(const State *s) const override;

            Cost motionCost(const State *s1, const State *s2) const override;

        //protected:
            sco::CostPtr toCost(sco::OptProbPtr problem) override;
        };

        /**
         * \brief Encourages smooth paths by using the double finite difference.
         */
        class JointAccelCost : public sco::Cost
        {
        public:
            JointAccelCost(const sco::VarArray& traj);

            sco::ConvexObjectivePtr convex(const std::vector<double>&x, sco::Model* model);

            double value(const std::vector<double>&);
        private:
            sco::VarArray vars_;
            sco::QuadExpr expr_;
        };

        class JointAccelObjective : public ConvexifiableObjective
        {
        public:
            JointAccelObjective(const SpaceInformationPtr &si);

            /** \brief Returns identity cost. */
            Cost stateCost(const State *s) const override;

            Cost motionCost(const State *s1, const State *s2) const override;

        //protected:
            sco::CostPtr toCost(sco::OptProbPtr problem) override;
        };
    }
}

#endif
