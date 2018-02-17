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

#ifndef OMPL_GEOMETRIC_PLANNERS_TRAJOPT_TRAJOPT_
#define OMPL_GEOMETRIC_PLANNERS_TRAJOPT_TRAJOPT_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/geometric/planners/trajopt/OmplOptProb.h"
#include "ompl/trajopt/typedefs.h"
#include "ompl/trajopt/optimizers.h"

namespace ompl
{
    namespace geometric
    {
        /**
        @anchor TrajOpt
        @par Short Description
        \ref gTrajOpt is an optimization based approach for motion planning.
        It uses sequential convex optimization (sco) to locally optimize a
        trajectory that might in collision.

        @par External Documentation
        John Schulman, Yan Duan, Jonathan Ho, Alex Lee, Ibrahim Awwal,
        Henry Bradlow, Jia Pan, Sachin Patil, Ken Goldberg and Pieter Abbeel
        Motion Planning with sequential convex optimization and convex collision
        checking, International Journal of Robotics Research, 1-20, 2014.
        http://journals.sagepub.com/doi/abs/10.1177/0278364914528132

        Based on code from http://rll.berkeley.edu/trajopt
        */

        class TrajOpt : public ompl::base::Planner
        {
        public:
            // TODO(Bryce): assuming that constraints are passed in as a part of
            // the SpaceInformation.
            TrajOpt(const ompl::base::SpaceInformationPtr &si);

            ~TrajOpt() override;

            base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override;

            void clear() override;

            void setup() override;

            void setTimeStepCount(int nSteps)
            {
                nSteps_ = nSteps;
            }

            int getTimeStepCount()
            {
                return nSteps_;
            }

            void setMaxIterations(int maxIter)
            {
                maxIter_ = maxIter;
            }

            int getMaxIterations()
            {
                return maxIter_;
            }

            void setInitialPenaltyCoef(double initPenaltyCoef)
            {
                initPenaltyCoef_ = initPenaltyCoef;
            }

            double getInitialPenaltyCoef()
            {
                return initPenaltyCoef_;
            }

            void setMinApproxImproveFraction(double minApproxImproveFrac)
            {
                minApproxImproveFrac_ = minApproxImproveFrac;
            }

            double getMinApproxImproveFraction()
            {
                return minApproxImproveFrac_;
            }

            void setInitialTrajectory(ompl::geometric::PathGeometric inPath);

        protected:
            ompl::base::PathPtr trajFromTraj2Ompl(trajopt::TrajArray traj);

            ompl::base::PlannerStatus constructOptProblem();

            ompl::base::PlannerStatus optimize(const ompl::base::PlannerTerminationCondition &ptc);

            void plotCallback(std::vector<double>& x);

            /** \brief The number of time steps/waypoints in the optimized trajectory.
                Smaller means quicker optimization, larger means finer trajectories. */
            size_t nSteps_{50};

            /** \brief The starting penalty coefficient, goes to infinity as the algorithm progresses. */
            double initPenaltyCoef_{20};

            /** \brief The starting size of the trust region. */
            double initTrustBoxSize_{0.1};

            double minApproxImproveFrac_{0.001};
            double maxIter_{40};

            sco::BasicTrustRegionSQP *sqpOptimizer;

            OmplOptProbPtr problem_;

            FILE* fd;
        };
    }
}

#endif
