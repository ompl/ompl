/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020,
 *  Max Planck Institute for Intelligent Systems (MPI-IS).
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
 *   * Neither the name of the MPI-IS nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
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

/* Author: Andreas Orthey, Sohaib Akbar */

#ifndef OMPL_MULTILEVEL_PLANNERS_BundleSpace_SPQRIMPL_
#define OMPL_MULTILEVEL_PLANNERS_BundleSpace_SPQRIMPL_
#include <ompl/multilevel/datastructures/BundleSpaceGraphSparse.h>
#include <ompl/datastructures/PDF.h>

namespace ompl
{
    namespace base
    {
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }
    namespace multilevel
    {
        /** \brief Sparse Quotient-space roadMap Planner (SPQR) Algorithm*/
        class SPQRImpl : public BundleSpaceGraphSparse
        {
            using BaseT = BundleSpaceGraphSparse;

        public:
            SPQRImpl(const ompl::base::SpaceInformationPtr &si, BundleSpace *parent_);

            virtual ~SPQRImpl() override;

            /** \brief One iteration of RRT with adjusted sampling function */
            virtual void grow() override;

            /** \brief sample random node from Probabilty density function*/
            // void expand();

            virtual bool isInfeasible() override;

            // void connectNeighbors(Configuration *x);

        protected:
            /** \brief Maximum failures limit for terminating the algorithm similar to SPARS */
            unsigned int maxFailures_{1000u};

            /** \brief for different ratio of expand vs grow 1:5*/
            unsigned int iterations_{0};

            double kPRMStarConstant_{0};

            std::vector<base::State *> randomWorkStates_;
        };
    }  // namespace multilevel
}  // namespace ompl

#endif
