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

#ifndef OMPL_MULTILEVEL_PLANNERS_BundleSpace_SMLRIMPL_
#define OMPL_MULTILEVEL_PLANNERS_BundleSpace_SMLRIMPL_
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
        /** \brief Sparse Quotient-space roadMap Planner (SMLR) Algorithm*/
        class SMLRImpl : public BundleSpaceGraphSparse
        {
            using BaseT = BundleSpaceGraphSparse;

        public:
            SMLRImpl(const base::SpaceInformationPtr &si, BundleSpace *parent_);

            virtual ~SMLRImpl() override;

            /** \brief One iteration of SPARS using restriction sampling with
             * visibility regions */
            virtual void grow() override;

            /** \brief Check if number of consecutive failures is larger than
             * maxFailures_ and no solution exists.*/
            virtual bool isInfeasible() override;

            /** \brief Check if number of consecutive failures is larger than
             * maxFailures_ */
            virtual bool hasConverged() override;

            virtual void clear() override;

            /** \brief Return estimate of free state space coverage using the
             * formula $(1 / (M + 1))$ whereby $M$ is the number of
             * consecutive failures to sample a feasible state */
            virtual double getImportance() const override;

        protected:

            std::vector<base::State *> randomWorkStates_;

            bool isInfeasible_{false};
        };
    }  // namespace multilevel
}  // namespace ompl

#endif
