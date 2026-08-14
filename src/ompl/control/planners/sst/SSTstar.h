/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Worcester Polytechnic Institute ELPIS Lab
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
 *   * Neither the name of the Worcester Polytechnic Institute nor the
 *     names of its contributors may be used to endorse or promote
 *     products derived from this software without specific prior
 *     written permission.
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

/* Authors: Zhuoyun Zhong */
/* Adapted from: ompl/control/planners/sst/SST.h by Zakary Littlefield */

#ifndef OMPL_CONTROL_PLANNERS_SST_SSTSTAR_
#define OMPL_CONTROL_PLANNERS_SST_SSTSTAR_

#include "ompl/control/planners/sst/SST.h"

namespace ompl
{
    namespace control
    {
        /**
           @anchor cSSTstar
           @par Short description
           \ref cSSTstar "SST*" (Stable Sparse RRT) is a asymptotically optimal incremental
           sampling-based motion planning algorithm for systems with dynamics. It makes use
           of random control inputs to perform a search for the best control inputs to explore
           the state space.
           @par External documentation
           Yanbo Li, Zakary Littlefield, Kostas E. Bekris, Sampling-based
           Asymptotically Optimal Sampling-based Kinodynamic Planning.
           [[PDF]](https://arxiv.org/abs/1407.2896)
        */
        class SSTstar : public SST
        {
        public:
            /** \brief Constructor */
            SSTstar(const SpaceInformationPtr &si);

            /** \brief Continue solving for some amount of time. Return true if solution was found. */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            /** \brief Set the initial search iteration before radius starts to shrink */
            void setInitialIteration(unsigned int initialN)
            {
                initialN_ = initialN;
            }

            /** \brief Get the initial search iteration before radius starts to shrink */
            unsigned int getInitialIteration() const
            {
                return initialN_;
            }

            /** \brief Set the shrink factor. */
            void setShrinkFactor(double shrinkFactor)
            {
                shrinkFactor_ = shrinkFactor;
            }

            /** \brief Get the shrink factor. */
            double getShrinkFactor() const
            {
                return shrinkFactor_;
            }

        protected:
            /** \brief The initial search iteration before radius starts to shrink */
            unsigned int initialN_{1000};

            /** \brief Radius shrink factor */
            double shrinkFactor_{0.99};
        };
    }  // namespace control
}  // namespace ompl

#endif