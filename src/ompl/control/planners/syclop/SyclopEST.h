/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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

/* Author: Matt Maly */

#ifndef OMPL_CONTROL_PLANNERS_SYCLOP_SYCLOPEST_
#define OMPL_CONTROL_PLANNERS_SYCLOP_SYCLOPEST_

#include "ompl/control/planners/syclop/Syclop.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/control/planners/syclop/GridDecomposition.h"

namespace ompl
{
    namespace control
    {
        /** \brief SyclopEST is Syclop with EST as its low-level tree planner.
            \anchor cSyclopEST
        */
        class SyclopEST : public Syclop
        {
        public:
            /** \brief Constructor. Requires a Decomposition, which Syclop uses to create high-level leads. */
            SyclopEST(const SpaceInformationPtr &si, const DecompositionPtr &d) : Syclop(si, d, "SyclopEST")
            {
            }

            ~SyclopEST() override
            {
                freeMemory();
            }

            void setup() override;
            void clear() override;
            void getPlannerData(base::PlannerData &data) const override;

        protected:
            Syclop::Motion *addRoot(const base::State *s) override;
            void selectAndExtend(Region &region, std::vector<Motion *> &newMotions) override;

            /** \brief Free the memory allocated by this planner. */
            void freeMemory();

            base::StateSamplerPtr sampler_;
            ControlSamplerPtr controlSampler_;
            std::vector<Motion *> motions_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_;
        };
    }
}
#endif
