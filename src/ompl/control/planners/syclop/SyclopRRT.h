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

#ifndef OMPL_CONTROL_PLANNERS_SYCLOP_SYCLOPRRT_
#define OMPL_CONTROL_PLANNERS_SYCLOP_SYCLOPRRT_

#include "ompl/control/planners/syclop/Syclop.h"
#include "ompl/control/planners/syclop/Decomposition.h"
#include "ompl/control/planners/syclop/GridDecomposition.h"
#include "ompl/datastructures/NearestNeighbors.h"

namespace ompl
{
    namespace control
    {
        /** \brief SyclopRRT is Syclop with RRT as its low-level tree planner.
            \anchor cSyclopRRT
        */
        class SyclopRRT : public Syclop
        {
        public:
            /** \brief Constructor. Requires a Decomposition, which Syclop uses to create high-level leads. */
            SyclopRRT(const SpaceInformationPtr &si, const DecompositionPtr &d)
              : Syclop(si, d, "SyclopRRT"), regionalNN_(false)
            {
            }

            ~SyclopRRT() override
            {
                freeMemory();
            }

            void setup() override;
            void clear() override;
            void getPlannerData(base::PlannerData &data) const override;

            /** \brief If regionalNearestNeighbors is enabled, then when computing the closest Motion to a generated
               state
                in a given Region, SyclopRRT will perform a linear search over the current Region and its neighbors
               instead of
                querying a NearestNeighbors datastructure over the whole tree.
                This approach is enabled by default, and should be disabled if there exist Regions of the Decomposition
               that
                will be extremely densely populated with states - in such cases, querying a global NearestNeighbors
               datastructure will
                probably be faster. */
            void setRegionalNearestNeighbors(bool enabled)
            {
                regionalNN_ = enabled;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                regionalNN_ = false;
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }

        protected:
            Syclop::Motion *addRoot(const base::State *s) override;
            void selectAndExtend(Region &region, std::vector<Motion *> &newMotions) override;

            /** \brief Free the memory allocated by this planner. */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            base::StateSamplerPtr sampler_;
            DirectedControlSamplerPtr controlSampler_;
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;
            bool regionalNN_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_;
        };
    }
}
#endif
