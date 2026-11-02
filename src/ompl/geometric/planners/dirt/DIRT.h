/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2025
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
*   * Neither the name of the copyright holder nor the names of its
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

#ifndef OMPL_GEOMETRIC_PLANNERS_DIRT_DIRT_
#define OMPL_GEOMETRIC_PLANNERS_DIRT_DIRT_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include <limits>

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor gDIRT
           @par Short description
           \ref gDIRT "DIRT" (Dominance-Informed Region Tree) is an asymptotically 
           near-optimal sampling-based motion planning algorithm that uses dominance 
           regions and heuristic guidance for efficient exploration.
           
           @par External documentation
           Based on the DIRT implementation in ML4KP.
        */
        class DIRT : public base::Planner
        {
        public:
            DIRT(const base::SpaceInformationPtr &si);
            ~DIRT() override;

            void setup() override;
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
            void getPlannerData(base::PlannerData &data) const override;
            void clear() override;

            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            double getRange() const
            {
                return maxDistance_;
            }

            void setBlossomNumber(unsigned int blossomNumber)
            {
                blossomNumber_ = blossomNumber;
            }

            unsigned int getBlossomNumber() const
            {
                return blossomNumber_;
            }

            void setUsePruning(bool usePruning)
            {
                usePruning_ = usePruning;
            }

            bool getUsePruning() const
            {
                return usePruning_;
            }

            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }

        protected:
            class Motion
            {
            public:
                Motion() = default;

                Motion(const base::SpaceInformationPtr &si)
                  : state_(si->allocState())
                {
                }

                ~Motion() = default;

                base::State *state_{nullptr};
                Motion *parent_{nullptr};
                unsigned numChildren_{0};
                bool inactive_{false};

                base::Cost accCost_{0.};
                base::Cost estimatedCostToGoal_{0.};
                double dirRadius_{0.};
                unsigned blossomNumber_{0};
                
                std::vector<base::State *> edgeGenerators_;
            };

            Motion *selectNode(Motion *sample);
            double computeDirectionalRadius(Motion *newMotion, Motion *parent, 
                                           const std::vector<Motion *> &nearby);
            base::State *monteCarloProp(Motion *m);
            base::Cost estimateCostToGoal(const base::State *state) const;
            void freeMemory();
            void pruneNode(Motion *node);

            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state_, b->state_);
            }

            base::StateSamplerPtr sampler_;
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;
            
            double maxDistance_{5.};
            unsigned blossomNumber_{5};
            bool usePruning_{true};
            
            double maxRadius_{0.};
            bool childExtension_{true};
            Motion *previousChild_{nullptr};
            
            RNG rng_;
            std::vector<base::State *> prevSolution_;
            base::Cost prevSolutionCost_{std::numeric_limits<double>::quiet_NaN()};
            base::OptimizationObjectivePtr opt_;
        };
    }
}

#endif
