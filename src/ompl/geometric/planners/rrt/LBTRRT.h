/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Tel Aviv University
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
*   * Neither the name of the Tel Aviv University nor the names of its
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

/* Author: Oren Salzman, Sertac Karaman, Ioan Sucan, Mark Moll */

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_LBT_RRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_LBT_RRT_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/datastructures/DynamicSSSP.h"

#include <fstream>

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor gLBTRRT
           @par Short description
           \ref gLBTRRT "LBTRRT" (Lower Bound Tree RRT) is a near asymptotically-optimal
           incremental sampling-based motion planning algorithm. \ref gLBTRRT "LBTRRT"
           algorithm is guaranteed to converge to a solution that is within a constant
           factor of the optimal solution. The notion of optimality is with
           respect to the distance function defined on the state space
           we are operating on.

           @par External documentation
           O. Salzman and D. Halperin, Sampling-based
           Asymptotically near-optimal RRT for fast, high-quality, motion planning, 2013.
           [[PDF]](https://arxiv.org/abs/1308.0189)
        */

        /** \brief Lower Bound Tree Rapidly-exploring Random Trees */
        class LBTRRT : public base::Planner
        {
        public:
            /** \brief Constructor */
            LBTRRT(const base::SpaceInformationPtr &si);

            ~LBTRRT() override;

            void getPlannerData(base::PlannerData &data) const override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

            /** \brief Set the goal bias

                In the process of randomly selecting states in
                the state space to attempt to go towards, the
                algorithm may in fact choose the actual goal state, if
                it knows it, with some probability. This probability
                is a real number between 0.0 and 1.0; its value should
                usually be around 0.05 and should not be too large. It
                is probably a good idea to use the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            /** \brief Set the range the planner is supposed to use.

                This parameter greatly influences the runtime of the
                algorithm. It represents the maximum length of a
                motion to be added in the tree of motions. */
            void setRange(double distance)
            {
                maxDistance_ = distance;
            }

            /** \brief Get the range the planner is using */
            double getRange() const
            {
                return maxDistance_;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            void setup() override;

            /** \brief Set the apprimation factor */
            void setApproximationFactor(double epsilon)
            {
                epsilon_ = epsilon;
            }

            /** \brief Get the apprimation factor */
            double getApproximationFactor() const
            {
                return epsilon_;
            }

            ///////////////////////////////////////
            // Planner progress property functions
            std::string getIterationCount() const
            {
                return std::to_string(iterations_);
            }
            std::string getBestCost() const
            {
                return std::to_string(bestCost_);
            }

        protected:
            /** \brief Representation of a motion

                a motion is a simultunaeous represntation of the two trees used by LBT-RRT
                a lower bound tree named Tlb and an approximaion tree named Tapx. */
            class Motion
            {
            public:
                Motion() = default;

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si)
                  : state_(si->allocState())
                {
                }

                ~Motion() = default;

                /** \brief The state contained by the motion */
                base::State *state_{nullptr};
                /** \brief unique id of the motion */
                std::size_t id_;
                /** \brief The lower bound cost of the motion
                while it is stored in the lowerBoundGraph_ and this may seem redundant,
                the cost in lowerBoundGraph_ may change causing ordering according to it
                inconsistencies
                */
                double costLb_;
                /** \brief The parent motion in the approximation tree */
                Motion *parentApx_{nullptr};
                /** \brief The approximation cost */
                double costApx_{0.0};
                /** \brief The children in the approximation tree */
                std::vector<Motion *> childrenApx_;
            };

            /** \brief comparator  - metric is the cost to reach state via a specific state*/
            struct IsLessThan
            {
                IsLessThan(LBTRRT *plannerPtr, Motion *motion) : plannerPtr_(plannerPtr), motion_(motion)
                {
                }

                bool operator()(const Motion *motionA, const Motion *motionB)
                {
                    double costA = motionA->costLb_;
                    double costB = motionB->costLb_;

                    double distA = plannerPtr_->distanceFunction(motionA, motion_);
                    double distB = plannerPtr_->distanceFunction(motionB, motion_);

                    return costA + distA < costB + distB;
                }
                LBTRRT *plannerPtr_;
                Motion *motion_;
            };  // IsLessThan

            /** \brief comparator  - metric is the lower bound cost*/
            struct IsLessThanLB
            {
                IsLessThanLB(LBTRRT *plannerPtr) : plannerPtr_(plannerPtr)
                {
                }

                bool operator()(const Motion *motionA, const Motion *motionB) const
                {
                    return motionA->costLb_ < motionB->costLb_;
                }
                LBTRRT *plannerPtr_;
            };  // IsLessThanLB

            /** \brief consider an edge for addition to the roadmap*/
            void considerEdge(Motion *parent, Motion *child, double c);

            /** \brief lazily update the parent in the approximation tree without updating costs to cildren*/
            double lazilyUpdateApxParent(Motion *child, Motion *parent);

            /** \brief update the child cost of the approximation tree */
            void updateChildCostsApx(Motion *m, double delta);

            /** \brief remove motion from its parent in the approximation tree*/
            void removeFromParentApx(Motion *m);

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state_, b->state_);
            }

            /** \brief local planner */
            bool checkMotion(const Motion *a, const Motion *b)
            {
                return checkMotion(a->state_, b->state_);
            }
            /** \brief local planner */
            bool checkMotion(const base::State *a, const base::State *b)
            {
                return si_->checkMotion(a, b);
            }

            /** \brief get motion from id */
            Motion *getMotion(std::size_t i)
            {
                return idToMotionMap_[i];
            }

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief A graph of motions Glb*/
            DynamicSSSP lowerBoundGraph_;

            /** \brief mapping between a motion id and the motion*/
            std::vector<Motion *> idToMotionMap_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{.05};

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{0.};

            /** \brief approximation factor*/
            double epsilon_{.4};

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};

            //////////////////////////////
            // Planner progress properties
            /** \brief Number of iterations the algorithm performed */
            unsigned int iterations_{0u};
            /** \brief Best cost found so far by algorithm */
            double bestCost_;
        };
    }
}

#endif  // OMPL_GEOMETRIC_PLANNERS_RRT_LBT_RRT_
