/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick
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
*   * Neither the name of Rutgers University nor the names of its
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

/* Authors: Zakary Littlefield */

#ifndef OMPL_CONTROL_PLANNERS_SST_SST_
#define OMPL_CONTROL_PLANNERS_SST_SST_

#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"

namespace ompl
{
    namespace control
    {
        /**
           @anchor cSST
           @par Short description
           \ref cSST "SST" (Stable Sparse RRT) is a asymptotically near-optimal incremental
           sampling-based motion planning algorithm for systems with dynamics. It makes use
           of random control inputs to perform a search for the best control inputs to explore
           the state space.
           @par External documentation
           Yanbo Li, Zakary Littlefield, Kostas E. Bekris, Sampling-based
           Asymptotically Optimal Sampling-based Kinodynamic Planning.
           [[PDF]](https://arxiv.org/abs/1407.2896)
        */
        class SST : public base::Planner
        {
        public:
            /** \brief Constructor */
            SST(const SpaceInformationPtr &si);

            ~SST() override;

            void setup() override;

            /** \brief Continue solving for some amount of time. Return true if solution was found. */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void getPlannerData(base::PlannerData &data) const override;

            /** \brief Clear datastructures. Call this function if the
                input data to the planner has changed and you do not
                want to continue planning */
            void clear() override;

            /** In the process of randomly selecting states in the state
                space to attempt to go towards, the algorithm may in fact
                choose the actual goal state, if it knows it, with some
                probability. This probability is a real number between 0.0
                and 1.0; its value should usually be around 0.05 and
                should not be too large. It is probably a good idea to use
                the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            /**
                \brief Set the radius for selecting nodes relative to random sample.

                This radius is used to mimic behavior of RRT* in that it promotes
                extending from nodes with good path cost from the root of the tree.
                Making this radius larger will provide higher quality paths, but has two
                major drawbacks; exploration will occur much more slowly and exploration
                around the boundary of the state space may become impossible. */
            void setSelectionRadius(double selectionRadius)
            {
                selectionRadius_ = selectionRadius;
            }

            /** \brief Get the selection radius the planner is using */
            double getSelectionRadius() const
            {
                return selectionRadius_;
            }

            /**
                \brief Set the radius for pruning nodes.

                This is the radius used to surround nodes in the witness set.
                Within this radius around a state in the witness set, only one
                active tree node can exist. This limits the size of the tree and
                forces computation to focus on low path costs nodes. If this value
                is too large, narrow passages will be impossible to traverse. In addition,
                children nodes may be removed if they are not at least this distance away
                from their parent nodes.*/
            void setPruningRadius(double pruningRadius)
            {
                pruningRadius_ = pruningRadius;
            }

            /** \brief Get the pruning radius the planner is using */
            double getPruningRadius() const
            {
                return pruningRadius_;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                witnesses_ = std::make_shared<NN<Motion *>>();
                setup();
            }

        protected:
            /** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:
                Motion() = default;

                /** \brief Constructor that allocates memory for the state and the control */
                Motion(const SpaceInformation *si)
                  : state_(si->allocState()), control_(si->allocControl())
                {
                }

                virtual ~Motion() = default;

                virtual base::State *getState() const
                {
                    return state_;
                }
                virtual Motion *getParent() const
                {
                    return parent_;
                }

                base::Cost accCost_{0};

                /** \brief The state contained by the motion */
                base::State *state_{nullptr};

                /** \brief The control contained by the motion */
                Control *control_{nullptr};

                /** \brief The number of steps_ the control is applied for */
                unsigned int steps_{0};

                /** \brief The parent motion in the exploration tree */
                Motion *parent_{nullptr};

                /** \brief Number of children */
                unsigned numChildren_{0};

                /** \brief If inactive, this node is not considered for selection.*/
                bool inactive_{false};
            };

            class Witness : public Motion
            {
            public:
                Witness() = default;

                Witness(const SpaceInformation *si) : Motion(si)
                {
                }
                base::State *getState() const override
                {
                    return rep_->state_;
                }
                Motion *getParent() const override
                {
                    return rep_->parent_;
                }

                void linkRep(Motion *lRep)
                {
                    rep_ = lRep;
                }

                /** \brief The node in the tree that is within the pruning radius.*/
                Motion *rep_{nullptr};
            };

            /** \brief Finds the best node in the tree withing the selection radius around a random sample.*/
            Motion *selectNode(Motion *sample);

            /** \brief Find the closest witness node to a newly generated potential node.*/
            Witness *findClosestWitness(Motion *node);

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state_, b->state_);
            }

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief Control sampler */
            ControlSamplerPtr controlSampler_;

            /** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
            const SpaceInformation *siC_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief A nearest-neighbors datastructure containing the tree of witness motions */
            std::shared_ptr<NearestNeighbors<Motion *>> witnesses_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{0.05};

            /** \brief The radius for determining the node selected for extension. */
            double selectionRadius_{0.2};

            /** \brief The radius for determining the size of the pruning region. */
            double pruningRadius_{0.1};

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The best solution we found so far. */
            std::vector<base::State *> prevSolution_;
            std::vector<Control *> prevSolutionControls_;
            std::vector<unsigned> prevSolutionSteps_;

            /** \brief The best solution cost we found so far. */
            base::Cost prevSolutionCost_;

            /** \brief The optimization objective. */
            base::OptimizationObjectivePtr opt_;
        };
    }
}

#endif