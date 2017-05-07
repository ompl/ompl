/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Alexander Jiang */

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_RRTPLUS_
#define OMPL_GEOMETRIC_PLANNERS_RRT_RRTPLUS_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include <vector>

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor gRRT+
           @par Short description
           RRT+ is a RRT variant.
           @par External documentation
        */

        /** \brief RRT+ */
        class RRTPlus : public base::Planner
        {
        public:
            /** \brief Constructor */
            RRTPlus(const base::SpaceInformationPtr &si);

            ~RRTPlus() override;

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

            /** \brief Set the maximum amount of time to spend per subsearch, in seconds

                This parameter influences the performance of the planner. */
            void setSubsearchBound(double subsearchBound)
            {
                subsearchBound_ = subsearchBound;
            }

            /** \brief Get The maximum amount of time to spend per subsearch, in seconds */
            double getSubsearchBound() const
            {
                return subsearchBound_;
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

        protected:
            /** \brief Representation of a motion

                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:
                Motion() : state(nullptr), parent(nullptr)
                {
                }

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si) : state(si->allocState()), parent(nullptr)
                {
                }

                ~Motion() = default;

                /** \brief The state contained by the motion */
                base::State *state;

                /** \brief The parent motion in the exploration tree */
                Motion *parent;
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

<<<<<<< HEAD
            /** \brief State sampler */
            base::StateSamplerPtr sampler_;
=======
            /// @cond IGNORE
            /** \brief Forward declaration of ConstrainedSubspaceStateSampler */
            OMPL_CLASS_FORWARD(ConstrainedSubspaceStateSampler);
            /// @endcond

            /** \brief Constrained subspace state sampler */
            ConstrainedSubspaceStateSamplerPtr sampler_;
>>>>>>> 864c6839b03729ce569519ae8de92573a6953522

            /** \brief This is a problem-specific state sampler that only samples unconstrained
                components of a CompoundStateSpace. */
            class ConstrainedSubspaceStateSampler : public base::CompoundStateSampler
            {
            public:
                ConstrainedSubspaceStateSampler(const base::StateSpace *ss) : CompoundStateSampler(ss)
                {
                    name_ = "Constrained Subspace State Sampler";
                    this->constrainAllComponents();
                    samplerCount_ = samplers_.size();
                }

                void sampleUniform(base::State *state) override
                {
                    base::State **comps = state->as<base::CompoundState>()->components;
                    for (unsigned int i = 0; i < samplerCount_; ++i)
                        if (unconstrainedComponents_[i])
                            samplers_[i]->sampleUniform(comps[i]);
                }

                // We don't need these for RRT+.
                void sampleUniformNear(base::State*, const base::State*, const double) override
                {
                    throw Exception("ConstrainedSubspaceStateSampler::sampleUniformNear", "not implemented");
                }

                void sampleGaussian(base::State*, const base::State*, const double) override
                {
                    throw Exception("ConstrainedSubspaceStateSampler::sampleGaussian", "not implemented");
                }

                /** \brief Constrains all components

                    Used to initialize the vector of unconstrained components. */
                void constrainAllComponents()
                {
                    unconstrainedComponents_.clear();
                    for (unsigned int i = 0; i < samplerCount_; ++i)
                        unconstrainedComponents_.push_back(false);
                    assert(unconstrainedComponents_.size() == samplerCount_);
                }

                /** \brief Unconstrains the specified component */
                void unconstrainComponent(unsigned int i)
                {
                    assert(i >= 0 && i < samplerCount_);
                    unconstrainedComponents_[i] = true;
                }

                // should this be static? should it be here or as a protected method of RRTPlus?
<<<<<<< HEAD
                static base::StateSamplerPtr allocConstrainedSubspaceStateSampler(const base::StateSpace *ss)
=======
                static ConstrainedSubspaceStateSamplerPtr allocConstrainedSubspaceStateSampler(const base::StateSpace *ss)
>>>>>>> 864c6839b03729ce569519ae8de92573a6953522
                {
                    return std::make_shared<ConstrainedSubspaceStateSampler>(ss);
                }
            protected:
                std::string name_;
                RNG rng_;

                /** \brief Tracks which components of the compound state space are unconstrained */
                std::vector<bool> unconstrainedComponents_;
            private:
                /** \brief The number of samplers that are composed */
                unsigned int samplerCount_;
            };

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_;

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_;

            /** \brief The maximum amount of time to spend per subsearch, in seconds */
            double subsearchBound_;

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_;
        };
    }
}

#endif
