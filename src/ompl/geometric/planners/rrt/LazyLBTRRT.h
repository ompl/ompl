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

/* Author: Oren Salzman, Mark Moll */

#ifndef OMPL_CONTRIB_LAZY_LBTRRT_
#define OMPL_CONTRIB_LAZY_LBTRRT_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/datastructures/LPAstarOnGraph.h"

#include <fstream>
#include <vector>
#include <tuple>
#include <cassert>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>

namespace ompl
{
    namespace geometric
    {
        /** \brief Rapidly-exploring Random Trees */
        class LazyLBTRRT : public base::Planner
        {
        public:
            /** \brief Constructor */
            LazyLBTRRT(const base::SpaceInformationPtr &si);

            ~LazyLBTRRT() override;

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
            /** \brief Representation of a motion */
            class Motion
            {
            public:
                Motion() = default;

                /** \brief Constructor that allocates memory for the state */
                Motion(const base::SpaceInformationPtr &si) : state_(si->allocState())
                {
                }

                ~Motion() = default;

                /** \brief The id of the motion */
                std::size_t id_;

                /** \brief The state contained by the motion */
                base::State *state_{nullptr};
            };

            using WeightProperty = boost::property<boost::edge_weight_t, double>;
            using BoostGraph = boost::adjacency_list<boost::vecS,         // container type for the out edge list
                                                     boost::vecS,         // container type for the vertex list
                                                     boost::undirectedS,  // directedS / undirectedS / bidirectionalS.
                                                     std::size_t,         // vertex properties
                                                     WeightProperty       // edge properties
                                                     >;

            friend class CostEstimatorApx;  // allow CostEstimatorApx access to private members
            class CostEstimatorApx
            {
            public:
                CostEstimatorApx(LazyLBTRRT *alg) : alg_(alg)
                {
                }
                double operator()(std::size_t i)
                {
                    double lb_estimate = (*(alg_->LPAstarLb_))(i);
                    if (lb_estimate != std::numeric_limits<double>::infinity())
                        return lb_estimate;

                    return alg_->distanceFunction(alg_->idToMotionMap_[i], alg_->startMotion_);
                }

            private:
                LazyLBTRRT *alg_;
            };  // CostEstimatorApx

            class CostEstimatorLb
            {
            public:
                CostEstimatorLb(base::Goal *goal, std::vector<Motion *> &idToMotionMap)
                  : goal_(goal), idToMotionMap_(idToMotionMap)
                {
                }
                double operator()(std::size_t i)
                {
                    double dist = 0.0;
                    goal_->isSatisfied(idToMotionMap_[i]->state_, &dist);

                    return dist;
                }

            private:
                base::Goal *goal_;
                std::vector<Motion *> &idToMotionMap_;
            };  // CostEstimatorLb

            using LPAstarApx = LPAstarOnGraph<BoostGraph, CostEstimatorApx>;
            using LPAstarLb = LPAstarOnGraph<BoostGraph, CostEstimatorLb>;

            /** \brief sample with goal biasing*/
            void sampleBiased(const base::GoalSampleableRegion *goal_s, base::State *rstate);

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const base::State *a, const base::State *b) const
            {
                return si_->distance(a, b);
            }
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state_, b->state_);
            }
            bool checkMotion(const base::State *a, const base::State *b) const
            {
                return si_->checkMotion(a, b);
            }
            bool checkMotion(const Motion *a, const Motion *b) const
            {
                return si_->checkMotion(a->state_, b->state_);
            }

            Motion *getMotion(std::size_t id) const
            {
                assert(idToMotionMap_.size() > id);
                return idToMotionMap_[id];
            }
            void addVertex(const Motion *a)
            {
                boost::add_vertex(a->id_, graphApx_);
                boost::add_vertex(a->id_, graphLb_);
            }

            void addEdgeApx(Motion *a, Motion *b, double c)
            {
                WeightProperty w(c);
                boost::add_edge(a->id_, b->id_, w, graphApx_);
                LPAstarApx_->insertEdge(a->id_, b->id_, c);
                LPAstarApx_->insertEdge(b->id_, a->id_, c);
            }
            void addEdgeLb(const Motion *a, const Motion *b, double c)
            {
                WeightProperty w(c);
                boost::add_edge(a->id_, b->id_, w, graphLb_);
                LPAstarLb_->insertEdge(a->id_, b->id_, c);
                LPAstarLb_->insertEdge(b->id_, a->id_, c);
            }
            bool edgeExistsApx(std::size_t a, std::size_t b)
            {
                return boost::edge(a, b, graphApx_).second;
            }
            bool edgeExistsApx(const Motion *a, const Motion *b)
            {
                return edgeExistsApx(a->id_, b->id_);
            }
            bool edgeExistsLb(const Motion *a, const Motion *b)
            {
                return boost::edge(a->id_, b->id_, graphLb_).second;
            }
            void removeEdgeLb(const Motion *a, const Motion *b)
            {
                boost::remove_edge(a->id_, b->id_, graphLb_);
                LPAstarLb_->removeEdge(a->id_, b->id_);
                LPAstarLb_->removeEdge(b->id_, a->id_);
            }
            std::tuple<Motion *, base::State *, double> rrtExtend(const base::GoalSampleableRegion *goal_s,
                                                                  base::State *xstate, Motion *rmotion,
                                                                  double &approxdif);
            void rrt(const base::PlannerTerminationCondition &ptc, base::GoalSampleableRegion *goal_s,
                     base::State *xstate, Motion *rmotion, double &approxdif);
            Motion *createMotion(const base::GoalSampleableRegion *goal_s, const base::State *st);
            Motion *createGoalMotion(const base::GoalSampleableRegion *goal_s);

            void closeBounds(const base::PlannerTerminationCondition &ptc);

            /** \brief Get the apprimation factor */
            double getApproximationFactor() const
            {
                return epsilon_;
            }

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{0.05};

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{0.};

            /** \brief The random number generator */
            RNG rng_;

            /** \brief approximation factor*/
            double epsilon_{.4};

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};

            BoostGraph graphLb_;
            BoostGraph graphApx_;
            Motion *startMotion_;
            Motion *goalMotion_{nullptr};  // root of LPAstarApx_
            LPAstarApx *LPAstarApx_{nullptr};  // rooted at target
            LPAstarLb *LPAstarLb_{nullptr};  // rooted at source
            std::vector<Motion *> idToMotionMap_;

            //////////////////////////////
            // Planner progress properties
            /** \brief Number of iterations the algorithm performed */
            unsigned int iterations_{0};
            /** \brief Best cost found so far by algorithm */
            double bestCost_;
        };
    }
}

#endif  // OMPL_CONTRIB_LAZY_LBTRRT_
