/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2019, University of Malaya
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

/* Author: Reza Mashayekhi */

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_RRTSTAR_CONNECT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_RRTSTAR_CONNECT_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/datastructures/NearestNeighbors.h"

#include <limits>
#include <vector>
#include <queue>
#include <deque>
#include <utility>
#include <list>

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor gRRTC
           @par Short description
           The basic idea is to grow two RRTs, one from the start and
           one from the goal, and attempt to connect them.
           @par External documentation
           S. Klemm et al., "RRT∗-Connect: Faster, asymptotically optimal motion planning," 
           2015 IEEE International Conference on Robotics and Biomimetics (ROBIO), Zhuhai, 2015, pp. 1670-1677
           doi: 10.1109/ROBIO.2015.7419012
        */

        /** \brief RRT*-Connect (RRTstarConnect) */
        class RRTstarConnect : public base::Planner
        {
        public:
            /** \brief Constructor */
            RRTstarConnect(const base::SpaceInformationPtr &si);

            ~RRTstarConnect() override;

            void getPlannerData(base::PlannerData &data) const override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            void clear() override;

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

            /** \brief Option that delays collision checking procedures.
                When it is enabled, all neighbors are sorted by cost. The
                planner then goes through this list, starting with the lowest
                cost, checking for collisions in order to find a parent. The planner
                stops iterating through the list when a collision free parent is found.
                This prevents the planner from collision checking each neighbor, reducing
                computation time in scenarios where collision checking procedures are expensive.*/
            void setDelayCC(bool delayCC)
            {
                delayCC_ = delayCC;
            }

            /** \brief Get the state of the delayed collision checking option */
            bool getDelayCC() const
            {
                return delayCC_;
            }

            /** \brief Controls whether the tree is pruned during the search. This pruning removes
                a vertex if and only if it \e and all its descendents passes the pruning condition.
                The pruning condition is whether the lower-bounding estimate of a solution
                constrained to pass the the \e vertex is greater than the current solution.
                Considering the descendents of a vertex prevents removing a descendent
                that may actually be capable of later providing a better solution once
                its incoming path passes through a different vertex (e.g., a change in homotopy class). */
            void setTreePruning(bool prune);

            /** \brief Get the state of the pruning option. */
            bool getTreePruning() const
            {
                return useTreePruning_;
            }

            /** \brief Set the fractional change in solution cost necessary for pruning to occur, i.e.,
                prune if the new solution is at least X% better than the old solution.
                (e.g., 0.0 will prune after every new solution, while 1.0 will never prune.) */
            void setPruneThreshold(const double pp)
            {
                pruneThreshold_ = pp;
            }

            /** \brief Get the current prune states percentage threshold parameter. */
            double getPruneThreshold() const
            {
                return pruneThreshold_;
            }

            /** \brief Use the measure of the pruned subproblem instead of the measure of the entire problem domain (if
            such an expression exists and a solution is present).
            Currently the only method to calculate this measure in closed-form is through a informed sampler, so this
            option also requires that. */
            void setPrunedMeasure(bool informedMeasure);

            /** \brief Get the state of using the pruned measure */
            bool getPrunedMeasure() const
            {
                return usePrunedMeasure_;
            }

            /** \brief Use direct sampling of the heuristic for the generation of random samples (e.g., x_rand).
           If a direct sampling method is not defined for the objective, rejection sampling will be used by default. */
            void setInformedSampling(bool informedSampling);

            /** \brief Get the state direct heuristic sampling */
            bool getInformedSampling() const
            {
                return useInformedSampling_;
            }

            /** \brief Controls whether pruning and new-state rejection uses an admissible cost-to-come estimate or not
             */
            void setAdmissibleCostToCome(const bool admissible)
            {
                useAdmissibleCostToCome_ = admissible;
            }

            /** \brief Get the admissibility of the pruning and new-state rejection heuristic */
            bool getAdmissibleCostToCome() const
            {
                return useAdmissibleCostToCome_;
            }

            /** \brief Use a k-nearest search for rewiring instead of a r-disc search. */
            void setKNearest(bool useKNearest)
            {
                useKNearest_ = useKNearest;
            }

            /** \brief Get the state of using a k-nearest search for rewiring. */
            bool getKNearest() const
            {
                return useKNearest_;
            }

            /** \brief Set the number of attempts to make while performing rejection or informed sampling */
            void setNumSamplingAttempts(unsigned int numAttempts)
            {
                numSampleAttempts_ = numAttempts;
            }

            /** \brief Get the number of attempts to make while performing rejection or informed sampling */
            unsigned int getNumSamplingAttempts() const
            {
                return numSampleAttempts_;
            }

            /** \brief Set the rewiring scale factor, s, such that r_rrg = s \times r_rrg* (or k_rrg = s \times k_rrg*)
             */
            void setRewireFactor(double rewireFactor)
            {
                rewireFactor_ = rewireFactor;
                calculateRewiringLowerBounds();
            }

            /** \brief Set the rewiring scale factor, s, such that r_rrg = s \times r_rrg* > r_rrg* (or k_rrg = s \times
             * k_rrg* > k_rrg*) */
            double getRewireFactor() const
            {
                return rewireFactor_;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if ((tStart_ && tStart_->size() != 0) || (tGoal_ && tGoal_->size() != 0))
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                tStart_ = std::make_shared<NN<Motion *>>();
                tGoal_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            void setup() override;

            unsigned int numIterations() const
            {
                return iterations_;
            }

            ompl::base::Cost bestCost() const
            {
                return bestCost_;
            }

        protected:
            /** \brief Representation of a motion */
            class Motion
            {
            public:
                Motion() = default;

                Motion(const base::SpaceInformationPtr &si) : state(si->allocState())
                {
                }

                ~Motion() = default;

				/** \brief The state contained by the motion */
				base::State *state{nullptr};

				/** \brief The parent motion in the exploration tree */
				Motion *parent{nullptr};

				/** \brief The root of the tree that the state belongs to*/
				const base::State *root{nullptr};

				/** \brief Set to true if this vertex is in the goal region */
				bool isConnectionPoint{false};

				/** \brief The cost up to this motion */
				base::Cost cost;

				/** \brief The incremental cost of this motion's parent to this motion (this is stored to save distance
				* computations in the updateChildCosts() method) */
				base::Cost incCost;

				/** \brief The set of motions descending from the current motion */
				std::vector<Motion *> children;
            };

            /** \brief A nearest-neighbor datastructure representing a tree of motions */
            typedef std::shared_ptr<NearestNeighbors<Motion *>> TreeData;

            /** \brief Gets the neighbours of a given motion, using either k-nearest of radius as appropriate. */
            void getNeighbors(TreeData &tree, Motion *motion, std::vector<Motion *> &nbh) const;

            /** \brief Removes the given motion from the parent's child list */
            void removeFromParent(Motion *m);

            /** \brief Updates the cost of the children of this node if the cost up to this node has changed */
            void updateChildCosts(Motion *m);

            /** \brief Add the children of a vertex to the given list. */
            void addChildrenToList(std::queue<Motion *, std::deque<Motion *>> *motionList, Motion *motion);

            /** \brief Check whether the given motion passes the specified cost threshold, meaning it will be \e kept
             * during pruning */
            bool keepCondition(const Motion *motion, const base::Cost &threshold, bool isStartTree) const;

            /** \brief Prunes all those states which estimated total cost is higher than pruneTreeCost.
                Returns the number of motions pruned. Depends on the parameter set by
               setPruneStatesImprovementThreshold() */
            int pruneTrees(const base::Cost &pruneTreeCost);

            /** \brief Prunes all those states which estimated total cost is higher than pruneTreeCost.
                Returns the number of motions pruned. Depends on the parameter set by
               setPruneStatesImprovementThreshold() */
            int pruneTree(TreeData &tree, const base::Cost &pruneTreeCost, bool isTreeStart);

            /** \brief Computes the solution cost heuristically as the cost to come from start to the motion plus
                 the cost to go from the motion to the goal. If the parameter \e use_admissible_heuristic
                 (\e setAdmissibleCostToCome()) is true, a heuristic estimate of the cost to come is used;
                 otherwise, the current cost to come to the motion is used (which may overestimate the cost
                 through the motion). */
            base::Cost solutionHeuristic(const Motion *motion, bool isTreeStart) const;

            /** \brief Information attached to growing a tree of motions (used internally) */
            struct TreeGrowingInfo
            {
                base::State *xstate;
                Motion *xmotion;
                bool start;
            };

            /** \brief The state of the tree after an attempt to extend it */
            enum GrowState
            {
                /// no progress has been made
                TRAPPED,
                /// progress has been made towards the randomly sampled state
                ADVANCED,
                /// the randomly sampled state was reached
                REACHED
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            // For sorting a list of costs and getting only their sorted indices
            struct CostIndexCompare
            {
                CostIndexCompare(const std::vector<base::Cost> &costs, const base::OptimizationObjective &opt)
                  : costs_(costs), opt_(opt)
                {
                }
                bool operator()(unsigned i, unsigned j)
                {
                    return opt_.isCostBetterThan(costs_[i], costs_[j]);
                }
                const std::vector<base::Cost> &costs_;
                const base::OptimizationObjective &opt_;
            };

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief number of attempt to rewire trees */
            unsigned int rewireTest{0};

            /** \brief number of generated states*/
            unsigned int statesGenerated{0};

            /** \brief vectors used in solve function */
            std::vector<base::Cost> costs;
            std::vector<base::Cost> incCosts;
            std::vector<std::size_t> sortedCostIndices;
            std::vector<int> valid;
            bool checkForSolution{false};

            /** \brief Grow a tree towards a random state */
            GrowState growTree(TreeData &tree, TreeGrowingInfo &tgi, Motion *rmotion);

            /** \brief Calculate the k_RRG* and r_RRG* terms */
            void calculateRewiringLowerBounds();

            /** \brief Generate a sample */
            bool sampleUniform(base::State *statePtr);

            /** \brief Create the samplers */
            void allocSampler();

            /** \brief State sampler */
             base::StateSamplerPtr sampler_;

             /** \brief An informed sampler */
             base::InformedSamplerPtr infSampler_;

            /** \brief The start tree */
            TreeData tStart_;

            /** \brief The goal tree */
            TreeData tGoal_;

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{0.};

            /** \brief Flag indicating whether intermediate states are added to the built tree of motions */
            bool addIntermediateStates_;

            /** \brief The random number generator */
            RNG rng_;

            /** \brief Option to use k-nearest search for rewiring */
            bool useKNearest_{true};

            /** \brief The rewiring factor, s, so that r_rrt = s \times r_rrt* > r_rrt* (or k_rrt = s \times k_rrt* >
             * k_rrt*) */
            double rewireFactor_{1.1};

            /** \brief A constant for k-nearest rewiring calculations */
            double k_rrt_{0u};

            /** \brief A constant for r-disc rewiring calculations */
            double r_rrt_{0.};

            /** \brief Option to delay and reduce collision checking within iterations */
            bool delayCC_{true};

            /** \brief The measure of the problem when we pruned it (if this isn't in use, it will be set to
             * si_->getSpaceMeasure())*/
            double prunedMeasure_{0.};

            /** \brief The tree is pruned when the change in solution cost is greater than this fraction. */
            double pruneThreshold_{.05};

            /** \brief Option to use the informed measure */
            bool usePrunedMeasure_{false};

            /** \brief Option to use informed sampling */
            bool useInformedSampling_{false};

            /** \brief The admissibility of the new-state rejection heuristic. */
            bool useAdmissibleCostToCome_{true};

            /** \brief The number of attempts to make at informed sampling */
            unsigned int numSampleAttempts_{100u};

            /** \brief Stores the start states as Motions. */
            std::vector<Motion *> startMotions_;

            /** \brief A list of states in the tree that satisfy the goal condition */
            std::vector<Motion *> goalMotions_;

            /** \brief Best cost found so far by algorithm */
            base::Cost bestCost_{std::numeric_limits<double>::quiet_NaN()};

            /** \brief The cost at which the graph was last pruned */
            base::Cost prunedCost_{std::numeric_limits<double>::quiet_NaN()};

            /** \brief Objective we're optimizing */
            base::OptimizationObjectivePtr opt_;

            /** \brief The pair of states in each tree connected during planning.  Used for PlannerData computation */
            std::pair<Motion *, Motion *> bestConnectionPoint_;

            /** \brief A list of The pair of states in each tree connected during planning.  Used for PlannerData computation */
            std::vector<std::pair<Motion *, Motion *>> connectionPoints_;

            /** \brief The status of the tree pruning option. */
            bool useTreePruning_{false};

            /** \brief Distance between the nearest pair of start tree and goal tree nodes. */
            double distanceBetweenTrees_;

            /** \brief Number of iterations the algorithm performed */
            unsigned int iterations_{0u};

            ///////////////////////////////////////
            // Planner progress property functions
            std::string numIterationsProperty() const
            {
                return std::to_string(numIterations());
            }

            std::string bestCostProperty() const
            {
                return std::to_string(bestCost().value());
            }
        };
    }
}

#endif
