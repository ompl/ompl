/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, University of Toronto
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
 *   * Neither the name of the University of Toronto nor the names of its
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

/* Authors: Jonathan Gammell, Marlin Strub */

#include "ompl/geometric/planners/informedtrees/BITstar.h"

#include <sstream>
#include <iomanip>
#include <memory>
#include <boost/range/adaptor/reversed.hpp>

#include "ompl/util/Console.h"
#include "ompl/util/Exception.h"
#include "ompl/util/String.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"

#include "ompl/geometric/planners/informedtrees/bitstar/HelperFunctions.h"
#include "ompl/geometric/planners/informedtrees/bitstar/IdGenerator.h"
#include "ompl/geometric/planners/informedtrees/bitstar/Vertex.h"
#include "ompl/geometric/planners/informedtrees/bitstar/CostHelper.h"
#include "ompl/geometric/planners/informedtrees/bitstar/ImplicitGraph.h"
#include "ompl/geometric/planners/informedtrees/bitstar/SearchQueue.h"

#ifdef BITSTAR_DEBUG
#warning Compiling BIT* with debug-level asserts.
#endif  // BITSTAR_DEBUG

namespace ompl
{
    namespace geometric
    {
        BITstar::BITstar(const ompl::base::SpaceInformationPtr &si, const std::string &name /*= "BITstar"*/)
          : ompl::base::Planner(si, name)
        {
#ifdef BITSTAR_DEBUG
            OMPL_WARN("%s: Compiled with debug-level asserts.", Planner::getName().c_str());
#endif  // BITSTAR_DEBUG

            // Allocate my helper classes, they hold settings and must never be deallocated. Give them a pointer to my
            // name, so they can output helpful error messages
            costHelpPtr_ = std::make_shared<CostHelper>();
            graphPtr_ = std::make_shared<ImplicitGraph>([this]() { return getName(); });
            queuePtr_ = std::make_shared<SearchQueue>([this]() { return getName(); });

            // Specify my planner specs:
            Planner::specs_.recognizedGoal = ompl::base::GOAL_SAMPLEABLE_REGION;
            Planner::specs_.multithreaded = false;
            // Approximate solutions are supported but must be enabled with the appropriate configuration parameter.
            Planner::specs_.approximateSolutions = graphPtr_->getTrackApproximateSolutions();
            Planner::specs_.optimizingPaths = true;
            Planner::specs_.directed = true;
            Planner::specs_.provingSolutionNonExistence = false;
            Planner::specs_.canReportIntermediateSolutions = true;

            // Register my setting callbacks
            Planner::declareParam<double>("rewire_factor", this, &BITstar::setRewireFactor, &BITstar::getRewireFactor,
                                          "1.0:0.01:3.0");
            Planner::declareParam<unsigned int>("samples_per_batch", this, &BITstar::setSamplesPerBatch,
                                                &BITstar::getSamplesPerBatch, "1:1:1000000");
            Planner::declareParam<bool>("use_k_nearest", this, &BITstar::setUseKNearest, &BITstar::getUseKNearest,
                                        "0,"
                                        "1");
            Planner::declareParam<bool>("use_graph_pruning", this, &BITstar::setPruning, &BITstar::getPruning,
                                        "0,"
                                        "1");
            Planner::declareParam<double>("prune_threshold_as_fractional_cost_change", this,
                                          &BITstar::setPruneThresholdFraction, &BITstar::getPruneThresholdFraction,
                                          "0.0:0.01:1.0");
            Planner::declareParam<bool>("delay_rewiring_to_first_solution", this,
                                        &BITstar::setDelayRewiringUntilInitialSolution,
                                        &BITstar::getDelayRewiringUntilInitialSolution, "0,1");
            Planner::declareParam<bool>("use_just_in_time_sampling", this, &BITstar::setJustInTimeSampling,
                                        &BITstar::getJustInTimeSampling, "0,1");
            Planner::declareParam<bool>("drop_unconnected_samples_on_prune", this, &BITstar::setDropSamplesOnPrune,
                                        &BITstar::getDropSamplesOnPrune, "0,1");
            Planner::declareParam<bool>("stop_on_each_solution_improvement", this, &BITstar::setStopOnSolnImprovement,
                                        &BITstar::getStopOnSolnImprovement, "0,1");
            Planner::declareParam<bool>("use_strict_queue_ordering", this, &BITstar::setStrictQueueOrdering,
                                        &BITstar::getStrictQueueOrdering, "0,1");
            Planner::declareParam<bool>("find_approximate_solutions", this, &BITstar::setConsiderApproximateSolutions,
                                        &BITstar::getConsiderApproximateSolutions, "0,1");

            // Register my progress info:
            addPlannerProgressProperty("best cost DOUBLE", [this] { return bestCostProgressProperty(); });
            addPlannerProgressProperty("number of segments in solution path INTEGER",
                                       [this] { return bestLengthProgressProperty(); });
            addPlannerProgressProperty("state collision checks INTEGER",
                                       [this] { return stateCollisionCheckProgressProperty(); });
            addPlannerProgressProperty("edge collision checks INTEGER",
                                       [this] { return edgeCollisionCheckProgressProperty(); });
            addPlannerProgressProperty("nearest neighbour calls INTEGER",
                                       [this] { return nearestNeighbourProgressProperty(); });

            // Extra progress info that aren't necessary for every day use. Uncomment if desired.
            /*
            addPlannerProgressProperty("edge queue size INTEGER", [this]
                                       {
                                           return edgeQueueSizeProgressProperty();
                                       });
            addPlannerProgressProperty("iterations INTEGER", [this]
                                       {
                                           return iterationProgressProperty();
                                       });
            addPlannerProgressProperty("batches INTEGER", [this]
                                       {
                                           return batchesProgressProperty();
                                       });
            addPlannerProgressProperty("graph prunings INTEGER", [this]
                                       {
                                           return pruningProgressProperty();
                                       });
            addPlannerProgressProperty("total states generated INTEGER", [this]
                                       {
                                           return totalStatesCreatedProgressProperty();
                                       });
            addPlannerProgressProperty("vertices constructed INTEGER", [this]
                                       {
                                           return verticesConstructedProgressProperty();
                                       });
            addPlannerProgressProperty("states pruned INTEGER", [this]
                                       {
                                           return statesPrunedProgressProperty();
                                       });
            addPlannerProgressProperty("graph vertices disconnected INTEGER", [this]
                                       {
                                           return verticesDisconnectedProgressProperty();
                                       });
            addPlannerProgressProperty("rewiring edges INTEGER", [this]
                                       {
                                           return rewiringProgressProperty();
                                       });
            */
        }

        void BITstar::setup()
        {
            // Call the base class setup. Marks Planner::setup_ as true.
            Planner::setup();

            // Check if we have a problem definition
            if (static_cast<bool>(Planner::pdef_))
            {
                // We do, do some initialization work.
                // See if we have an optimization objective
                if (!Planner::pdef_->hasOptimizationObjective())
                {
                    OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length.",
                                Planner::getName().c_str());
                    Planner::pdef_->setOptimizationObjective(
                        std::make_shared<base::PathLengthOptimizationObjective>(Planner::si_));
                }
                // No else, we were given one.

                // Initialize the best cost found so far to be infinite.
                bestCost_ = Planner::pdef_->getOptimizationObjective()->infiniteCost();

                // If the problem definition *has* a goal, make sure it is of appropriate type
                if (static_cast<bool>(Planner::pdef_->getGoal()))
                {
                    if (!Planner::pdef_->getGoal()->hasType(ompl::base::GOAL_SAMPLEABLE_REGION))
                    {
                        OMPL_ERROR("%s::setup() BIT* currently only supports goals that can be cast to a sampleable "
                                   "goal region.",
                                   Planner::getName().c_str());
                        // Mark as not setup:
                        Planner::setup_ = false;
                        return;
                    }
                    // No else, of correct type.
                }
                // No else, called without a goal. Is this MoveIt?

                // Setup the CostHelper, it provides everything I need from optimization objective plus some frills
                costHelpPtr_->setup(Planner::pdef_->getOptimizationObjective(), graphPtr_.get());

                // Setup the queue
                queuePtr_->setup(costHelpPtr_.get(), graphPtr_.get());

                // Setup the graph, it does not hold a copy of this or Planner::pis_, but uses them to create a NN
                // struct and check for starts/goals, respectively.
                graphPtr_->setup(Planner::si_, Planner::pdef_, costHelpPtr_.get(), queuePtr_.get(), this,
                                 Planner::pis_);
                graphPtr_->setPruning(isPruningEnabled_);

                // Set the best and pruned costs to the proper objective-based values:
                bestCost_ = costHelpPtr_->infiniteCost();
                prunedCost_ = costHelpPtr_->infiniteCost();

                // Get the measure of the problem
                prunedMeasure_ = Planner::si_->getSpaceMeasure();

                // If the planner is default named, we change it:
                if (!graphPtr_->getUseKNearest() && Planner::getName() == "kBITstar")
                {
                    // It's current the default k-nearest BIT* name, and we're toggling, so set to the default r-disc
                    OMPL_WARN("BIT*: An r-disc version of BIT* can not be named 'kBITstar', as this name is reserved "
                              "for the k-nearest version. Changing the name to 'BITstar'.");
                    Planner::setName("BITstar");
                }
                else if (graphPtr_->getUseKNearest() && Planner::getName() == "BITstar")
                {
                    // It's current the default r-disc BIT* name, and we're toggling, so set to the default k-nearest
                    OMPL_WARN("BIT*: A k-nearest version of BIT* can not be named 'BITstar', as this name is reserved "
                              "for the r-disc version. Changing the name to 'kBITstar'.");
                    Planner::setName("kBITstar");
                }
                else if (!graphPtr_->getUseKNearest() && Planner::getName() == "kABITstar")
                {
                    // It's current the default k-nearest ABIT* name, and we're toggling, so set to the default r-disc
                    OMPL_WARN("ABIT*: An r-disc version of ABIT* can not be named 'kABITstar', as this name is "
                              "reserved for the k-nearest version. Changing the name to 'ABITstar'.");
                    Planner::setName("ABITstar");
                }
                else if (graphPtr_->getUseKNearest() && Planner::getName() == "ABITstar")
                {
                    // It's current the default r-disc ABIT* name, and we're toggling, so set to the default k-nearest
                    OMPL_WARN("ABIT*: A k-nearest version of ABIT* can not be named 'ABITstar', as this name is "
                              "reserved for the r-disc version. Changing the name to 'kABITstar'.");
                    Planner::setName("kABITstar");
                }
                // It's not default named, don't change it
            }
            else
            {
                // We don't, so we can't setup. Make sure that is explicit.
                Planner::setup_ = false;
            }
        }

        void BITstar::clear()
        {
            // Clear all the variables.
            // Keep this in the order of the constructors:

            // The various helper classes. DO NOT reset the pointers, they hold configuration parameters:
            costHelpPtr_->reset();
            graphPtr_->reset();
            queuePtr_->reset();

            // Reset the various calculations and convenience containers. Will be recalculated on setup
            curGoalVertex_.reset();
            bestCost_ = ompl::base::Cost(std::numeric_limits<double>::infinity());
            bestLength_ = 0u;
            prunedCost_ = ompl::base::Cost(std::numeric_limits<double>::infinity());
            prunedMeasure_ = 0.0;
            hasExactSolution_ = false;
            stopLoop_ = false;
            numBatches_ = 0u;
            numPrunings_ = 0u;
            numIterations_ = 0u;
            numEdgeCollisionChecks_ = 0u;
            numRewirings_ = 0u;

            // DO NOT reset the configuration parameters:
            // samplesPerBatch_
            // usePruning_
            // pruneFraction_
            // stopOnSolnChange_

            // Mark as not setup:
            Planner::setup_ = false;

            // Call my base clear:
            Planner::clear();
        }

        ompl::base::PlannerStatus BITstar::solve(const ompl::base::PlannerTerminationCondition &ptc)
        {
            // Check that Planner::setup_ is true, if not call this->setup()
            Planner::checkValidity();

            // Assert setup succeeded
            if (!Planner::setup_)
            {
                throw ompl::Exception("%s::solve() failed to set up the planner. Has a problem definition been set?",
                                      Planner::getName().c_str());
            }
            // No else

            OMPL_INFORM("%s: Searching for a solution to the given planning problem.", Planner::getName().c_str());

            // Reset the manual stop to the iteration loop:
            stopLoop_ = false;

            // If we don't have a goal yet, recall updateStartAndGoalStates, but wait for the first goal (or until the
            // PTC comes true and we give up):
            if (!graphPtr_->hasAGoal())
            {
                graphPtr_->updateStartAndGoalStates(Planner::pis_, ptc);
            }

            // Warn if we are missing a start
            if (!graphPtr_->hasAStart())
            {
                // We don't have a start, since there's no way to wait for one to appear, so we will not be solving this
                // "problem" today
                OMPL_WARN("%s: A solution cannot be found as no valid start states are available.",
                          Planner::getName().c_str());
            }
            // No else, it's a start

            // Warn if we are missing a goal
            if (!graphPtr_->hasAGoal())
            {
                // We don't have a goal (and we waited as long as ptc allowed us for one to appear), so we will not be
                // solving this "problem" today
                OMPL_WARN("%s: A solution cannot be found as no valid goal states are available.",
                          Planner::getName().c_str());
            }
            // No else, there's a goal to all of this

            // Insert the outgoing edges of the start vertices.
            if (numIterations_ == 0u)
            {
                queuePtr_->insertOutgoingEdgesOfStartVertices();
            }

            /* Iterate as long as:
              - We're allowed (ptc == false && stopLoop_ == false), AND
              - We haven't found a good enough solution (costHelpPtr_->isSatisfied(bestCost) == false),
              - AND
                - There is a theoretically better solution (costHelpPtr_->isCostBetterThan(graphPtr_->minCost(),
              bestCost_) == true), OR
                - There is are start/goal states we've yet to consider (pis_.haveMoreStartStates() == true ||
              pis_.haveMoreGoalStates() == true)
            */
            while (!ptc && !stopLoop_ && !costHelpPtr_->isSatisfied(bestCost_) &&
                   (costHelpPtr_->isCostBetterThan(graphPtr_->minCost(), bestCost_) ||
                    Planner::pis_.haveMoreStartStates() || Planner::pis_.haveMoreGoalStates()))
            {
                this->iterate();
            }

            // Announce
            if (hasExactSolution_)
            {
                this->endSuccessMessage();
            }
            else
            {
                this->endFailureMessage();
            }

            // Publish
            if (hasExactSolution_ || graphPtr_->getTrackApproximateSolutions())
            {
                // Any solution
                this->publishSolution();
            }
            // No else, no solution to publish

            // From OMPL's point-of-view, BIT* can always have an approximate solution, so mark solution true if either
            // we have an exact solution or are finding approximate ones.
            // Our returned solution will only be approximate if it is not exact and we are finding approximate
            // solutions.
            // PlannerStatus(addedSolution, approximate)
            return {hasExactSolution_ || graphPtr_->getTrackApproximateSolutions(),
                    !hasExactSolution_ && graphPtr_->getTrackApproximateSolutions()};
        }

        void BITstar::getPlannerData(ompl::base::PlannerData &data) const
        {
            // Get the base planner class data:
            Planner::getPlannerData(data);

            // Add the samples (the graph)
            graphPtr_->getGraphAsPlannerData(data);

            // Did we find a solution?
            if (hasExactSolution_)
            {
                // Exact solution
                data.markGoalState(curGoalVertex_->state());
            }
            else if (!hasExactSolution_ && graphPtr_->getTrackApproximateSolutions())
            {
                // Approximate solution
                data.markGoalState(graphPtr_->closestVertexToGoal()->state());
            }
            // No else, no solution
        }

        std::pair<ompl::base::State const *, ompl::base::State const *> BITstar::getNextEdgeInQueue()
        {
            // Variable:
            // The next edge as a basic pair of states
            std::pair<ompl::base::State const *, ompl::base::State const *> nextEdge;

            if (!queuePtr_->isEmpty())
            {
                // Variable
                // The edge in the front of the queue
                VertexConstPtrPair frontEdge = queuePtr_->getFrontEdge();

                // The next edge in the queue:
                nextEdge = std::make_pair(frontEdge.first->state(), frontEdge.second->state());
            }
            else
            {
                // An empty edge:
                nextEdge = std::make_pair<ompl::base::State *, ompl::base::State *>(nullptr, nullptr);
            }

            return nextEdge;
        }

        ompl::base::Cost BITstar::getNextEdgeValueInQueue()
        {
            // Variable
            // The cost of the next edge
            ompl::base::Cost nextCost;

            if (!queuePtr_->isEmpty())
            {
                // The next cost in the queue:
                nextCost = queuePtr_->getFrontEdgeValue().at(0u);
            }
            else
            {
                // An infinite cost:
                nextCost = costHelpPtr_->infiniteCost();
            }

            return nextCost;
        }

        void BITstar::getEdgeQueue(VertexConstPtrPairVector *edgesInQueue)
        {
            queuePtr_->getEdges(edgesInQueue);
        }

        unsigned int BITstar::numIterations() const
        {
            return numIterations_;
        }

        ompl::base::Cost BITstar::bestCost() const
        {
            return bestCost_;
        }

        unsigned int BITstar::numBatches() const
        {
            return numBatches_;
        }
        /////////////////////////////////////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////////////////////////////////////
        // Protected functions:
        void BITstar::iterate()
        {
            // Keep track of how many iterations we've performed.
            ++numIterations_;

            // If the search is done or the queue is empty, we need to populate the queue.
            if (isSearchDone_ || queuePtr_->isEmpty())
            {
                // Check whether we've exhausted the current approximation.
                if (isFinalSearchOnBatch_ || !hasExactSolution_)
                {
                    // Prune the graph if enabled.
                    if (isPruningEnabled_)
                    {
                        this->prune();
                    }

                    // Add a new batch.
                    this->newBatch();

                    // Set the inflation factor to an initial value.
                    queuePtr_->setInflationFactor(initialInflationFactor_);

                    // Clear the search queue.
                    queuePtr_->clear();

                    // Restart the queue, adding the outgoing edges of the start vertices to the queue.
                    queuePtr_->insertOutgoingEdgesOfStartVertices();

                    // Set flag to false.
                    isFinalSearchOnBatch_ = false;

                    // Set the new truncation factor.
                    truncationFactor_ =
                        1.0 + truncationScalingParameter_ /
                                  (static_cast<float>(graphPtr_->numVertices() + graphPtr_->numSamples()));
                }
                else
                {
                    // Exhaust the current approximation by performing an uninflated search.
                    queuePtr_->setInflationFactor(
                        1.0 + inflationScalingParameter_ /
                                  (static_cast<float>(graphPtr_->numVertices() + graphPtr_->numSamples())));
                    queuePtr_->rebuildEdgeQueue();
                    queuePtr_->insertOutgoingEdgesOfInconsistentVertices();
                    queuePtr_->clearInconsistentSet();
                    isFinalSearchOnBatch_ = true;
                }

                isSearchDone_ = false;
            }
            else
            {
                // Get the most promising edge.
                VertexPtrPair edge = queuePtr_->popFrontEdge();

                // If this edge is already part of the search tree it's a freebie.
                if (edge.second->hasParent() && edge.second->getParent()->getId() == edge.first->getId())
                {
                    if (!edge.first->isExpandedOnCurrentSearch())
                    {
                        edge.first->registerExpansion();
                    }
                    queuePtr_->insertOutgoingEdges(edge.second);
                }
                // In the best case, can this edge improve our solution given the current graph?
                // g_t(v) + c_hat(v,x) + h_hat(x) < g_t(x_g)?
                else if (costHelpPtr_->isCostBetterThan(
                             costHelpPtr_->inflateCost(costHelpPtr_->currentHeuristicEdge(edge), truncationFactor_),
                             bestCost_))
                {
                    // What about improving the current graph?
                    // g_t(v) + c_hat(v,x)  < g_t(x)?
                    if (costHelpPtr_->isCostBetterThan(costHelpPtr_->currentHeuristicToTarget(edge),
                                                       edge.second->getCost()))
                    {
                        // Ok, so it *could* be a useful edge. Do the work of calculating its cost for real

                        // Get the true cost of the edge
                        ompl::base::Cost trueEdgeCost = costHelpPtr_->trueEdgeCost(edge);

                        // Can this actual edge ever improve our solution?
                        // g_hat(v) + c(v,x) + h_hat(x) < g_t(x_g)?
                        if (costHelpPtr_->isCostBetterThan(
                                costHelpPtr_->combineCosts(costHelpPtr_->costToComeHeuristic(edge.first), trueEdgeCost,
                                                           costHelpPtr_->costToGoHeuristic(edge.second)),
                                bestCost_))
                        {
                            // Does this edge have a collision?
                            if (this->checkEdge(edge))
                            {
                                // Remember that this edge has passed the collision checks.
                                this->whitelistEdge(edge);

                                // Does the current edge improve our graph?
                                // g_t(v) + c(v,x) < g_t(x)?
                                if (costHelpPtr_->isCostBetterThan(
                                        costHelpPtr_->combineCosts(edge.first->getCost(), trueEdgeCost),
                                        edge.second->getCost()))
                                {
                                    // YAAAAH. Add the edge! Allowing for the sample to be removed from free if it is
                                    // not currently connected and otherwise propagate cost updates to descendants.
                                    // addEdge will update the queue and handle the extra work that occurs if this edge
                                    // improves the solution.
                                    this->addEdge(edge, trueEdgeCost);

                                    // If the path to the goal has changed, we will need to update the cached info about
                                    // the solution cost or solution length:
                                    this->updateGoalVertex();

                                    // If this is the first edge that's being expanded in the current search, remember
                                    // the cost-to-come and the search / approximation ids.
                                    if (!edge.first->isExpandedOnCurrentSearch())
                                    {
                                        edge.first->registerExpansion();
                                    }
                                }
                                // No else, this edge may be useful at some later date.
                            }
                            else  // Remember that this edge is in collision.
                            {
                                this->blacklistEdge(edge);
                            }
                        }
                        // No else, we failed
                    }
                    // No else, we failed
                }
                else
                {
                    isSearchDone_ = true;
                }
            }  // Search queue not empty.
        }

        void BITstar::newBatch()
        {
            // Increment the batch counter.
            ++numBatches_;

            // Do we need to update our starts or goals?
            if (Planner::pis_.haveMoreStartStates() || Planner::pis_.haveMoreGoalStates())
            {
                // There are new starts/goals to get.
                graphPtr_->updateStartAndGoalStates(Planner::pis_, ompl::base::plannerAlwaysTerminatingCondition());
            }
            // No else, we have enough of a problem to do some work, and everything's up to date.

            // Add a new batch of samples.
            graphPtr_->addNewSamples(samplesPerBatch_);
        }

        void BITstar::prune()
        {
#ifdef BITSTAR_DEBUG
            if (!isPruningEnabled_)
            {
                throw ompl::Exception("Pruning is not enabled, but prune() is called nonetheless.");
            }
#endif  // BITSTAR_DEBUG

            // If we don't have an exact solution, we can't prune sensibly.
            if (hasExactSolution_)
            {
                /* Profiling reveals that pruning is very expensive, mainly because the nearest neighbour structure of
                 * the samples has to be updated. On the other hand, nearest neighbour lookup gets more expensive the
                 * bigger the structure, so it's a tradeoff. Pruning on every cost update seems insensible, but so does
                 * never pruning at all. The criteria to prune should depend on how many vertices/samples there are and
                 * how many of them could be pruned, as the decrease in cost associated with nearest neighbour lookup
                 * for fewer samples must justify the cost of pruning. It turns out that counting is affordable, so we
                 * don't need to use any proxy here. */

                // Count the number of samples that could be pruned.
                auto samples = graphPtr_->getCopyOfSamples();
                auto numSamplesThatCouldBePruned =
                    std::count_if(samples.begin(), samples.end(), [this](const auto& sample){
                        return graphPtr_->canSampleBePruned(sample);
                    });

                // Only prune if the decrease in number of samples and the associated decrease in nearest neighbour
                // lookup cost justifies the cost of pruning. There has to be a way to make this more formal, and less
                // knob-turney, right?
                if (static_cast<float>(numSamplesThatCouldBePruned) /
                        static_cast<float>(graphPtr_->numSamples() + graphPtr_->numVertices()) >=
                    pruneFraction_)
                {
                    // Get the current informed measure of the problem space.
                    double informedMeasure = graphPtr_->getInformedMeasure(bestCost_);

                    // Increment the pruning counter:
                    ++numPrunings_;

                    // Prune the graph.
                    std::pair<unsigned int, unsigned int> numPruned = graphPtr_->prune(informedMeasure);

                    // Store the cost at which we pruned:
                    prunedCost_ = bestCost_;

                    // Also store the measure.
                    prunedMeasure_ = informedMeasure;

                    OMPL_INFORM("%s: Pruning disconnected %d vertices from the tree and completely removed %d samples.",
                                Planner::getName().c_str(), numPruned.first, numPruned.second);
                }
            }
            // No else.
        }

        void BITstar::blacklistEdge(const VertexPtrPair &edge) const
        {
            // We store the actual blacklist with the parent vertex for efficient lookup.
            edge.first->blacklistChild(edge.second);
        }

        void BITstar::whitelistEdge(const VertexPtrPair &edge) const
        {
            // We store the actual whitelist with the parent vertex for efficient lookup.
            edge.first->whitelistChild(edge.second);
        }

        void BITstar::publishSolution()
        {
            // Variable
            // The reverse path of state pointers
            std::vector<const ompl::base::State *> reversePath;
            // Allocate a path geometric
            auto pathGeoPtr = std::make_shared<ompl::geometric::PathGeometric>(Planner::si_);

            // Get the reversed path
            reversePath = this->bestPathFromGoalToStart();

            // Now iterate that vector in reverse, putting the states into the path geometric
            for (const auto &solnState : boost::adaptors::reverse(reversePath))
            {
                pathGeoPtr->append(solnState);
            }

            // Now create the solution
            ompl::base::PlannerSolution soln(pathGeoPtr);

            // Mark the name:
            soln.setPlannerName(Planner::getName());

            // Mark as approximate if not exact:
            if (!hasExactSolution_ && graphPtr_->getTrackApproximateSolutions())
            {
                soln.setApproximate(graphPtr_->smallestDistanceToGoal());
            }

            // Mark whether the solution met the optimization objective:
            soln.setOptimized(Planner::pdef_->getOptimizationObjective(), bestCost_,
                              Planner::pdef_->getOptimizationObjective()->isSatisfied(bestCost_));

            // Add the solution to the Problem Definition:
            Planner::pdef_->addSolutionPath(soln);
        }

        std::vector<const ompl::base::State *> BITstar::bestPathFromGoalToStart() const
        {
            // Variables:
            // A vector of states from goal->start:
            std::vector<const ompl::base::State *> reversePath;
            // The vertex used to ascend up from the goal:
            VertexConstPtr curVertex;

            // Iterate up the chain from the goal, creating a backwards vector:
            if (hasExactSolution_)
            {
                // Start at vertex in the goal
                curVertex = curGoalVertex_;
            }
            else if (!hasExactSolution_ && graphPtr_->getTrackApproximateSolutions())
            {
                // Start at the vertex closest to the goal
                curVertex = graphPtr_->closestVertexToGoal();
            }
            else
            {
                throw ompl::Exception("bestPathFromGoalToStart called without an exact or approximate solution.");
            }

            // Insert the goal into the path
            reversePath.push_back(curVertex->state());

            // Then, use the vertex pointer like an iterator. Starting at the goal, we iterate up the chain pushing the
            // *parent* of the iterator into the vector until the vertex has no parent.
            // This will allows us to add the start (as the parent of the first child) and then stop when we get to the
            // start itself, avoiding trying to find its nonexistent child
            for (/*Already allocated & initialized*/; !curVertex->isRoot(); curVertex = curVertex->getParent())
            {
#ifdef BITSTAR_DEBUG
                // Check the case where the chain ends incorrectly.
                if (curVertex->hasParent() == false)
                {
                    throw ompl::Exception("The path to the goal does not originate at a start state. Something went "
                                          "wrong.");
                }
#endif  // BITSTAR_DEBUG

                // Push back the parent into the vector as a state pointer:
                reversePath.push_back(curVertex->getParent()->state());
            }
            return reversePath;
        }

        bool BITstar::checkEdge(const VertexConstPtrPair &edge)
        {
#ifdef BITSTAR_DEBUG
            if (edge.first->isBlacklistedAsChild(edge.second))
            {
                throw ompl::Exception("A blacklisted edge made it into the edge queue.");
            }
#endif  // BITSTAR_DEBUG
        // If this is a whitelisted edge, there's no need to do (repeat) the collision checking.
            if (edge.first->isWhitelistedAsChild(edge.second))
            {
                return true;
            }
            else  // This is a new edge, we need to check whether it is feasible.
            {
                ++numEdgeCollisionChecks_;
                return Planner::si_->checkMotion(edge.first->state(), edge.second->state());
            }
        }

        void BITstar::addEdge(const VertexPtrPair &edge, const ompl::base::Cost &edgeCost)
        {
#ifdef BITSTAR_DEBUG
            if (edge.first->isInTree() == false)
            {
                throw ompl::Exception("Adding an edge from a vertex not connected to the graph");
            }
            if (costHelpPtr_->isCostEquivalentTo(costHelpPtr_->trueEdgeCost(edge), edgeCost) == false)
            {
                throw ompl::Exception("You have passed the wrong edge cost to addEdge.");
            }
#endif  // BITSTAR_DEBUG

            // If the child already has a parent, this is a rewiring.
            if (edge.second->hasParent())
            {
                // Replace the old parent.
                this->replaceParent(edge, edgeCost);
            }  // If not, we add the vertex without replaceing a parent.
            else
            {
                // Add a parent to the child.
                edge.second->addParent(edge.first, edgeCost);

                // Add a child to the parent.
                edge.first->addChild(edge.second);

                // Add the vertex to the set of vertices.
                graphPtr_->registerAsVertex(edge.second);
            }

            // If the vertex hasn't already been expanded, insert its outgoing edges
            if (!edge.second->isExpandedOnCurrentSearch())
            {
                queuePtr_->insertOutgoingEdges(edge.second);
            }
            else  // If the vertex has already been expanded, remember it as inconsistent.
            {
                queuePtr_->addToInconsistentSet(edge.second);
            }
        }

        void BITstar::replaceParent(const VertexPtrPair &edge, const ompl::base::Cost &edgeCost)
        {
#ifdef BITSTAR_DEBUG
            if (edge.second->getParent()->getId() == edge.first->getId())
            {
                throw ompl::Exception("The new and old parents of the given rewiring are the same.");
            }
            if (costHelpPtr_->isCostBetterThan(edge.second->getCost(),
                                               costHelpPtr_->combineCosts(edge.first->getCost(), edgeCost)) == true)
            {
                throw ompl::Exception("The new edge will increase the cost-to-come of the vertex!");
            }
#endif  // BITSTAR_DEBUG

            // Increment our counter:
            ++numRewirings_;

            // Remove the child from the parent, not updating costs
            edge.second->getParent()->removeChild(edge.second);

            // Remove the parent from the child, not updating costs
            edge.second->removeParent(false);

            // Add the parent to the child. updating the downstream costs.
            edge.second->addParent(edge.first, edgeCost);

            // Add the child to the parent.
            edge.first->addChild(edge.second);
        }

        void BITstar::updateGoalVertex()
        {
            // Variable
            // Whether we've updated the goal, be pessimistic.
            bool goalUpdated = false;
            // The new goal, start with the current goal
            VertexConstPtr newBestGoal = curGoalVertex_;
            // The new cost, start as the current bestCost_
            ompl::base::Cost newCost = bestCost_;

            // Iterate through the vector of goals, and see if the solution has changed
            for (auto it = graphPtr_->goalVerticesBeginConst(); it != graphPtr_->goalVerticesEndConst(); ++it)
            {
                // First, is this goal even in the tree?
                if ((*it)->isInTree())
                {
                    // Next, is there currently a solution?
                    if (static_cast<bool>(newBestGoal))
                    {
                        // There is already a solution, is it to this goal?
                        if ((*it)->getId() == newBestGoal->getId())
                        {
                            // Ah-ha, We meet again! Are we doing any better? We check the length as sometimes the path
                            // length changes with minimal change in cost.
                            if (!costHelpPtr_->isCostEquivalentTo((*it)->getCost(), newCost) ||
                                ((*it)->getDepth() + 1u) != bestLength_)
                            {
                                // The path to the current best goal has changed, so we need to update it.
                                goalUpdated = true;
                                newBestGoal = *it;
                                newCost = newBestGoal->getCost();
                            }
                            // No else, no change
                        }
                        else
                        {
                            // It is not to this goal, we have a second solution! What an easy problem... but is it
                            // better?
                            if (costHelpPtr_->isCostBetterThan((*it)->getCost(), newCost))
                            {
                                // It is! Save this as a better goal:
                                goalUpdated = true;
                                newBestGoal = *it;
                                newCost = newBestGoal->getCost();
                            }
                            // No else, not a better solution
                        }
                    }
                    else
                    {
                        // There isn't a preexisting solution, that means that any goal is an update:
                        goalUpdated = true;
                        newBestGoal = *it;
                        newCost = newBestGoal->getCost();
                    }
                }
                // No else, can't be a better solution if it's not in the spanning tree, can it?
            }

            // Did we update the goal?
            if (goalUpdated)
            {
                // Mark that we have a solution
                hasExactSolution_ = true;

                // Store the current goal
                curGoalVertex_ = newBestGoal;

                // Update the best cost:
                bestCost_ = newCost;

                // and best length
                bestLength_ = curGoalVertex_->getDepth() + 1u;

                // Tell everyone else about it.
                queuePtr_->registerSolutionCost(bestCost_);
                graphPtr_->registerSolutionCost(bestCost_);

                // Stop the solution loop if enabled:
                stopLoop_ = stopOnSolutionChange_;

                // Brag:
                this->goalMessage();

                // If enabled, pass the intermediate solution back through the call back:
                if (static_cast<bool>(Planner::pdef_->getIntermediateSolutionCallback()))
                {
                    // The form of path passed to the intermediate solution callback is not well documented, but it
                    // *appears* that it's not supposed
                    // to include the start or goal; however, that makes no sense for multiple start/goal problems, so
                    // we're going to include it anyway (sorry).
                    // Similarly, it appears to be ordered as (goal, goal-1, goal-2,...start+1, start) which
                    // conveniently allows us to reuse code.
                    Planner::pdef_->getIntermediateSolutionCallback()(this, this->bestPathFromGoalToStart(), bestCost_);
                }
            }
            // No else, the goal didn't change
        }

        void BITstar::goalMessage() const
        {
            OMPL_INFORM("%s (%u iters): Found a solution of cost %.4f (%u vertices) from %u samples by processing %u "
                        "edges (%u collision checked) to create %u vertices and perform %u rewirings. The graph "
                        "currently has %u vertices.",
                        Planner::getName().c_str(), numIterations_, bestCost_.value(), bestLength_,
                        graphPtr_->numStatesGenerated(), queuePtr_->numEdgesPopped(), numEdgeCollisionChecks_,
                        graphPtr_->numVerticesConnected(), numRewirings_, graphPtr_->numVertices());
        }

        void BITstar::endSuccessMessage() const
        {
            OMPL_INFORM("%s: Finished with a solution of cost %.4f (%u vertices) found from %u samples by processing "
                        "%u edges (%u collision checked) to create %u vertices and perform %u rewirings. The final "
                        "graph has %u vertices.",
                        Planner::getName().c_str(), bestCost_.value(), bestLength_, graphPtr_->numStatesGenerated(),
                        queuePtr_->numEdgesPopped(), numEdgeCollisionChecks_, graphPtr_->numVerticesConnected(),
                        numRewirings_, graphPtr_->numVertices());
        }

        void BITstar::endFailureMessage() const
        {
            if (graphPtr_->getTrackApproximateSolutions())
            {
                OMPL_INFORM("%s (%u iters): Did not find an exact solution from %u samples after processing %u edges "
                            "(%u collision checked) to create %u vertices and perform %u rewirings. The final graph "
                            "has %u vertices. The best approximate solution was %.4f from the goal and has a cost of "
                            "%.4f.",
                            Planner::getName().c_str(), numIterations_, graphPtr_->numStatesGenerated(),
                            queuePtr_->numEdgesPopped(), numEdgeCollisionChecks_, graphPtr_->numVerticesConnected(),
                            numRewirings_, graphPtr_->numVertices(), graphPtr_->smallestDistanceToGoal(),
                            graphPtr_->closestVertexToGoal()->getCost().value());
            }
            else
            {
                OMPL_INFORM("%s (%u iters): Did not find an exact solution from %u samples after processing %u edges "
                            "(%u collision checked) to create %u vertices and perform %u rewirings. The final graph "
                            "has %u vertices.",
                            Planner::getName().c_str(), numIterations_, graphPtr_->numStatesGenerated(),
                            queuePtr_->numEdgesPopped(), numEdgeCollisionChecks_, graphPtr_->numVerticesConnected(),
                            numRewirings_, graphPtr_->numVertices());
            }
        }

        void BITstar::statusMessage(const ompl::msg::LogLevel &logLevel, const std::string &status) const
        {
            // Check if we need to create the message
            if (logLevel >= ompl::msg::getLogLevel())
            {
                // Variable
                // The message as a stream:
                std::stringstream outputStream;

                // Create the stream:
                // The name of the planner
                outputStream << Planner::getName();
                outputStream << " (";
                // The current path cost:
                outputStream << "l: " << std::setw(6) << std::setfill(' ') << std::setprecision(5) << bestCost_.value();
                // The number of batches:
                outputStream << ", b: " << std::setw(5) << std::setfill(' ') << numBatches_;
                // The number of iterations
                outputStream << ", i: " << std::setw(5) << std::setfill(' ') << numIterations_;
                // The number of states current in the graph
                outputStream << ", g: " << std::setw(5) << std::setfill(' ') << graphPtr_->numVertices();
                // The number of free states
                outputStream << ", f: " << std::setw(5) << std::setfill(' ') << graphPtr_->numSamples();
                // The number edges in the queue:
                outputStream << ", q: " << std::setw(5) << std::setfill(' ') << queuePtr_->numEdges();
                // The total number of edges taken out of the queue:
                outputStream << ", t: " << std::setw(5) << std::setfill(' ') << queuePtr_->numEdgesPopped();
                // The number of samples generated
                outputStream << ", s: " << std::setw(5) << std::setfill(' ') << graphPtr_->numStatesGenerated();
                // The number of vertices ever added to the graph:
                outputStream << ", v: " << std::setw(5) << std::setfill(' ') << graphPtr_->numVerticesConnected();
                // The number of prunings:
                outputStream << ", p: " << std::setw(5) << std::setfill(' ') << numPrunings_;
                // The number of rewirings:
                outputStream << ", r: " << std::setw(5) << std::setfill(' ') << numRewirings_;
                // The number of nearest-neighbour calls
                outputStream << ", n: " << std::setw(5) << std::setfill(' ') << graphPtr_->numNearestLookups();
                // The number of state collision checks:
                outputStream << ", c(s): " << std::setw(5) << std::setfill(' ') << graphPtr_->numStateCollisionChecks();
                // The number of edge collision checks:
                outputStream << ", c(e): " << std::setw(5) << std::setfill(' ') << numEdgeCollisionChecks_;
                outputStream << "):    ";
                // The message:
                outputStream << status;

                if (logLevel == ompl::msg::LOG_DEBUG)
                {
                    OMPL_DEBUG("%s: ", outputStream.str().c_str());
                }
                else if (logLevel == ompl::msg::LOG_INFO)
                {
                    OMPL_INFORM("%s: ", outputStream.str().c_str());
                }
                else if (logLevel == ompl::msg::LOG_WARN)
                {
                    OMPL_WARN("%s: ", outputStream.str().c_str());
                }
                else if (logLevel == ompl::msg::LOG_ERROR)
                {
                    OMPL_ERROR("%s: ", outputStream.str().c_str());
                }
                else
                {
                    throw ompl::Exception("Log level not recognized");
                }
            }
            // No else, this message is below the log level
        }
        /////////////////////////////////////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////////////////////////////////////
        // Boring sets/gets (Public) and progress properties (Protected):
        void BITstar::setInitialInflationFactor(double factor)
        {
            initialInflationFactor_ = factor;
            queuePtr_->setInflationFactor(factor);
        }

        void BITstar::setInflationScalingParameter(double factor)
        {
            inflationScalingParameter_ = factor;
        }

        void BITstar::setTruncationScalingParameter(double factor)
        {
            truncationScalingParameter_ = factor;
        }

        void BITstar::enableCascadingRewirings(bool enable)
        {
            queuePtr_->enableCascadingRewirings(enable);
        }

        double BITstar::getInitialInflationFactor() const
        {
            return initialInflationFactor_;
        }

        double BITstar::getInflationScalingParameter() const
        {
            return inflationScalingParameter_;
        }

        double BITstar::getTruncationScalingParameter() const
        {
            return truncationScalingParameter_;
        }

        double BITstar::getCurrentInflationFactor() const
        {
            return queuePtr_->getInflationFactor();
        }

        double BITstar::getCurrentTruncationFactor() const
        {
            return truncationFactor_;
        }

        void BITstar::setRewireFactor(double rewireFactor)
        {
            graphPtr_->setRewireFactor(rewireFactor);
        }

        double BITstar::getRewireFactor() const
        {
            return graphPtr_->getRewireFactor();
        }

        void BITstar::setSamplesPerBatch(unsigned int samplesPerBatch)
        {
            samplesPerBatch_ = samplesPerBatch;
        }

        unsigned int BITstar::getSamplesPerBatch() const
        {
            return samplesPerBatch_;
        }

        void BITstar::setUseKNearest(bool useKNearest)
        {
            // Store
            graphPtr_->setUseKNearest(useKNearest);

            // If the planner is default named, we change it:
            if (!graphPtr_->getUseKNearest() && Planner::getName() == "kBITstar")
            {
                // It's current the default k-nearest BIT* name, and we're toggling, so set to the default r-disc
                OMPL_WARN("BIT*: An r-disc version of BIT* can not be named 'kBITstar', as this name is reserved for "
                          "the k-nearest version. Changing the name to 'BITstar'.");
                Planner::setName("BITstar");
            }
            else if (graphPtr_->getUseKNearest() && Planner::getName() == "BITstar")
            {
                // It's current the default r-disc BIT* name, and we're toggling, so set to the default k-nearest
                OMPL_WARN("BIT*: A k-nearest version of BIT* can not be named 'BITstar', as this name is reserved for "
                          "the r-disc version. Changing the name to 'kBITstar'.");
                Planner::setName("kBITstar");
            }
            else if (!graphPtr_->getUseKNearest() && Planner::getName() == "kABITstar")
            {
                // It's current the default k-nearest ABIT* name, and we're toggling, so set to the default r-disc
                OMPL_WARN("ABIT*: An r-disc version of ABIT* can not be named 'kABITstar', as this name is reserved "
                          "for the k-nearest version. Changing the name to 'ABITstar'.");
                Planner::setName("ABITstar");
            }
            else if (graphPtr_->getUseKNearest() && Planner::getName() == "ABITstar")
            {
                // It's current the default r-disc ABIT* name, and we're toggling, so set to the default k-nearest
                OMPL_WARN("ABIT*: A k-nearest version of ABIT* can not be named 'ABITstar', as this name is reserved "
                          "for the r-disc version. Changing the name to 'kABITstar'.");
                Planner::setName("kABITstar");
            }
            // It's not default named, don't change it
        }

        bool BITstar::getUseKNearest() const
        {
            return graphPtr_->getUseKNearest();
        }

        void BITstar::setStrictQueueOrdering(bool /* beStrict */)
        {
            OMPL_WARN("%s: This option no longer has any effect; The queue is always strictly ordered.",
                      Planner::getName().c_str());
        }

        bool BITstar::getStrictQueueOrdering() const
        {
            OMPL_WARN("%s: This option no longer has any effect; The queue is always strictly ordered.",
                      Planner::getName().c_str());
            return true;
        }

        void BITstar::setPruning(bool usePruning)
        {
            isPruningEnabled_ = usePruning;
            graphPtr_->setPruning(usePruning);
        }

        bool BITstar::getPruning() const
        {
            return isPruningEnabled_;
        }

        void BITstar::setPruneThresholdFraction(double fractionalChange)
        {
            if (fractionalChange < 0.0 || fractionalChange > 1.0)
            {
                throw ompl::Exception("Prune threshold must be specified as a fraction between [0, 1].");
            }

            pruneFraction_ = fractionalChange;
        }

        double BITstar::getPruneThresholdFraction() const
        {
            return pruneFraction_;
        }

        void BITstar::setDelayRewiringUntilInitialSolution(bool /* delayRewiring */)
        {
            OMPL_WARN("%s: This option no longer has any effect; Rewiring is never delayed.",
                      Planner::getName().c_str());
        }

        bool BITstar::getDelayRewiringUntilInitialSolution() const
        {
            OMPL_WARN("%s: This option no longer has any effect; Rewiring is never delayed.",
                      Planner::getName().c_str());
            return false;
        }

        void BITstar::setJustInTimeSampling(bool useJit)
        {
            graphPtr_->setJustInTimeSampling(useJit);
        }

        bool BITstar::getJustInTimeSampling() const
        {
            return graphPtr_->getJustInTimeSampling();
        }

        void BITstar::setDropSamplesOnPrune(bool dropSamples)
        {
            graphPtr_->setDropSamplesOnPrune(dropSamples);
        }

        bool BITstar::getDropSamplesOnPrune() const
        {
            return graphPtr_->getDropSamplesOnPrune();
        }

        void BITstar::setStopOnSolnImprovement(bool stopOnChange)
        {
            stopOnSolutionChange_ = stopOnChange;
        }

        bool BITstar::getStopOnSolnImprovement() const
        {
            return stopOnSolutionChange_;
        }

        void BITstar::setConsiderApproximateSolutions(bool findApproximate)
        {
            // Store
            graphPtr_->setTrackApproximateSolutions(findApproximate);

            // Mark the Planner spec:
            Planner::specs_.approximateSolutions = graphPtr_->getTrackApproximateSolutions();
        }

        bool BITstar::getConsiderApproximateSolutions() const
        {
            return graphPtr_->getTrackApproximateSolutions();
        }

        void BITstar::setAverageNumOfAllowedFailedAttemptsWhenSampling(std::size_t number)
        {
            graphPtr_->setAverageNumOfAllowedFailedAttemptsWhenSampling(number);
        }

        std::size_t BITstar::getAverageNumOfAllowedFailedAttemptsWhenSampling() const
        {
            return graphPtr_->getAverageNumOfAllowedFailedAttemptsWhenSampling();
        }

        template <template <typename T> class NN>
        void BITstar::setNearestNeighbors()
        {
            // Check if the problem is already setup, if so, the NN structs have data in them and you can't really
            // change them:
            if (Planner::setup_)
            {
                OMPL_WARN("%s: The nearest neighbour datastructures cannot be changed once the planner is setup. "
                          "Continuing to use the existing containers.",
                          Planner::getName().c_str());
            }
            else
            {
                graphPtr_->setNearestNeighbors<NN>();
            }
        }

        std::string BITstar::bestCostProgressProperty() const
        {
            return ompl::toString(bestCost_.value());
        }

        std::string BITstar::bestLengthProgressProperty() const
        {
            return std::to_string(bestLength_);
        }

        std::string BITstar::edgeQueueSizeProgressProperty() const
        {
            return std::to_string(queuePtr_->numEdges());
        }

        std::string BITstar::iterationProgressProperty() const
        {
            return std::to_string(numIterations_);
        }

        std::string BITstar::batchesProgressProperty() const
        {
            return std::to_string(numBatches_);
        }

        std::string BITstar::pruningProgressProperty() const
        {
            return std::to_string(numPrunings_);
        }

        std::string BITstar::totalStatesCreatedProgressProperty() const
        {
            return std::to_string(graphPtr_->numStatesGenerated());
        }

        std::string BITstar::verticesConstructedProgressProperty() const
        {
            return std::to_string(graphPtr_->numVerticesConnected());
        }

        std::string BITstar::statesPrunedProgressProperty() const
        {
            return std::to_string(graphPtr_->numFreeStatesPruned());
        }

        std::string BITstar::verticesDisconnectedProgressProperty() const
        {
            return std::to_string(graphPtr_->numVerticesDisconnected());
        }

        std::string BITstar::rewiringProgressProperty() const
        {
            return std::to_string(numRewirings_);
        }

        std::string BITstar::stateCollisionCheckProgressProperty() const
        {
            return std::to_string(graphPtr_->numStateCollisionChecks());
        }

        std::string BITstar::edgeCollisionCheckProgressProperty() const
        {
            return std::to_string(numEdgeCollisionChecks_);
        }

        std::string BITstar::nearestNeighbourProgressProperty() const
        {
            return std::to_string(graphPtr_->numNearestLookups());
        }

        std::string BITstar::edgesProcessedProgressProperty() const
        {
            return std::to_string(queuePtr_->numEdgesPopped());
        }
        /////////////////////////////////////////////////////////////////////////////////////////////
    }  // namespace geometric
}  // namespace ompl
