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

/* Authors: Jonathan Gammell */

// My definition:
#include "ompl/geometric/planners/bitstar/BITstar.h"

// For, you know, math
#include <cmath>
// For stringstreams
#include <sstream>
// For stream manipulations
#include <iomanip>
// For smart pointers
#include <memory>
// For boost math constants
#include <boost/math/constants/constants.hpp>
// For boost::adaptors::reverse which let "for (auto ...)" loops iterate in reverse
#include <boost/range/adaptor/reversed.hpp>

// For OMPL_INFORM et al.
#include "ompl/util/Console.h"
// For exceptions:
#include "ompl/util/Exception.h"
// For geometric equations like unitNBallMeasure
#include "ompl/util/GeometricEquations.h"
// For ompl::base::GoalSampleableRegion, which both GoalState and GoalStates derive from:
#include "ompl/base/goals/GoalSampleableRegion.h"
// For getDefaultNearestNeighbors
#include "ompl/tools/config/SelfConfig.h"
// For ompl::geometric::path
#include "ompl/geometric/PathGeometric.h"
// For the default optimization objective:
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"

namespace ompl
{
    namespace geometric
    {
        /////////////////////////////////////////////////////////////////////////////////////////////
        // Public functions:
        BITstar::BITstar(const ompl::base::SpaceInformationPtr &si, const std::string &name /*= "BITstar"*/)
          : ompl::base::Planner(si, name)
          , sampler_()
          , opt_()
          , startVerticesPtr_()
          , goalVerticesPtr_()
          , costHelpPtr_()
          , prunedStartVertices_()
          , prunedGoalVertices_()
          , curGoalVertex_()
          , freeStateNN_()
          , vertexNN_()
          , intQueue_()
          , newSamples_()
          , recycledSamples_()
          , numUniformStates_(0u)
          , r_(0.0)                                             // Purposeful Gibberish
          , k_rgg_(0.0)                                         // Purposeful Gibberish
          , k_(0u)                                              // Purposeful Gibberish
          , bestCost_(std::numeric_limits<double>::infinity())  // Gets set in setup to the proper calls from
                                                                // OptimizationObjective
          , bestLength_(0u)
          , prunedCost_(std::numeric_limits<double>::infinity())  // Gets set in setup to the proper calls from
                                                                  // OptimizationObjective
          , prunedMeasure_(0.0)  // Gets set in setup with the proper call to Planner::si_->getSpaceMeasure()
          , minCost_(std::numeric_limits<double>::infinity())      // Gets set in setup to the proper calls from
                                                                   // OptimizationObjective
          , costSampled_(std::numeric_limits<double>::infinity())  // Gets set in setup to the proper calls from
                                                                   // OptimizationObjective
          , hasExactSolution_(false)
          , stopLoop_(false)
          , approxGoalVertex_()
          , approxDiff_(std::numeric_limits<double>::infinity())
          , numIterations_(0u)
          , numBatches_(0u)
          , numPrunings_(0u)
          , numSamples_(0u)
          , numVertices_(0u)
          , numFreeStatesPruned_(0u)
          , numVerticesDisconnected_(0u)
          , numRewirings_(0u)
          , numStateCollisionChecks_(0u)
          , numEdgeCollisionChecks_(0u)
          , numNearestNeighbours_(0u)
          , numEdgesProcessed_(0u)
          , useStrictQueueOrdering_(false)
          , rewireFactor_(1.1)
          , samplesPerBatch_(100u)
          , useKNearest_(true)
          , usePruning_(true)
          , pruneFraction_(0.05)
          , delayRewiring_(true)
          , useJustInTimeSampling_(false)
          , dropSamplesOnPrune_(false)
          , stopOnSolnChange_(false)
          , findApprox_(false)
        {
            // Make sure the default name reflects the default k-nearest setting, if not overridden to something else
            if (useKNearest_ == true && Planner::getName() == "BITstar")
            {
                // It's the current default r-disc BIT* name, but we're using k-nearest, so change
                Planner::setName("kBITstar");
            }
            else if (useKNearest_ == false && Planner::getName() == "kBITstar")
            {
                // It's the current default k-nearest BIT* name, but we're using r-disc, so change
                Planner::setName("BITstar");
            }
            // It's not default named, don't change it

            // Specify my planner specs:
            Planner::specs_.recognizedGoal = ompl::base::GOAL_SAMPLEABLE_REGION;
            Planner::specs_.multithreaded = false;
            Planner::specs_.approximateSolutions = findApprox_;
            Planner::specs_.optimizingPaths = true;
            Planner::specs_.directed = true;
            Planner::specs_.provingSolutionNonExistence = false;
            Planner::specs_.canReportIntermediateSolutions = true;

            // Register my setting callbacks
            Planner::declareParam<double>("rewire_factor", this, &BITstar::setRewireFactor, &BITstar::getRewireFactor,
                                          "1.0:0.01:3.0");
            Planner::declareParam<unsigned int>("samples_per_batch", this, &BITstar::setSamplesPerBatch,
                                                &BITstar::getSamplesPerBatch, "1:1:1000000");
            Planner::declareParam<bool>("use_k_nearest", this, &BITstar::setKNearest, &BITstar::getKNearest, "0,1");
            Planner::declareParam<bool>("use_graph_pruning", this, &BITstar::setPruning, &BITstar::getPruning, "0,1");
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
            addPlannerProgressProperty("best cost DOUBLE", [this]
                                       {
                                           return bestCostProgressProperty();
                                       });
            addPlannerProgressProperty("number of segments in solution path INTEGER", [this]
                                       {
                                           return bestLengthProgressProperty();
                                       });
            addPlannerProgressProperty("current free states INTEGER", [this]
                                       {
                                           return currentFreeProgressProperty();
                                       });
            addPlannerProgressProperty("current graph vertices INTEGER", [this]
                                       {
                                           return currentVertexProgressProperty();
                                       });
            addPlannerProgressProperty("state collision checks INTEGER", [this]
                                       {
                                           return stateCollisionCheckProgressProperty();
                                       });
            addPlannerProgressProperty("edge collision checks INTEGER", [this]
                                       {
                                           return edgeCollisionCheckProgressProperty();
                                       });
            addPlannerProgressProperty("nearest neighbour calls INTEGER", [this]
                                       {
                                           return nearestNeighbourProgressProperty();
                                       });

            // Extra progress info that aren't necessary for every day use. Uncomment if desired.
            // addPlannerProgressProperty("vertex queue size INTEGER", [this] { return
            // vertexQueueSizeProgressProperty(); });
            // addPlannerProgressProperty("edge queue size INTEGER", [this] { return edgeQueueSizeProgressProperty();
            // });
            // addPlannerProgressProperty("iterations INTEGER", [this] { return iterationProgressProperty(); });
            // addPlannerProgressProperty("batches INTEGER", [this] { return batchesProgressProperty(); });
            // addPlannerProgressProperty("graph prunings INTEGER", [this] { return pruningProgressProperty(); });
            // addPlannerProgressProperty("total states generated INTEGER", [this] { return
            // totalStatesCreatedProgressProperty(); });
            // addPlannerProgressProperty("vertices constructed INTEGER", [this] { return
            // verticesConstructedProgressProperty(); });
            // addPlannerProgressProperty("states pruned INTEGER", [this] { return statesPrunedProgressProperty(); });
            // addPlannerProgressProperty("graph vertices disconnected INTEGER", [this] { return
            // verticesDisconnectedProgressProperty(); });
            // addPlannerProgressProperty("rewiring edges INTEGER", [this] { return rewiringProgressProperty(); });
        }

        BITstar::~BITstar() = default;

        void BITstar::setup()
        {
            // Call the base class setup:
            Planner::setup();

            // Do some sanity checks
            // Make sure we have a problem definition
            if (static_cast<bool>(Planner::pdef_) == false)
            {
                OMPL_ERROR("%s::setup() was called without a problem definition.", Planner::getName().c_str());
                Planner::setup_ = false;
                return;
            }

            // Make sure we have an optimization objective
            if (Planner::pdef_->hasOptimizationObjective() == false)
            {
                OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length.",
                            Planner::getName().c_str());
                Planner::pdef_->setOptimizationObjective(
                    std::make_shared<base::PathLengthOptimizationObjective>(Planner::si_));
            }

            // If the problem definition *has* a goal, make sure it is of appropriate type
            if (static_cast<bool>(Planner::pdef_->getGoal()) == true)
            {
                if (Planner::pdef_->getGoal()->hasType(ompl::base::GOAL_SAMPLEABLE_REGION) == false)
                {
                    OMPL_ERROR("%s::setup() BIT* currently only supports goals that can be cast to a sampleable goal "
                               "region (i.e., are countable sets).",
                               Planner::getName().c_str());
                    Planner::setup_ = false;
                    return;
                }
                // No else, of correct type.
            }
            // No else, called without a goal. Is this MoveIt?

            // Store the optimization objective for future ease of use
            opt_ = Planner::pdef_->getOptimizationObjective();

            // Allocated the starts list
            if (static_cast<bool>(startVerticesPtr_) == false)
            {
                startVerticesPtr_ = std::make_shared<VertexPtrList>();
            }
            // No else, already allocated

            // Allocated the goals list
            if (static_cast<bool>(goalVerticesPtr_) == false)
            {
                goalVerticesPtr_ = std::make_shared<VertexPtrList>();
            }
            // No else, already allocated

            // Allocated the cost-helper
            if (static_cast<bool>(costHelpPtr_) == false)
            {
                costHelpPtr_ = std::make_shared<CostHelper>(opt_, startVerticesPtr_, goalVerticesPtr_);
            }
            // No else, already allocated

            // Configure the nearest-neighbour constructs.
            // Only allocate if they are empty (as they can be set to a specific version by a call to
            // setNearestNeighbors)
            if (static_cast<bool>(freeStateNN_) == false)
            {
                freeStateNN_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<VertexPtr>(this));
            }
            // No else, already allocated (by a call to setNearestNeighbors())

            if (static_cast<bool>(vertexNN_) == false)
            {
                vertexNN_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<VertexPtr>(this));
            }
            // No else, already allocated (by a call to setNearestNeighbors())

            // Configure:
            NearestNeighbors<VertexPtr>::DistanceFunction distfun(
                [this](const VertexConstPtr &a, const VertexConstPtr &b)
                {
                    return nnDistance(a, b);
                });
            freeStateNN_->setDistanceFunction(distfun);
            vertexNN_->setDistanceFunction(distfun);

            // Configure the queue
            intQueue_ = std::make_shared<IntegratedQueue>(costHelpPtr_,
                                                          [this](const VertexConstPtr &a, const VertexConstPtr &b)
                                                          {
                                                              return nnDistance(a, b);
                                                          },
                                                          [this](const VertexPtr &a, VertexPtrVector *b)
                                                          {
                                                              return nearestSamples(a, b);
                                                          },
                                                          [this](const VertexPtr &a, VertexPtrVector *b)
                                                          {
                                                              return nearestVertices(a, b);
                                                          });
            intQueue_->setDelayedRewiring(delayRewiring_);

            // Set the best-cost, pruned-cost, sampled-cost and min-cost to the proper opt_-based values:
            bestCost_ = costHelpPtr_->infiniteCost();
            prunedCost_ = costHelpPtr_->infiniteCost();
            minCost_ = costHelpPtr_->infiniteCost();
            costSampled_ = costHelpPtr_->infiniteCost();

            // Add any start and goals vertices that exist to the queue, but do NOT wait for any more goals:
            this->updateStartAndGoalStates(ompl::base::plannerAlwaysTerminatingCondition());

            // Get the measure of the problem
            prunedMeasure_ = Planner::si_->getSpaceMeasure();

            // Does the problem have finite boundaries?
            if (std::isfinite(prunedMeasure_) == false)
            {
                // It does not, so let's estimate a measure of the planning problem.
                // A not horrible place to start would be hypercube proportional to the distance between the start and
                // goal. It's not *great*, but at least it sort of captures the order-of-magnitude of the problem.

                // First, some asserts.
                // Check that JIT sampling is on, which is required for infinite problems
                if (useJustInTimeSampling_ == false)
                {
                    throw ompl::Exception("For unbounded planning problems, just-in-time sampling must be enabled "
                                          "before calling setup.");
                }
                // No else

                // Check that we have a start and goal
                if (startVerticesPtr_->empty() == true || goalVerticesPtr_->empty() == true)
                {
                    throw ompl::Exception("For unbounded planning problems, at least one start and one goal must exist "
                                          "before calling setup.");
                }
                // No else

                // Variables
                // The maximum distance between start and goal:
                double maxDist = 0.0;
                // The scale on the maximum distance, i.e. the width of the hypercube is equal to this value times the
                // distance between start and goal.
                // This number is completely made up.
                double distScale = 2.0;

                // Find the max distance
                for (const auto &startVertex : *startVerticesPtr_)
                {
                    for (const auto &goalVertex : *goalVerticesPtr_)
                    {
                        maxDist = std::max(maxDist,
                                           Planner::si_->distance(startVertex->stateConst(), goalVertex->stateConst()));
                    }
                }

                // Calculate an estimate of the problem measure by (hyper)cubing the max distance
                prunedMeasure_ = std::pow(distScale * maxDist, Planner::si_->getStateDimension());
            }
            // No else, finite problem dimension

            // Finally initialize the nearestNeighbour terms:
            this->initializeNearestTerms();

            // Debug: Output an estimate of the state measure:
            // this->estimateMeasures();
        }

        void BITstar::clear()
        {
            // Clear all the variables.
            // Keep this in the order of the constructors list:

            // The various convenience pointers:
            sampler_.reset();
            opt_.reset();
            startVerticesPtr_->clear();
            startVerticesPtr_.reset();
            goalVerticesPtr_->clear();
            goalVerticesPtr_.reset();
            costHelpPtr_.reset();
            prunedStartVertices_.clear();
            prunedGoalVertices_.clear();
            curGoalVertex_.reset();

            // The list of samples
            if (static_cast<bool>(freeStateNN_) == true)
            {
                freeStateNN_->clear();
                freeStateNN_.reset();
            }
            // No else, not allocated

            // The list of vertices
            if (static_cast<bool>(vertexNN_) == true)
            {
                vertexNN_->clear();
                vertexNN_.reset();
            }

            // The list of new and recycled samples
            newSamples_.clear();
            recycledSamples_.clear();

            // The queue:
            if (static_cast<bool>(intQueue_) == true)
            {
                intQueue_->clear();
                intQueue_.reset();
            }

            // DO NOT reset the parameters:
            // useStrictQueueOrdering_
            // rewireFactor_
            // samplesPerBatch_
            // useKNearest_
            // usePruning_
            // pruneFraction_
            // delayRewiring_
            // useJustInTimeSampling_
            // dropSamplesOnPrune_
            // stopOnSolnChange_

            // Reset the various calculations? TODO: Should I recalculate them?
            numUniformStates_ = 0u;
            r_ = 0.0;
            k_rgg_ = 0.0;  // This is a double for better rounding later
            k_ = 0u;
            bestCost_ = ompl::base::Cost(std::numeric_limits<double>::infinity());
            bestLength_ = 0u;
            prunedCost_ = ompl::base::Cost(std::numeric_limits<double>::infinity());
            prunedMeasure_ = Planner::si_->getSpaceMeasure();
            minCost_ = ompl::base::Cost(0.0);
            costSampled_ = ompl::base::Cost(0.0);
            hasExactSolution_ = false;
            stopLoop_ = false;
            approxGoalVertex_.reset();
            approxDiff_ = std::numeric_limits<double>::infinity();
            numIterations_ = 0u;
            numSamples_ = 0u;
            numVertices_ = 0u;
            numFreeStatesPruned_ = 0u;
            numVerticesDisconnected_ = 0u;
            numStateCollisionChecks_ = 0u;
            numEdgeCollisionChecks_ = 0u;
            numNearestNeighbours_ = 0u;
            numEdgesProcessed_ = 0u;
            numRewirings_ = 0u;
            numBatches_ = 0u;
            numPrunings_ = 0u;

            // Mark as not setup:
            Planner::setup_ = false;

            // Call my base clear:
            Planner::clear();
        }

        ompl::base::PlannerStatus BITstar::solve(const ompl::base::PlannerTerminationCondition &ptc)
        {
            Planner::checkValidity();
            OMPL_INFORM("%s: Searching for a solution to the given planning problem.", Planner::getName().c_str());

            // Reset the manual stop to the iteration loop:
            stopLoop_ = false;

            // If we don't have a goal yet, recall updateStartAndGoalStates, but wait for the first goal (or until the
            // PTC comes true and we give up):
            if (goalVerticesPtr_->empty() == true)
            {
                this->updateStartAndGoalStates(ptc);
            }

            // Run the outerloop until we're stopped, a suitable cost is found, or until we find the minimum possible
            // cost within tolerance:
            while (opt_->isSatisfied(bestCost_) == false && ptc == false &&
                   (costHelpPtr_->isCostBetterThan(minCost_, bestCost_) == true ||
                    Planner::pis_.haveMoreStartStates() == true || Planner::pis_.haveMoreGoalStates() == true) &&
                   stopLoop_ == false)
            {
                this->iterate();
            }

            // Announce
            if (hasExactSolution_ == true)
            {
                this->endSuccessMessage();
            }
            else
            {
                this->endFailureMessage();
            }

            // Publish
            if (hasExactSolution_ == true || findApprox_ == true)
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
            return ompl::base::PlannerStatus(hasExactSolution_ || findApprox_, !hasExactSolution_ && findApprox_);
        }

        void BITstar::getPlannerData(ompl::base::PlannerData &data) const
        {
            // Get the base planner class data:
            Planner::getPlannerData(data);

            // Add samples
            if (freeStateNN_)
            {
                // Variables:
                // The list of unused samples:
                VertexPtrVector samples;

                // Get the list of samples
                freeStateNN_->list(samples);

                // Iterate through it turning each into a disconnected vertex
                for (const auto &freeSample : samples)
                {
                    // No, add as a regular vertex:
                    data.addVertex(ompl::base::PlannerDataVertex(freeSample->stateConst()));
                }
            }
            // No else.

            // Add vertices
            if (vertexNN_)
            {
                // Variables:
                // The list of vertices in the graph:
                VertexPtrVector vertices;

                // Get the list of vertices
                vertexNN_->list(vertices);

                // Iterate through it turning each into a vertex with an edge:
                for (const auto &vertex : vertices)
                {
                    // Is the vertex the start?
                    if (vertex->isRoot() == true)
                    {
                        // Yes, add as a start vertex:
                        data.addStartVertex(ompl::base::PlannerDataVertex(vertex->stateConst()));
                    }
                    else
                    {
                        // No, add as a regular vertex:
                        data.addVertex(ompl::base::PlannerDataVertex(vertex->stateConst()));

                        // And as an incoming edge
                        data.addEdge(ompl::base::PlannerDataVertex(vertex->getParentConst()->stateConst()),
                                     ompl::base::PlannerDataVertex(vertex->stateConst()));
                    }
                }
            }
            // No else.

            // Did we find a solution?
            if (hasExactSolution_ == true)
            {
                // Exact solution
                data.markGoalState(curGoalVertex_->stateConst());
            }
            else if (hasExactSolution_ == false && findApprox_ == true)
            {
                // Approximate solution
                data.markGoalState(approxGoalVertex_->stateConst());
            }
            // No else, no solution
        }

        std::pair<ompl::base::State const *, ompl::base::State const *> BITstar::getNextEdgeInQueue()
        {
            // Variable:
            // The next edge as a basic pair of states
            std::pair<ompl::base::State const *, ompl::base::State const *> nextEdge;

            // If we're using strict queue ordering, make sure the queue is up to date
            if (useStrictQueueOrdering_ == true)
            {
                // Resort the queues as necessary (if the graph has been rewired).
                this->resort();
            }

            if (intQueue_->isEmpty() == false)
            {
                // The next edge in the queue:
                nextEdge =
                    std::make_pair(intQueue_->frontEdge().first->state(), intQueue_->frontEdge().second->state());
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

            // If we're using strict queue ordering, make sure the queue is up to date
            if (useStrictQueueOrdering_ == true)
            {
                // Resort the queues as necessary (if the graph has been rewired).
                this->resort();
            }

            if (intQueue_->isEmpty() == false)
            {
                // The next cost in the queue:
                nextCost = intQueue_->frontEdgeValue().at(0u);
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
            intQueue_->getEdges(edgesInQueue);
        }

        void BITstar::getVertexQueue(VertexConstPtrVector *verticesInQueue)
        {
            intQueue_->getVertices(verticesInQueue);
        }

        template <template <typename T> class NN>
        void BITstar::setNearestNeighbors()
        {
            // Check if the problem is already setup, if so, the NN structs have data in them and you can't really
            // change them:
            if (Planner::setup_ == true)
            {
                throw ompl::Exception("The type of nearest neighbour datastructure cannot be changed once a planner is "
                                      "setup. ");
            }
            else
            {
                // The problem isn't setup yet, create NN structs of the specified type:
                freeStateNN_ = std::make_shared<NN<VertexPtr>>();
                vertexNN_ = std::make_shared<NN<VertexPtr>>();
            }
        }
        /////////////////////////////////////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////////////////////////////////////
        // Protected functions:
        void BITstar::estimateMeasures()
        {
            OMPL_INFORM("%s: Estimating the measure of the planning domain. This is a debugging function that does not "
                        "have any effect on the planner.",
                        Planner::getName().c_str());
            // Variables:
            // The total number of samples:
            unsigned int numTotalSamples;
            // The resulting samples in free:
            unsigned int numFreeSamples;
            // The resulting samples in obs:
            unsigned int numObsSamples;
            // The sample fraction of free:
            double fractionFree;
            // The sample fraction of obs:
            double fractionObs;
            // The total measure of the space:
            double totalMeasure;
            // The resulting estimate of the free measure
            double freeMeasure;
            // The resulting estimate of the obs measure
            double obsMeasure;

            // Set the total number of samples
            numTotalSamples = 100000u;
            numFreeSamples = 0u;
            numObsSamples = 0u;

            // Draw samples, classifying each one
            for (unsigned int i = 0u; i < numTotalSamples; ++i)
            {
                // Allocate a state
                ompl::base::State *aState = Planner::si_->allocState();

                // Sample:
                sampler_->sampleUniform(aState, bestCost_);

                // Check if collision free
                if (Planner::si_->isValid(aState) == true)
                {
                    ++numFreeSamples;
                }
                else
                {
                    ++numObsSamples;
                }
            }

            // Calculate the fractions:
            fractionFree = static_cast<double>(numFreeSamples) / static_cast<double>(numTotalSamples);

            fractionObs = static_cast<double>(numObsSamples) / static_cast<double>(numTotalSamples);

            // Get the total measure of the space
            totalMeasure = Planner::si_->getSpaceMeasure();

            // Calculate the measure of the free space
            freeMeasure = fractionFree * totalMeasure;

            // Calculate the measure of the obs space
            obsMeasure = fractionObs * totalMeasure;

            // Announce
            OMPL_INFORM("%s: %u samples (%u free, %u in collision) from a space with measure %.4f estimates %.2f%% "
                        "free and %.2f%% in collision (measures of %.4f and %.4f, respectively).",
                        Planner::getName().c_str(), numTotalSamples, numFreeSamples, numObsSamples, totalMeasure,
                        100.0 * fractionFree, 100.0 * fractionObs, freeMeasure, obsMeasure);
        }

        void BITstar::iterate()
        {
            // Info:
            ++numIterations_;

            // If we're using strict queue ordering, make sure the queues are up to date
            if (useStrictQueueOrdering_ == true)
            {
                // The queues will be resorted if the graph has been rewired.
                this->resort();
            }

            // Is the edge queue empty
            if (intQueue_->isEmpty() == true)
            {
                // Is it also unsorted?
                if (intQueue_->isSorted() == false)
                {
                    // If it is, then we've hit a rare condition where we emptied it without having to sort it, so
                    // address that
                    this->resort();
                }
                else
                {
                    // If not, then we're either just starting the problem, or just finished a batch. Either way, make a
                    // batch of samples and fill the queue for the first time:
                    this->newBatch();
                }
            }
            else
            {
                // If the edge queue is not empty, then there is work to do!

                // Variables:
                // The current edge:
                VertexPtrPair bestEdge;

                // Pop the minimum edge
                ++numEdgesProcessed_;
                intQueue_->popFrontEdge(&bestEdge);

                // In the best case, can this edge improve our solution given the current graph?
                // g_t(v) + c_hat(v,x) + h_hat(x) < g_t(x_g)?
                if (costHelpPtr_->isCostBetterThan(costHelpPtr_->currentHeuristicEdge(bestEdge), bestCost_) == true)
                {
                    // What about improving the current graph?
                    // g_t(v) + c_hat(v,x)  < g_t(x)?
                    if (costHelpPtr_->isCostBetterThan(costHelpPtr_->currentHeuristicTarget(bestEdge),
                                                       bestEdge.second->getCost()) == true)
                    {
                        // Ok, so it *could* be a useful edge. Do the work of calculating its cost for real

                        // Variables:
                        // The true cost of the edge:
                        ompl::base::Cost trueEdgeCost;

                        // Get the true cost of the edge
                        trueEdgeCost = this->trueEdgeCost(bestEdge);

                        // Can this actual edge ever improve our solution?
                        // g_hat(v) + c(v,x) + h_hat(x) < g_t(x_g)?
                        if (costHelpPtr_->isCostBetterThan(
                                costHelpPtr_->combineCosts(costHelpPtr_->costToComeHeuristic(bestEdge.first),
                                                           trueEdgeCost,
                                                           costHelpPtr_->costToGoHeuristic(bestEdge.second)),
                                bestCost_) == true)
                        {
                            // Does this edge have a collision?
                            if (this->checkEdge(bestEdge) == true)
                            {
                                // Does the current edge improve our graph?
                                // g_t(v) + c(v,x) < g_t(x)?
                                if (costHelpPtr_->isCostBetterThan(
                                        costHelpPtr_->combineCosts(bestEdge.first->getCost(), trueEdgeCost),
                                        bestEdge.second->getCost()) == true)
                                {
                                    // YAAAAH. Add the edge! Allowing for the sample to be removed from free if it is
                                    // not currently connected and otherwise propagate cost updates to descendants.
                                    // addEdge will update the queue and handle the extra work that occurs if this edge
                                    // improves the solution.
                                    this->addEdge(bestEdge, trueEdgeCost, true, true);

                                    /*
                                    //Remove any unnecessary incoming edges in the edge queue
                                    intQueue_->updateEdgesTo(bestEdge.second);
                                    */

                                    // We will only prune the whole graph/samples on a new batch.
                                }
                                // No else, this edge may be useful at some later date.
                            }
                            // No else, we failed
                        }
                        // No else, we failed
                    }
                    // No else, we failed
                }
                else if (intQueue_->isSorted() == false)
                {
                    // The edge cannot improve our solution, but the queue is imperfectly sorted, so we must resort
                    // before we give up.
                    this->resort();
                }
                else
                {
                    // Else, I cannot improve the current solution, and as the queue is perfectly sorted and I am the
                    // best edge, no one can improve the current solution . Give up on the batch:
                    intQueue_->finish();
                }
            }  // Integrated queue not empty.
        }

        void BITstar::newBatch()
        {
            // Info:
            ++numBatches_;

            // Reset the queue:
            intQueue_->reset();

            // Do we need to update our starts or goals?
            if (Planner::pis_.haveMoreStartStates() == true || Planner::pis_.haveMoreGoalStates() == true)
            {
                // There are new starts/goals to get.
                this->updateStartAndGoalStates(ompl::base::plannerAlwaysTerminatingCondition());
            }
            // No else, we have enough of a problem to do some work, and everything's up to date.

            // Prune the graph (if enabled)
            this->prune();

            // Set the cost sampled to the minimum
            costSampled_ = minCost_;

            // Update the nearest-neighbour terms for the number of samples we *will* have.
            this->updateNearestTerms();

            // Relabel all the previous samples as old
            for (auto &freeSample : newSamples_)
            {
                // If the sample still exists, mark as old. It can get pruned during a resort.
                if (freeSample->isPruned() == false)
                {
                    freeSample->markOld();
                }
                // No else, this sample has been pruned and will shortly disappear
            }

            // Clear the list of new samples
            newSamples_.clear();

            // Make the recycled vertices to new:
            newSamples_ = recycledSamples_;

            // Clear the list of recycled
            recycledSamples_.clear();
        }

        void BITstar::updateSamples(const VertexConstPtr &vertex)
        {
            // Variable
            // The required cost to contain the neighbourhood of this vertex:
            ompl::base::Cost costReqd = neighbourhoodCost(vertex);

            // Check if we need to generate new samples inorder to completely cover the neighbourhood of the vertex
            if (costHelpPtr_->isCostBetterThan(costSampled_, costReqd))
            {
                // Variable
                // The total number of samples we wish to have.
                unsigned int totalReqdSamples;

                // Get the measure of what we're sampling
                if (useJustInTimeSampling_ == true)
                {
                    // Variables
                    // The sample density for this slice of the problem.
                    double sampleDensity;
                    // The resulting number of samples needed for this slice as a *double*
                    double dblNum;

                    // Calculate the sample density given the number of samples per batch and the measure of this batch
                    // by assuming that this batch will fill the same measure as the previous
                    sampleDensity = static_cast<double>(samplesPerBatch_) / prunedMeasure_;

                    // Convert that into the number of samples needed for this slice.
                    dblNum = sampleDensity * sampler_->getInformedMeasure(costSampled_, costReqd);

                    // The integer of the double are definitely sampled
                    totalReqdSamples = numSamples_ + static_cast<unsigned int>(dblNum);

                    // And the fractional part represents the probability of one more sample. I like being pedantic.
                    if (rng_.uniform01() <= (dblNum - static_cast<double>(totalReqdSamples)))
                    {
                        // One more please
                        ++totalReqdSamples;
                    }
                    // No else.
                }
                else
                {
                    // We're generating all our samples in one batch. Do it to it.
                    totalReqdSamples = numSamples_ + samplesPerBatch_;
                }

                // Actually generate the new samples
                while (numSamples_ < totalReqdSamples)
                {
                    // Variable
                    // The new state:
                    auto newState = std::make_shared<Vertex>(Planner::si_, opt_);

                    // Sample in the interval [costSampled_, costReqd):
                    sampler_->sampleUniform(newState->state(), costSampled_, costReqd);

                    // If the state is collision free, add it to the list of free states
                    ++numStateCollisionChecks_;
                    if (Planner::si_->isValid(newState->stateConst()) == true)
                    {
                        // Add the new state as a sample
                        this->addSample(newState);

                        // Update the number of uniformly distributed states
                        ++numUniformStates_;

                        // Update the number of sample
                        ++numSamples_;
                    }
                    // No else
                }

                // Mark that we've sampled all cost spaces (This is in preparation for JIT sampling)
                costSampled_ = costReqd;
            }
            // No else, the samples are up to date
        }

        bool BITstar::prune()
        {
            // Variable:
            // Whether or not we pruned, start as unpruned
            bool vertexPruned = false;

            // Test if we should we do a little tidying up:
            // Is pruning enabled? Do we have a solution? Has the solution changed enough?
            if ((usePruning_ == true) && (hasExactSolution_ == true) &&
                (std::abs(costHelpPtr_->fractionalChange(bestCost_, prunedCost_)) > pruneFraction_))
            {
                // Variables:
                // The current measure of the problem space:
                double informedMeasure = sampler_->getInformedMeasure(bestCost_);

                // Is there good reason to prune? I.e., is the informed subset measurably less than the total problem
                // domain? If an informed measure is not available, we'll assume yes:
                if ((sampler_->hasInformedMeasure() == true && informedMeasure < si_->getSpaceMeasure()) ||
                    (sampler_->hasInformedMeasure() == false))
                {
                    // Variable:
                    // The number of starts/goals pruned
                    std::pair<unsigned int, unsigned int> numStartsGoalsPruned;
                    // The number of samples pruned
                    unsigned int numSamplesPruned;
                    // The number of vertices disconnected and pruned
                    std::pair<unsigned int, unsigned int> numPruned;

                    OMPL_INFORM("%s: Pruning the planning problem from a solution of %.4f to %.4f, resulting in a "
                                "change of problem size from %.4f to %.4f.",
                                Planner::getName().c_str(), prunedCost_.value(), bestCost_.value(), prunedMeasure_,
                                informedMeasure);

                    // Increment the pruning counter:
                    ++numPrunings_;

                    // First, prune the starts/goals:
                    numStartsGoalsPruned = this->pruneStartsGoals();

                    // Prune the samples
                    numSamplesPruned = this->pruneSamples();

                    // Prune the graph. This can be done extra efficiently by using some info in the integrated queue.
                    // This requires access to the nearest neighbour structures so vertices can be moved to free
                    // states.
                    numPruned = intQueue_->prune(curGoalVertex_, vertexNN_, freeStateNN_, &recycledSamples_);

                    // The number of vertices and samples pruned are incrementally updated.
                    numVerticesDisconnected_ = numVerticesDisconnected_ + numPruned.first;
                    numFreeStatesPruned_ = numFreeStatesPruned_ + numPruned.second;

                    // Store the cost at which we pruned:
                    prunedCost_ = bestCost_;

                    // And the measure:
                    prunedMeasure_ = informedMeasure;

                    // Check if any states have actually been pruned
                    vertexPruned = (numPruned.second > 0u);

                    OMPL_INFORM("%s: Pruning removed %d start and %d goal states, disconnected a total of %d vertices "
                                "from the tree, and completely removed %d samples.",
                                Planner::getName().c_str(), numStartsGoalsPruned.first, numStartsGoalsPruned.second,
                                numPruned.first + numStartsGoalsPruned.first,
                                numPruned.second + numStartsGoalsPruned.first + numStartsGoalsPruned.second +
                                    numSamplesPruned);
                }
                // No else, it's not worth the work to prune...
            }
            // No else, why was I called?

            return vertexPruned;
        }

        bool BITstar::resort()
        {
            // Variable:
            // The number of vertices and samples pruned
            std::pair<unsigned int, unsigned int> numPruned;

            // During resorting we can be lazy and skip resorting vertices that will just be pruned later. So, are we
            // using pruning?
            if (usePruning_ == true)
            {
                // We are, give the queue access to the nearest neighbour structures so vertices can be pruned instead
                // of resorted.
                // The number of vertices pruned is also incrementally updated.
                numPruned = intQueue_->resort(vertexNN_, freeStateNN_, &recycledSamples_);
            }
            else
            {
                // We are not, give it empty NN structs
                numPruned = intQueue_->resort(VertexPtrNNPtr(), VertexPtrNNPtr(), nullptr);
            }

            // The number of vertices and samples pruned are incrementally updated.
            numVerticesDisconnected_ = numVerticesDisconnected_ + numPruned.first;
            numFreeStatesPruned_ = numFreeStatesPruned_ + numPruned.second;

            return (numPruned.second > 0u);
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
            if (hasExactSolution_ == false && findApprox_ == true)
            {
                soln.setApproximate(approxDiff_);
            }

            // Mark whether the solution met the optimization objective:
            soln.optimized_ = opt_->isSatisfied(bestCost_);

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
            if (hasExactSolution_ == true)
            {
                // Start at vertex in the goal
                curVertex = curGoalVertex_;
            }
            else if (hasExactSolution_ == false && findApprox_ == true)
            {
                // Start at the vertex closest to the goal
                curVertex = approxGoalVertex_;
            }
            else
            {
                throw ompl::Exception("bestPathFromGoalToStart called without an exact or approximate solution.");
            }

            // Insert the goal into the path
            reversePath.push_back(curVertex->stateConst());

            // Then, use the vertex pointer like an iterator. Starting at the goal, we iterate up the chain pushing the
            // *parent* of the iterator into the vector until the vertex has no parent.
            // This will allows us to add the start (as the parent of the first child) and then stop when we get to the
            // start itself, avoiding trying to find its nonexistent child
            for (/*Already allocated*/; curVertex->isRoot() == false; curVertex = curVertex->getParentConst())
            {
                // Check the case where the chain ends incorrectly. This is unnecessary but sure helpful in debugging:
                if (curVertex->hasParent() == false)
                {
                    throw ompl::Exception("The path to the goal does not originate at a start state. Something went "
                                          "wrong.");
                }

                // Push back the parent into the vector as a state pointer:
                reversePath.push_back(curVertex->getParentConst()->stateConst());
            }
            return reversePath;
        }

        void BITstar::updateStartAndGoalStates(const base::PlannerTerminationCondition &ptc)
        {
            // Variable
            // Whether we've added a start or goal:
            bool addedGoal = false;
            bool addedStart = false;
            // Whether we have to rebuid the queue, i.e.. whether we've called updateStartAndGoalStates before
            bool rebuildQueue = false;

            // Add the new starts and goals to the lists of said vertices.
            // Do goals first, as they are only added as samples.
            // We do this as nested conditions so we always call nextGoal(ptc) at least once (regardless of whether
            // there are moreGoalStates or not)
            // in case we have been given a non trivial PTC that wants us to wait, but do *not* call it again if there
            // are no more goals
            //(as in the nontrivial PTC case, doing so would cause us to wait out the ptc and never try to solve
            // anything)
            do
            {
                // Variable
                // A new goal pointer, if there are none, it will be a nullptr.
                // We will wait for the duration of PTC for a new goal to appear.
                const ompl::base::State *newGoal = Planner::pis_.nextGoal(ptc);

                // Check if it's valid
                if (static_cast<bool>(newGoal) == true)
                {
                    // It is valid and we are adding a goal, we will need to rebuild the queue if any starts have
                    // previously been added as their (and any descendents') heuristic cost-to-go may change:
                    rebuildQueue = (startVerticesPtr_->size() > 0u);

                    // Allocate the vertex pointer
                    goalVerticesPtr_->push_back(std::make_shared<Vertex>(Planner::si_, opt_));

                    // Copy the value into the state
                    Planner::si_->copyState(goalVerticesPtr_->back()->state(), newGoal);

                    // And add this goal to the set of samples:
                    this->addSample(goalVerticesPtr_->back());

                    // Mark that we've added:
                    addedGoal = true;
                }
                // No else, there was no goal.
            } while (Planner::pis_.haveMoreGoalStates() == true);

            // And then do the for starts. We do this last as the starts are added to the queue, which uses a cost-to-go
            // heuristic in it's ordering, and for that we want all the goals updated.
            // As there is no way to wait for new *start* states, this loop can be cleaner
            // There is no need to rebuild the queue when we add start vertices, as the queue is ordered on current
            // cost-to-come, and adding a start doesn't change that.
            while (Planner::pis_.haveMoreStartStates() == true)
            {
                // Variable
                // A new start pointer
                const ompl::base::State *newStart = Planner::pis_.nextStart();

                // Allocate the vertex pointer:
                startVerticesPtr_->push_back(std::make_shared<Vertex>(Planner::si_, opt_, true));

                // Copy the value into the state:
                Planner::si_->copyState(startVerticesPtr_->back()->state(), newStart);

                // Add this start to the queue. It is not a sample, so skip that step:
                this->addVertex(startVerticesPtr_->back(), false);

                // Mark that we've added:
                addedStart = true;
            }

            // Now, if we added a new start and have previously pruned goals, we may want to readd them.
            if (addedStart == true && prunedGoalVertices_.empty() == false)
            {
                // Variable
                // An iterator to the list of pruned goals
                auto prunedGoalIter = prunedGoalVertices_.begin();

                // Consider each one
                while (prunedGoalIter != prunedGoalVertices_.end())
                {
                    // Mark as unpruned
                    (*prunedGoalIter)->markUnpruned();

                    // Check if it should be readded (i.e., would it be pruned *now*?)
                    if (intQueue_->vertexPruneCondition(*prunedGoalIter) == true)
                    {
                        // It would be pruned, so remark as pruned
                        (*prunedGoalIter)->markPruned();

                        // and move onto the next:
                        ++prunedGoalIter;
                    }
                    else
                    {
                        // It would not be pruned now, so readd it!
                        // Add back to the list:
                        goalVerticesPtr_->push_back(*prunedGoalIter);

                        // Add as a sample
                        this->addSample(*prunedGoalIter);

                        // Mark what we've added:
                        addedGoal = true;

                        // Remove the start from the list, this returns the next iterator
                        prunedGoalIter = prunedGoalVertices_.erase(prunedGoalIter);

                        // Just like the other new goals, we will need to rebuild the queue if any starts have
                        // previously been added. Which was a condition to be here in the first place
                        rebuildQueue = true;
                    }
                }
            }

            // Now, if we added a goal and have previously pruned starts, we will have to do the same on those
            if (addedGoal == true && prunedStartVertices_.empty() == false)
            {
                // Variable
                // An iterator to the list of pruned starts
                auto prunedStartIter = prunedStartVertices_.begin();

                // Consider each one
                while (prunedStartIter != prunedStartVertices_.end())
                {
                    // Mark as unpruned
                    (*prunedStartIter)->markUnpruned();

                    // Check if it should be readded (i.e., would it be pruned *now*?)
                    if (intQueue_->vertexPruneCondition(*prunedStartIter) == true)
                    {
                        // It would be pruned, so remark as pruned
                        (*prunedStartIter)->markPruned();

                        // and move onto the next:
                        ++prunedStartIter;
                    }
                    else
                    {
                        // It would not be pruned, readd it!
                        // Add it back to the list
                        startVerticesPtr_->push_back(*prunedStartIter);

                        // Add to the queue as a vertex. It is not a sample, so skip that step:
                        this->addVertex(*prunedStartIter, false);

                        // Mark what we've added:
                        addedStart = true;

                        // Remove the start from the list, this returns the next iterator
                        prunedStartIter = prunedStartVertices_.erase(prunedStartIter);
                    }
                }
            }

            // If we've added a state, we have some updating to do.
            if (addedGoal == true || addedStart == true)
            {
                // Update the minimum cost
                for (const auto &startVertex : *startVerticesPtr_)
                {
                    // Take the better of the min cost so far and the cost-to-go from this start
                    minCost_ = costHelpPtr_->betterCost(minCost_, costHelpPtr_->costToGoHeuristic(startVertex));
                }

                // If we have at least one start and goal, allocate a sampler
                if (startVerticesPtr_->size() > 0u && goalVerticesPtr_->size() > 0u)
                {
                    // There is a start and goal, allocate
                    sampler_ =
                        opt_->allocInformedStateSampler(Planner::pdef_, std::numeric_limits<unsigned int>::max());
                }
                // No else, this will get allocated when we get the updated start/goal.

                // Was there an existing queue that needs to be rebuilt?
                if (rebuildQueue == true)
                {
                    // There was, inform
                    OMPL_INFORM("%s: Updating starts/goals and rebuilding the queue.", Planner::getName().c_str());

                    // Flag the queue as unsorted downstream from every existing start.
                    for (const auto &startVertex : *startVerticesPtr_)
                    {
                        intQueue_->markVertexUnsorted(startVertex);
                    }

                    // Resort the queue.
                    this->resort();
                }
                // No else

                // Iterate through the existing vertices and find the current best approximate solution (if enabled)
                this->findNewApproxSoln();
            }
            // No else, why were we called?

            // Make sure that if we have a goal, we also have a start, since there's no way to wait for more *starts*
            if (goalVerticesPtr_->empty() == false && startVerticesPtr_->empty() == true)
            {
                OMPL_WARN("%s, The problem has a goal but not a start. As PlannerInputStates provides no method to "
                          "wait for a _start_ state, this will likely be problematic.",
                          Planner::getName().c_str());
            }
            // No else
        }

        std::pair<unsigned int, unsigned int> BITstar::pruneStartsGoals()
        {
            // Variable:
            // The number of starts/goals pruned
            std::pair<unsigned int, unsigned int> numPruned(0u, 0u);

            // Are there superfluous starts to prune?
            if (startVerticesPtr_->size() > 1u)
            {
                // Yes, Iterate through the list

                // Variable
                // The iterator to the start:
                auto startIter = startVerticesPtr_->begin();

                // Run until at the end:
                while (startIter != startVerticesPtr_->end())
                {
                    // Check if this start has met the criteria to be pruned
                    if (intQueue_->vertexPruneCondition(*startIter) == true)
                    {
                        // It has, update counters. By definition of the heuristics, start vertices are either in the
                        // tree or are deleted. Since we're pruning this one, that means we're going all the way to
                        // remove it:
                        numPruned.first = numPruned.first + 1u;
                        ++numVerticesDisconnected_;
                        ++numFreeStatesPruned_;

                        // Remove the start vertex completely from the queue, they don't have parents
                        intQueue_->eraseVertex(*startIter, false, vertexNN_, freeStateNN_, &recycledSamples_);

                        // Store the start vertex in the pruned list, in case it later needs to be readded:
                        prunedStartVertices_.push_back(*startIter);

                        // Remove from the list, this returns the next iterator
                        startIter = startVerticesPtr_->erase(startIter);
                    }
                    else
                    {
                        // Still valid, move to the next one:
                        ++startIter;
                    }
                }
            }
            // No else, can't prune 1 start.

            // Are there superfluous goals to prune?
            if (goalVerticesPtr_->size() > 1u)
            {
                // Yes, Iterate through the list

                // Variable
                // The iterator to the start:
                auto goalIter = goalVerticesPtr_->begin();

                // Run until at the end:
                while (goalIter != goalVerticesPtr_->end())
                {
                    // Check if this start has met the criteria to be pruned
                    if (intQueue_->vertexPruneCondition(*goalIter) == true)
                    {
                        // Update the counter of how many goals pruned
                        numPruned.second = numPruned.second + 1u;

                        // It has, remove the goal vertex completely
                        // Check if this vertex is in the tree
                        if ((*goalIter)->isInTree() == true)
                        {
                            // It is, increment the counters
                            ++numVerticesDisconnected_;
                            ++numFreeStatesPruned_;

                            // And erase it from the queue:
                            intQueue_->eraseVertex(*goalIter, (*goalIter)->hasParent(), vertexNN_, freeStateNN_,
                                                   &recycledSamples_);

                            // Store the start vertex in the pruned list, in case it later needs to be readded:
                            prunedGoalVertices_.push_back(*goalIter);

                            // Remove from the list, this returns the next iterator
                            goalIter = goalVerticesPtr_->erase(goalIter);
                        }
                        else
                        {
                            // It is not, so we just delete it like a sample
                            this->dropSample(*goalIter);

                            // Remove from the list, this returns the next iterator
                            goalIter = goalVerticesPtr_->erase(goalIter);
                        }
                    }
                    else
                    {
                        // The goal is still valid, get the next
                        ++goalIter;
                    }
                }
            }
            // No else, can't prune 1 goal.

            return numPruned;
        }

        unsigned int BITstar::pruneSamples()
        {
            // Variable:
            // The number of samples pruned in this pass:
            unsigned int numPruned = 0u;

            // Are we dropping samples anytime we prune?
            if (dropSamplesOnPrune_ == true)
            {
                // We are, store the number pruned
                numPruned = freeStateNN_->size();

                // and the number of uniform samples
                numUniformStates_ = 0u;

                // Then remove all of the samples
                freeStateNN_->clear();
            }
            else
            {
                // Variable:
                // The list of samples:
                VertexPtrVector samples;

                // Get the list of samples
                freeStateNN_->list(samples);

                // Iterate through the list and remove any samples that have a heuristic larger than the bestCost_
                for (const auto &freeSample : samples)
                {
                    // Check if this state should be pruned:
                    if (intQueue_->samplePruneCondition(freeSample) == true)
                    {
                        // Yes, remove it
                        this->dropSample(freeSample);

                        // and increment the counter
                        ++numPruned;
                    }
                    // No else, keep.
                }
            }

            // Update the global pruned counter
            numFreeStatesPruned_ = numFreeStatesPruned_ + numPruned;

            return numPruned;
        }

        bool BITstar::checkEdge(const VertexConstPtrPair &edge)
        {
            ++numEdgeCollisionChecks_;
            return Planner::si_->checkMotion(edge.first->stateConst(), edge.second->stateConst());
        }

        void BITstar::dropSample(const VertexPtr &oldSample)
        {
            // Create a copy of the vertex pointer so we don't delete it out from under ourselves.
            VertexPtr sampleToDelete(oldSample);

            // Remove from the list of samples
            freeStateNN_->remove(sampleToDelete);

            // Mark the sample as pruned
            sampleToDelete->markPruned();
        }

        void BITstar::addEdge(const VertexPtrPair &newEdge, const ompl::base::Cost &edgeCost,
                              const bool &removeFromFree, const bool &updateDescendants)
        {
            if (newEdge.first->isInTree() == false)
            {
                throw ompl::Exception("Adding an edge from a vertex not connected to the graph");
            }

            // This should be a debug-level-only assert some day:
            /*
            if (costHelpPtr_->isCostEquivalentTo(this->trueEdgeCost(newEdge), edgeCost) == false)
            {
                throw ompl::Exception("You have passed the wrong edge cost to addEdge.");
            }
            */

            // Variables
            // The edge is a rewiring if it is current in the tree:
            bool isRewiring = newEdge.second->hasParent();

            // Perform a rewiring?
            if (isRewiring == true)
            {
                // Replace the edge
                this->replaceParent(newEdge, edgeCost, updateDescendants);
            }
            else
            {
                // If not, we just add the vertex, first mark the target vertex as no longer new and unexpanded:
                newEdge.second->markUnexpandedToSamples();
                newEdge.second->markUnexpandedToVertices();

                // Then add a child to the parent, not updating costs:
                newEdge.first->addChild(newEdge.second, false);

                // Add a parent to the child, updating descendant costs if requested:
                newEdge.second->addParent(newEdge.first, edgeCost, updateDescendants);

                // Then add to the queues as necessary
                this->addVertex(newEdge.second, removeFromFree);
            }

            // If the path to the goal has changed, we may need to update the cached info about the solution cost or
            // solution length:
            this->updateGoalVertex();

            // If we added a new vertex it may improve an approximate solution (if we are searching for one).
            if (isRewiring == false)
            {
                // Check for an updated approximate solution. Does no work if an exact solution exists or we're not
                // considering approximate solutions.
                this->updateApproxSoln(newEdge.second);
            }
        }

        void BITstar::replaceParent(const VertexPtrPair &newEdge, const ompl::base::Cost &edgeCost,
                                    const bool &updateDescendants)
        {
            if (newEdge.second->getParent()->getId() == newEdge.first->getId())
            {
                throw ompl::Exception("The new and old parents of the given rewiring are the same.");
            }

            // This would be a good debug-level-only assert
            if (costHelpPtr_->isCostBetterThan(newEdge.second->getCost(),
                                               costHelpPtr_->combineCosts(newEdge.first->getCost(), edgeCost)) == true)
            {
                throw ompl::Exception("The new edge will increase the cost-to-come of the vertex!");
            }

            // Increment our counter:
            ++numRewirings_;

            // Remove the child from the parent, not updating costs
            newEdge.second->getParent()->removeChild(newEdge.second, false);

            // Remove the parent from the child, not updating costs
            newEdge.second->removeParent(false);

            // Add the child to the parent, not updating costs
            newEdge.first->addChild(newEdge.second, false);

            // Add the parent to the child. This updates the cost of the child as well as all it's descendents (if
            // requested).
            newEdge.second->addParent(newEdge.first, edgeCost, updateDescendants);

            // Mark the queues as unsorted below this child
            intQueue_->markVertexUnsorted(newEdge.second);
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

            // Iterate through the list of goals, and see if the solution has changed
            for (const auto &goalVertex : *goalVerticesPtr_)
            {
                // First, is this goal even in the tree?
                if (goalVertex->isInTree() == true)
                {
                    // Next, is there currently a solution?
                    if (static_cast<bool>(newBestGoal) == true)
                    {
                        // There is already a solution, is it to this goal?
                        if (goalVertex->getId() == newBestGoal->getId())
                        {
                            // Ah-ha, We meet again! Are we doing any better? We check the length as sometimes the path
                            // length changes with minimal change in cost.
                            if (costHelpPtr_->isCostEquivalentTo(goalVertex->getCost(), newCost) == false ||
                                (goalVertex->getDepth() + 1u) != bestLength_)
                            {
                                // The path to the current best goal has changed, so we need to update it.
                                goalUpdated = true;
                                newBestGoal = goalVertex;
                                newCost = newBestGoal->getCost();
                            }
                            // No else, no change
                        }
                        else
                        {
                            // It is not to this goal, we have a second solution! What an easy problem... but is it
                            // better?
                            if (costHelpPtr_->isCostBetterThan(goalVertex->getCost(), newCost) == true)
                            {
                                // It is! Save this as a better goal:
                                goalUpdated = true;
                                newBestGoal = goalVertex;
                                newCost = newBestGoal->getCost();
                            }
                            // No else, not a better solution
                        }
                    }
                    else
                    {
                        // There isn't a preexisting solution, that means that any goal is an update:
                        goalUpdated = true;
                        newBestGoal = goalVertex;
                        newCost = newBestGoal->getCost();
                    }
                }
                // No else, can't be a better solution if it's not in the spanning tree, can it?
            }

            // Did we update the goal?
            if (goalUpdated == true)
            {
                // We have a better solution!
                if (hasExactSolution_ == false)
                {
                    approxDiff_ = std::numeric_limits<double>::infinity();
                    approxGoalVertex_.reset();
                }

                // Mark that we have a solution
                hasExactSolution_ = true;
                intQueue_->hasSolution();

                // Store the current goal
                curGoalVertex_ = newBestGoal;

                // Update the best cost:
                bestCost_ = newCost;

                // and best length
                bestLength_ = curGoalVertex_->getDepth() + 1u;

                // Update the queue threshold:
                intQueue_->setThreshold(bestCost_);

                // Stop the solution loop if enabled:
                stopLoop_ = stopOnSolnChange_;

                // Brag:
                this->goalMessage();

                // If enabled, pass the intermediate solution back through the call back:
                if (static_cast<bool>(Planner::pdef_->getIntermediateSolutionCallback()) == true)
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

        void BITstar::findNewApproxSoln()
        {
            if (hasExactSolution_ == false && findApprox_ == true && static_cast<bool>(vertexNN_) == true)
            {
                // Variable
                // The vertices in the graph
                VertexPtrVector vertices;

                // Get the list of vertices
                vertexNN_->list(vertices);

                // Iterate through them and call updateApproxSoln
                for (const auto &vertex : vertices)
                {
                    this->updateApproxSoln(vertex);
                }
            }
            // No else, I do nothing.
        }

        void BITstar::updateApproxSoln(const VertexConstPtr &newVertex)
        {
            // Only do work if an exact solution does not exist
            if (hasExactSolution_ == false && findApprox_ == true)
            {
                // Variable
                // The distance from this vertex to the goal:
                double distFromGoal;

                // Find the shortest distance between the given vertex and a goal
                Planner::pdef_->getGoal()->isSatisfied(newVertex->stateConst(), &distFromGoal);

                // Compare to the current best approximate solution
                if (distFromGoal < approxDiff_)
                {
                    // Better, update the approximate solution
                    approxGoalVertex_ = newVertex;
                    approxDiff_ = distFromGoal;
                }
                // No else, don't update if worse
            }
            // No else, I do nothing
        }

        void BITstar::addSample(const VertexPtr &newSample)
        {
            // Mark as new
            newSample->markNew();

            // Add to the list of new samples
            newSamples_.push_back(newSample);

            // Add to the NN structure:
            freeStateNN_->add(newSample);
        }

        void BITstar::addVertex(const VertexPtr &newVertex, const bool &removeFromFree)
        {
            // Make sure it's connected first, so that the queue gets updated properly. This is a day of debugging I'll
            // never get back
            if (newVertex->isInTree() == false)
            {
                throw ompl::Exception("Vertices must be connected to the graph before adding");
            }

            // Remove the vertex from the list of samples (if it even existed)
            if (removeFromFree == true)
            {
                freeStateNN_->remove(newVertex);
            }
            // No else

            // Add to the NN structure:
            vertexNN_->add(newVertex);

            // Add to the queue:
            intQueue_->insertVertex(newVertex);

            // Increment the number of vertices added:
            ++numVertices_;
        }

        unsigned int BITstar::nearestSamples(const VertexPtr &vertex, VertexPtrVector *neighbourSamples)
        {
            // Make sure sampling has happened first:
            this->updateSamples(vertex);

            // Increment our counter:
            ++numNearestNeighbours_;

            if (useKNearest_ == true)
            {
                freeStateNN_->nearestK(vertex, k_, *neighbourSamples);
                return k_;
            }
            else
            {
                freeStateNN_->nearestR(vertex, r_, *neighbourSamples);
                return 0u;
            }
        }

        unsigned int BITstar::nearestVertices(const VertexPtr &vertex, VertexPtrVector *neighbourVertices)
        {
            // Increment our counter:
            ++numNearestNeighbours_;

            if (useKNearest_ == true)
            {
                vertexNN_->nearestK(vertex, k_, *neighbourVertices);
                return k_;
            }
            else
            {
                vertexNN_->nearestR(vertex, r_, *neighbourVertices);
                return 0u;
            }
        }

        double BITstar::nnDistance(const VertexConstPtr &a, const VertexConstPtr &b) const
        {
            // Using RRTstar as an example, this order gives us the distance FROM the queried state TO the other
            // neighbours in the structure.
            // The distance function between two states
            if (!a->stateConst())
            {
                throw ompl::Exception("a->state is unallocated");
            }
            if (!b->stateConst())
            {
                throw ompl::Exception("b->state is unallocated");
            }
            return Planner::si_->distance(b->stateConst(), a->stateConst());
        }

        ompl::base::Cost BITstar::trueEdgeCost(const VertexConstPtrPair &edgePair) const
        {
            return costHelpPtr_->motionCost(edgePair.first->stateConst(), edgePair.second->stateConst());
        }

        ompl::base::Cost BITstar::neighbourhoodCost(const VertexConstPtr &vertex) const
        {
            // Even though the problem domain is defined by prunedCost_ (the cost the last time we pruned), there is no
            // point generating samples outside bestCost_ (which may be less).
            if (useJustInTimeSampling_ == true)
            {
                return costHelpPtr_->betterCost(
                    bestCost_, costHelpPtr_->combineCosts(costHelpPtr_->lowerBoundHeuristicVertex(vertex),
                                                          ompl::base::Cost(2.0 * r_)));
            }
            else
            {
                return bestCost_;
            }
        }

        void BITstar::initializeNearestTerms()
        {
            // Calculate the k-nearest constant
            k_rgg_ = this->minimumRggK();

            // Update the actual terms
            this->updateNearestTerms();
        }

        void BITstar::updateNearestTerms()
        {
            // First try all start-goal pairs (which will be when it is the 0-th batch)
            if (numBatches_ == 0u)
            {
                k_ = startVerticesPtr_->size() + goalVerticesPtr_->size();
                r_ = std::numeric_limits<double>::infinity();
            }
            else
            {
                // Variables:
                // The number of uniformly distributed states:
                unsigned int N;

                // Calculate the number of N, are we dropping samples?
                if (dropSamplesOnPrune_ == true)
                {
                    // We are, so we've been tracking the number of uniform states, just use that
                    N = numUniformStates_;
                }
                else
                {
                    // We are not, so the all vertices and samples are uniform, use that
                    N = vertexNN_->size() + freeStateNN_->size();
                }

                // If this is the first batch, we will have calculated the connection limits from only the starts and
                // goals, yet have an RGG with samplesPerBatch_. That will be a complex graph.
                // In this case, let us calculate the connection limits considering the samples about to be generated.
                // Doing so is equivalent to setting an upper-bound on the radius, which RRT* does with it's
                // min(maxEdgeLength,RGG-radius).
                if (numBatches_ == 1u)
                {
                    N = N + samplesPerBatch_;
                }

                // Now update the appropriate term
                if (useKNearest_ == true)
                {
                    k_ = this->calculateK(N);
                }
                else
                {
                    r_ = this->calculateR(N);
                }
            }
        }

        double BITstar::calculateR(unsigned int N) const
        {
            // Variables
            // The dimension cast as a double for readibility;
            double dimDbl = static_cast<double>(Planner::si_->getStateDimension());
            // The size of the graph
            double cardDbl = static_cast<double>(N);

            // Calculate the term and return
            return this->minimumRggR() * std::pow(std::log(cardDbl) / cardDbl, 1 / dimDbl);
        }

        unsigned int BITstar::calculateK(unsigned int N) const
        {
            // Calculate the term and return
            return std::ceil(k_rgg_ * std::log(static_cast<double>(N)));
        }

        double BITstar::minimumRggR() const
        {
            // Variables
            // The dimension cast as a double for readibility;
            double dimDbl = static_cast<double>(Planner::si_->getStateDimension());

            // Calculate the term and return
            return rewireFactor_ * 2.0 *
                   std::pow((1.0 + 1.0 / dimDbl) *
                                (prunedMeasure_ / unitNBallMeasure(Planner::si_->getStateDimension())),
                            1.0 / dimDbl);  // RRG radius (biggest for unit-volume problem)
            // return rewireFactor_*std::pow( 2.0*(1.0 + 1.0/dimDbl)*(
            // prunedMeasure_/unitNBallMeasure(Planner::si_->getStateDimension()) ), 1.0/dimDbl ); //RRT* radius
            // (smaller for unit-volume problem)
            // return rewireFactor_*2.0*std::pow( (1.0/dimDbl)*(
            // prunedMeasure_/unitNBallMeasure(Planner::si_->getStateDimension()) ), 1.0/dimDbl ); //FMT* radius
            // (smallest for R2, equiv to RRT* for R3 and then middle for higher d. All unit-volume)
        }

        double BITstar::minimumRggK() const
        {
            // Variables
            // The dimension cast as a double for readibility;
            double dimDbl = static_cast<double>(Planner::si_->getStateDimension());

            // Calculate the term and return
            return rewireFactor_ * (boost::math::constants::e<double>() +
                                    (boost::math::constants::e<double>() / dimDbl));  // RRG k-nearest
        }

        void BITstar::goalMessage() const
        {
            OMPL_INFORM("%s (%u iters): Found a solution of cost %.4f (%u vertices) from %u samples by processing %u "
                        "edges (%u collision checked) to create %u vertices and perform %u rewirings. The graph "
                        "currently has %u vertices.",
                        Planner::getName().c_str(), numIterations_, bestCost_.value(), bestLength_, numSamples_,
                        numEdgesProcessed_, numEdgeCollisionChecks_, numVertices_, numRewirings_, vertexNN_->size());
        }

        void BITstar::endSuccessMessage() const
        {
            OMPL_INFORM("%s: Finished with a solution of cost %.4f (%u vertices) found from %u samples by processing "
                        "%u edges (%u collision checked) to create %u vertices and perform %u rewirings. The final "
                        "graph has %u vertices.",
                        Planner::getName().c_str(), bestCost_.value(), bestLength_, numSamples_, numEdgesProcessed_,
                        numEdgeCollisionChecks_, numVertices_, numRewirings_, vertexNN_->size());
        }

        void BITstar::endFailureMessage() const
        {
            if (findApprox_ == true)
            {
                OMPL_INFORM("%s (%u iters): Did not find an exact solution from %u samples after processing %u edges "
                            "(%u "
                            "collision checked) to create %u vertices and perform %u rewirings. The final graph has %u "
                            "vertices. The best approximate solution was %.4f from the goal and has a cost of %.4f.",
                            Planner::getName().c_str(), numIterations_, numSamples_, numEdgesProcessed_,
                            numEdgeCollisionChecks_, numVertices_, numRewirings_, vertexNN_->size(), approxDiff_,
                            approxGoalVertex_->getCost().value());
            }
            else
            {
                OMPL_INFORM("%s (%u iters): Did not find an exact solution from %u samples after processing %u edges "
                            "(%u "
                            "collision checked) to create %u vertices and perform %u rewirings. The final graph has %u "
                            "vertices.",
                            Planner::getName().c_str(), numIterations_, numSamples_, numEdgesProcessed_,
                            numEdgeCollisionChecks_, numVertices_, numRewirings_, vertexNN_->size());
            }
        }

        void BITstar::statusMessage(const ompl::msg::LogLevel &msgLevel, const std::string &status) const
        {
            // Check if we need to create the message
            if (msgLevel >= ompl::msg::getLogLevel())
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
                outputStream << ", g: " << std::setw(5) << std::setfill(' ') << vertexNN_->size();
                // The number of free states
                outputStream << ", f: " << std::setw(5) << std::setfill(' ') << freeStateNN_->size();
                // The number edges in the queue:
                outputStream << ", q: " << std::setw(5) << std::setfill(' ') << intQueue_->numEdges();
                // The total number of edges taken out of the queue:
                outputStream << ", t: " << std::setw(5) << std::setfill(' ') << numEdgesProcessed_;
                // The number of samples generated
                outputStream << ", s: " << std::setw(5) << std::setfill(' ') << numSamples_;
                // The number of vertices ever added to the graph:
                outputStream << ", v: " << std::setw(5) << std::setfill(' ') << numVertices_;
                // The number of prunings:
                outputStream << ", p: " << std::setw(5) << std::setfill(' ') << numPrunings_;
                // The number of rewirings:
                outputStream << ", r: " << std::setw(5) << std::setfill(' ') << numRewirings_;
                // The number of nearest-neighbour calls
                outputStream << ", n: " << std::setw(5) << std::setfill(' ') << numNearestNeighbours_;
                // The number of state collision checks:
                outputStream << ", c(s): " << std::setw(5) << std::setfill(' ') << numStateCollisionChecks_;
                // The number of edge collision checks:
                outputStream << ", c(e): " << std::setw(5) << std::setfill(' ') << numEdgeCollisionChecks_;
                outputStream << "):    ";
                // The message:
                outputStream << status;

                if (msgLevel == ompl::msg::LOG_DEBUG)
                {
                    OMPL_DEBUG("%s", outputStream.str().c_str());
                }
                else if (msgLevel == ompl::msg::LOG_INFO)
                {
                    OMPL_INFORM("%s", outputStream.str().c_str());
                }
                else if (msgLevel == ompl::msg::LOG_WARN)
                {
                    OMPL_WARN("%s", outputStream.str().c_str());
                }
                else if (msgLevel == ompl::msg::LOG_ERROR)
                {
                    OMPL_ERROR("%s", outputStream.str().c_str());
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
        void BITstar::setRewireFactor(double rewireFactor)
        {
            rewireFactor_ = rewireFactor;

            // Check if there's things to update
            if (this->isSetup() == true)
            {
                // Reinitialize the terms:
                this->initializeNearestTerms();
            }
        }

        double BITstar::getRewireFactor() const
        {
            return rewireFactor_;
        }

        void BITstar::setSamplesPerBatch(unsigned int n)
        {
            samplesPerBatch_ = n;
        }

        unsigned int BITstar::getSamplesPerBatch() const
        {
            return samplesPerBatch_;
        }

        void BITstar::setKNearest(bool useKNearest)
        {
            // Check if the flag has changed
            if (useKNearest != useKNearest_)
            {
                // If the planner is default named, we change it:
                if (useKNearest_ == true && Planner::getName() == "kBITstar")
                {
                    // It's current the default k-nearest BIT* name, and we're toggling, so set to the default r-disc
                    Planner::setName("BITstar");
                }
                else if (useKNearest_ == false && Planner::getName() == "BITstar")
                {
                    // It's current the default r-disc BIT* name, and we're toggling, so set to the default k-nearest
                    Planner::setName("kBITstar");
                }
                // It's not default named, don't change it

                // Set the k-nearest flag
                useKNearest_ = useKNearest;

                if (useKNearest_ == true)
                {
                    // Check that we're not doing JIT
                    if (useJustInTimeSampling_ == true)
                    {
                        throw ompl::Exception("JIT sampling does not work with the k-nearest variant of BIT*.");
                    }
                }

                // Check if there's things to update
                if (this->isSetup() == true)
                {
                    // Reinitialize the terms:
                    this->initializeNearestTerms();
                }
            }
            // No else, it didn't change.
        }

        bool BITstar::getKNearest() const
        {
            return useKNearest_;
        }

        void BITstar::setStrictQueueOrdering(bool beStrict)
        {
            useStrictQueueOrdering_ = beStrict;
        }

        bool BITstar::getStrictQueueOrdering() const
        {
            return useStrictQueueOrdering_;
        }

        void BITstar::setConsiderApproximateSolutions(bool findApproximate)
        {
            // Check if the flag has changed
            if (findApproximate != findApprox_)
            {
                // Store the flag and mark the Planner spec:
                findApprox_ = findApproximate;
                Planner::specs_.approximateSolutions = findApprox_;

                // Did we turn approximate solutions on or off?
                if (findApproximate == true)
                {
                    // We turned it on, find an approximate solution:
                    this->findNewApproxSoln();
                }
                else
                {
                    // We turned it off, clear the approximate solution variables:
                    approxDiff_ = std::numeric_limits<double>::infinity();
                    approxGoalVertex_.reset();
                }
            }
        }

        bool BITstar::getConsiderApproximateSolutions() const
        {
            return findApprox_;
        }

        void BITstar::setPruning(bool prune)
        {
            if (prune == false)
            {
                OMPL_WARN("%s: Turning pruning off has never really been tested.", Planner::getName().c_str());
            }

            usePruning_ = prune;
        }

        bool BITstar::getPruning() const
        {
            return usePruning_;
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

        void BITstar::setDelayRewiringUntilInitialSolution(bool delayRewiring)
        {
            delayRewiring_ = delayRewiring;

            // Configure queue if constructed:
            if (intQueue_)
            {
                intQueue_->setDelayedRewiring(delayRewiring_);
            }
        }

        bool BITstar::getDelayRewiringUntilInitialSolution() const
        {
            return delayRewiring_;
        }

        void BITstar::setJustInTimeSampling(bool useJit)
        {
            if (useJit == true)
            {
                OMPL_WARN("%s: Just-in-time sampling is experimental and currently only implemented for problems "
                          "seeking to minimize path-length.",
                          Planner::getName().c_str());

                // Assert that this the r-disc connection scheme
                if (useKNearest_ == true)
                {
                    throw ompl::Exception("JIT sampling does not work with the k-nearest variant of BIT*.");
                }
            }

            // Store
            useJustInTimeSampling_ = useJit;
        }

        bool BITstar::getJustInTimeSampling() const
        {
            return useJustInTimeSampling_;
        }

        void BITstar::setDropSamplesOnPrune(bool dropSamples)
        {
            // If we're turning the function on, make sure the number of uniform states is up to date.
            if (dropSamples == true && dropSamplesOnPrune_ == false)
            {
                // Start at 0
                numUniformStates_ = 0u;

                // Add vertices and samples, if they exist
                if (static_cast<bool>(vertexNN_) == true)
                {
                    numUniformStates_ = numUniformStates_ + vertexNN_->size();
                }
                if (static_cast<bool>(freeStateNN_) == true)
                {
                    numUniformStates_ = numUniformStates_ + freeStateNN_->size();
                }

                // Remove starts and goals if this won't make us negative. This protects against the case where the
                // problem is setup with starts/goals but not started yet
                if (numUniformStates_ >= (startVerticesPtr_->size() + goalVerticesPtr_->size()))
                {
                    numUniformStates_ = numUniformStates_ - startVerticesPtr_->size() - goalVerticesPtr_->size();
                }
            }

            dropSamplesOnPrune_ = dropSamples;
        }

        bool BITstar::getDropSamplesOnPrune() const
        {
            return dropSamplesOnPrune_;
        }

        void BITstar::setStopOnSolnImprovement(bool stopOnChange)
        {
            stopOnSolnChange_ = stopOnChange;
        }

        bool BITstar::getStopOnSolnImprovement() const
        {
            return stopOnSolnChange_;
        }

        ompl::base::Cost BITstar::bestCost() const
        {
            return bestCost_;
        }

        std::string BITstar::bestCostProgressProperty() const
        {
            return std::to_string(this->bestCost().value());
        }

        std::string BITstar::bestLengthProgressProperty() const
        {
            return std::to_string(bestLength_);
        }

        std::string BITstar::currentFreeProgressProperty() const
        {
            return std::to_string(freeStateNN_->size());
        }

        std::string BITstar::currentVertexProgressProperty() const
        {
            return std::to_string(vertexNN_->size());
        }

        std::string BITstar::vertexQueueSizeProgressProperty() const
        {
            return std::to_string(intQueue_->numVertices());
        }

        std::string BITstar::edgeQueueSizeProgressProperty() const
        {
            return std::to_string(intQueue_->numEdges());
        }

        unsigned int BITstar::numIterations() const
        {
            return numIterations_;
        }

        std::string BITstar::iterationProgressProperty() const
        {
            return std::to_string(this->numIterations());
        }

        unsigned int BITstar::numBatches() const
        {
            return numBatches_;
        }

        std::string BITstar::batchesProgressProperty() const
        {
            return std::to_string(this->numBatches());
        }

        std::string BITstar::pruningProgressProperty() const
        {
            return std::to_string(numPrunings_);
        }

        std::string BITstar::totalStatesCreatedProgressProperty() const
        {
            return std::to_string(numSamples_);
        }

        std::string BITstar::verticesConstructedProgressProperty() const
        {
            return std::to_string(numVertices_);
        }

        std::string BITstar::statesPrunedProgressProperty() const
        {
            return std::to_string(numFreeStatesPruned_);
        }

        std::string BITstar::verticesDisconnectedProgressProperty() const
        {
            return std::to_string(numVerticesDisconnected_);
        }

        std::string BITstar::rewiringProgressProperty() const
        {
            return std::to_string(numRewirings_);
        }

        std::string BITstar::stateCollisionCheckProgressProperty() const
        {
            return std::to_string(numStateCollisionChecks_);
        }

        std::string BITstar::edgeCollisionCheckProgressProperty() const
        {
            return std::to_string(numEdgeCollisionChecks_);
        }

        std::string BITstar::nearestNeighbourProgressProperty() const
        {
            return std::to_string(numNearestNeighbours_);
        }

        std::string BITstar::edgesProcessedProgressProperty() const
        {
            return std::to_string(numEdgesProcessed_);
        }
        /////////////////////////////////////////////////////////////////////////////////////////////
    }  // geometric
}  // ompl
