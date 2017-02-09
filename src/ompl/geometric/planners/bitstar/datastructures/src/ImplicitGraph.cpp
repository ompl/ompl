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
#include "ompl/geometric/planners/bitstar/datastructures/ImplicitGraph.h"

// STL/Boost:
// For std::move
#include <utility>
// For smart pointers
#include <memory>
// For, you know, math
#include <cmath>
// For boost math constants
#include <boost/math/constants/constants.hpp>

// OMPL:
// For OMPL_INFORM et al.
#include "ompl/util/Console.h"
// For exceptions:
#include "ompl/util/Exception.h"
// For SelfConfig
#include "ompl/tools/config/SelfConfig.h"
// For RNG
#include "ompl/util/RandomNumbers.h"
// For geometric equations like unitNBallMeasure
#include "ompl/util/GeometricEquations.h"

// BIT*:
// The vertex class:
#include "ompl/geometric/planners/bitstar/datastructures/Vertex.h"
// The cost-helper class:
#include "ompl/geometric/planners/bitstar/datastructures/CostHelper.h"
// The search queue class
#include "ompl/geometric/planners/bitstar/datastructures/SearchQueue.h"

namespace ompl
{
    namespace geometric
    {
        /////////////////////////////////////////////////////////////////////////////////////////////
        // Public functions:
        BITstar::ImplicitGraph::ImplicitGraph(std::function<std::string()> nameFunc)
          : nameFunc_(std::move(nameFunc))
          , isSetup_(false)
          , si_(nullptr)
          , pdef_(nullptr)
          , costHelpPtr_(nullptr)
          , queuePtr_(nullptr)
          , rng_()
          , sampler_(nullptr)
          , startVertices_()
          , goalVertices_()
          , prunedStartVertices_()
          , prunedGoalVertices_()
          , newSamples_()
          , recycledSamples_()
          , freeStateNN_(nullptr)
          , vertexNN_(nullptr)
          , samplesInThisBatch_(0u)
          , numUniformStates_(0u)
          , r_(0.0)                     // Purposeful Gibberish
          , k_rgg_(0.0)                 // Purposeful Gibberish
          , k_(0u)                      // Purposeful Gibberish
          , approximationMeasure_(0.0)  // Gets set in setup with the proper call to si_->getSpaceMeasure()
          , minCost_(std::numeric_limits<double>::infinity())      // Gets set in setup to the proper calls from
                                                                   // OptimizationObjective
          , maxCost_(std::numeric_limits<double>::infinity())      // Gets set in setup to the proper calls from
                                                                   // OptimizationObjective
          , costSampled_(std::numeric_limits<double>::infinity())  // Gets set in setup to the proper calls from
                                                                   // OptimizationObjective
          , hasExactSolution_(false)
          , closestVertexToGoal_(nullptr)
          , closestDistToGoal_(std::numeric_limits<double>::infinity())
          , numSamples_(0u)
          , numVertices_(0u)
          , numFreeStatesPruned_(0u)
          , numVerticesDisconnected_(0u)
          , numNearestNeighbours_(0u)
          , numStateCollisionChecks_(0u)
          , rewireFactor_(1.1)
          , useKNearest_(true)
          , useJustInTimeSampling_(false)
          , dropSamplesOnPrune_(false)
          , findApprox_(false)
        {
        }

        void BITstar::ImplicitGraph::setup(const ompl::base::SpaceInformationPtr &si,
                                           const ompl::base::ProblemDefinitionPtr &pdef,
                                           const CostHelperPtr &costHelper, const SearchQueuePtr &searchQueue,
                                           const ompl::base::Planner *plannerPtr, ompl::base::PlannerInputStates &pis)
        {
            // Store that I am setup so that any debug-level tests will pass. This requires assuring that this function
            // is ordered properly.
            isSetup_ = true;

            // Store arguments
            si_ = si;
            pdef_ = pdef;
            costHelpPtr_ = costHelper;
            queuePtr_ = searchQueue;

            // Configure the nearest-neighbour constructs.
            // Only allocate if they are empty (as they can be set to a specific version by a call to
            // setNearestNeighbors)
            if (static_cast<bool>(freeStateNN_) == false)
            {
                freeStateNN_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<VertexPtr>(plannerPtr));
            }
            // No else, already allocated (by a call to setNearestNeighbors())

            if (static_cast<bool>(vertexNN_) == false)
            {
                vertexNN_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<VertexPtr>(plannerPtr));
            }
            // No else, already allocated (by a call to setNearestNeighbors())

            // Configure:
            NearestNeighbors<VertexPtr>::DistanceFunction distfun(
                [this](const VertexConstPtr &a, const VertexConstPtr &b)
                {
                    return distanceFunction(a, b);
                });
            freeStateNN_->setDistanceFunction(distfun);
            vertexNN_->setDistanceFunction(distfun);

            // Set the min, max and sampled cost to the proper objective-based values:
            minCost_ = costHelpPtr_->infiniteCost();
            maxCost_ = costHelpPtr_->infiniteCost();
            costSampled_ = costHelpPtr_->infiniteCost();

            // Add any start and goals vertices that exist to the queue, but do NOT wait for any more goals:
            this->updateStartAndGoalStates(pis, ompl::base::plannerAlwaysTerminatingCondition());

            // Get the measure of the problem
            approximationMeasure_ = si_->getSpaceMeasure();

            // Does the problem have finite boundaries?
            if (std::isfinite(approximationMeasure_) == false)
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
                if (startVertices_.empty() == true || goalVertices_.empty() == true)
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
                for (const auto &startVertex : startVertices_)
                {
                    for (const auto &goalVertex : goalVertices_)
                    {
                        maxDist = std::max(maxDist, si_->distance(startVertex->stateConst(), goalVertex->stateConst()));
                    }
                }

                // Calculate an estimate of the problem measure by (hyper)cubing the max distance
                approximationMeasure_ = std::pow(distScale * maxDist, si_->getStateDimension());
            }
            // No else, finite problem dimension

            // Finally initialize the nearestNeighbour terms:
            // Calculate the k-nearest constant
            k_rgg_ = this->minimumRggK();

            // Make the initial k all vertices:
            k_ = startVertices_.size() + goalVertices_.size();

            // Make the initial r infinity
            r_ = std::numeric_limits<double>::infinity();
        }

        void BITstar::ImplicitGraph::clear()
        {
            // Reset everything to the state of construction OTHER than planner name and settings/parameters
            // Keep this in the order of the constructors for easy verification:

            // Mark as cleared
            isSetup_ = false;

            // Pointers given at setup
            si_.reset();
            pdef_.reset();
            costHelpPtr_.reset();
            queuePtr_.reset();

            // Sampling
            rng_ = ompl::RNG();
            sampler_.reset();

            // Containers
            startVertices_.clear();
            goalVertices_.clear();
            prunedStartVertices_.clear();
            prunedGoalVertices_.clear();
            newSamples_.clear();
            recycledSamples_.clear();

            // The set of samples
            if (static_cast<bool>(freeStateNN_) == true)
            {
                freeStateNN_->clear();
                freeStateNN_.reset();
            }
            // No else, not allocated

            // The set of vertices
            if (static_cast<bool>(vertexNN_) == true)
            {
                vertexNN_->clear();
                vertexNN_.reset();
            }

            // The various calculations and tracked values
            samplesInThisBatch_ = 0u;
            numUniformStates_ = 0u;
            r_ = 0.0;
            k_rgg_ = 0.0;  // This is a double for better rounding later
            k_ = 0u;

            approximationMeasure_ = si_->getSpaceMeasure();
            minCost_ = ompl::base::Cost(std::numeric_limits<double>::infinity());
            maxCost_ = ompl::base::Cost(std::numeric_limits<double>::infinity());
            costSampled_ = ompl::base::Cost(std::numeric_limits<double>::infinity());
            hasExactSolution_ = false;
            closestVertexToGoal_.reset();
            closestDistToGoal_ = std::numeric_limits<double>::infinity();

            // The planner property trackers:
            numSamples_ = 0u;
            numVertices_ = 0u;
            numFreeStatesPruned_ = 0u;
            numVerticesDisconnected_ = 0u;
            numNearestNeighbours_ = 0u;
            numStateCollisionChecks_ = 0u;

            // The various convenience pointers:
            // DO NOT reset the parameters:
            // rewireFactor_
            // useKNearest_
            // useJustInTimeSampling_
            // dropSamplesOnPrune_
            // findApprox_
        }

        double BITstar::ImplicitGraph::distanceFunction(const VertexConstPtr &a, const VertexConstPtr &b) const
        {
            this->confirmSetup();

#ifdef BITSTAR_DEBUG
            if (static_cast<bool>(a->stateConst()) == false)
            {
                throw ompl::Exception("a->state is unallocated");
            }
            if (static_cast<bool>(b->stateConst()) == false)
            {
                throw ompl::Exception("b->state is unallocated");
            }
#endif  // BITSTAR_DEBUG

            // Using RRTstar as an example, this order gives us the distance FROM the queried state TO the other
            // neighbours in the structure.
            // The distance function between two states
            return si_->distance(b->stateConst(), a->stateConst());
        }

        void BITstar::ImplicitGraph::nearestSamples(const VertexPtr &vertex, VertexPtrVector *neighbourSamples)
        {
            this->confirmSetup();

            // Make sure sampling has happened first:
            this->updateSamples(vertex);

            // Increment our counter:
            ++numNearestNeighbours_;

            if (useKNearest_ == true)
            {
                freeStateNN_->nearestK(vertex, k_, *neighbourSamples);
            }
            else
            {
                freeStateNN_->nearestR(vertex, r_, *neighbourSamples);
            }
        }

        void BITstar::ImplicitGraph::nearestVertices(const VertexPtr &vertex, VertexPtrVector *neighbourVertices)
        {
            this->confirmSetup();

            // Increment our counter:
            ++numNearestNeighbours_;

            if (useKNearest_ == true)
            {
                vertexNN_->nearestK(vertex, k_, *neighbourVertices);
            }
            else
            {
                vertexNN_->nearestR(vertex, r_, *neighbourVertices);
            }
        }

        void BITstar::ImplicitGraph::getGraphAsPlannerData(ompl::base::PlannerData &data) const
        {
            this->confirmSetup();

            // Add samples
            if (static_cast<bool>(freeStateNN_) == true)
            {
                // Variables:
                // The vector of unused samples:
                VertexPtrVector samples;

                // Get the vector of samples
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
            if (static_cast<bool>(vertexNN_) == true)
            {
                // Variables:
                // The vector of vertices in the graph:
                VertexPtrVector vertices;

                // Get the vector of vertices
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
        }

        void BITstar::ImplicitGraph::hasSolution(const ompl::base::Cost &solnCost)
        {
            this->confirmSetup();

            // We have a solution!
            hasExactSolution_ = true;

            // Store it's cost as the maximum we'd ever want to sample
            maxCost_ = solnCost;

            // Clear the approximate solution
            closestDistToGoal_ = std::numeric_limits<double>::infinity();
            closestVertexToGoal_.reset();
        }

        void BITstar::ImplicitGraph::updateStartAndGoalStates(ompl::base::PlannerInputStates &pis,
                                                              const base::PlannerTerminationCondition &ptc)
        {
            this->confirmSetup();

            // Variable
            // Whether we've added a start or goal:
            bool addedGoal = false;
            bool addedStart = false;
            // Whether we have to rebuid the queue, i.e.. whether we've called updateStartAndGoalStates before
            bool rebuildQueue = false;

            /*
            Add the new starts and goals to the vectors of said vertices. Do goals first, as they are only added as
            samples. We do this as nested conditions so we always call nextGoal(ptc) at least once (regardless of
            whether there are moreGoalStates or not) in case we have been given a non trivial PTC that wants us to wait,
            but do *not* call it again if there are no more goals (as in the nontrivial PTC case, doing so would cause
            us to wait out the ptc and never try to solve anything)
            */
            do
            {
                // Variable
                // A new goal pointer, if there are none, it will be a nullptr.
                // We will wait for the duration of PTC for a new goal to appear.
                const ompl::base::State *newGoal = pis.nextGoal(ptc);

                // Check if it's valid
                if (static_cast<bool>(newGoal) == true)
                {
                    // It is valid and we are adding a goal, we will need to rebuild the queue if any starts have
                    // previously been added as their (and any descendents') heuristic cost-to-go may change:
                    rebuildQueue = (startVertices_.empty() == false);

                    // Allocate the vertex pointer
                    goalVertices_.push_back(std::make_shared<Vertex>(si_, costHelpPtr_->getOptObj()));

                    // Copy the value into the state
                    si_->copyState(goalVertices_.back()->state(), newGoal);

                    // And add this goal to the set of samples:
                    this->addSample(goalVertices_.back());

                    // Mark that we've added:
                    addedGoal = true;
                }
                // No else, there was no goal.
            } while (pis.haveMoreGoalStates() == true);

            // And then do the for starts. We do this last as the starts are added to the queue, which uses a cost-to-go
            // heuristic in it's ordering, and for that we want all the goals updated.
            // As there is no way to wait for new *start* states, this loop can be cleaner
            // There is no need to rebuild the queue when we add start vertices, as the queue is ordered on current
            // cost-to-come, and adding a start doesn't change that.
            while (pis.haveMoreStartStates() == true)
            {
                // Variable
                // A new start pointer
                const ompl::base::State *newStart = pis.nextStart();

                // Allocate the vertex pointer:
                startVertices_.push_back(std::make_shared<Vertex>(si_, costHelpPtr_->getOptObj(), true));

                // Copy the value into the state:
                si_->copyState(startVertices_.back()->state(), newStart);

                // Add this start vertex to the queue. It is not a sample, so skip that step:
                queuePtr_->enqueueVertex(startVertices_.back(), false);

                // Mark that we've added:
                addedStart = true;
            }

            // Now, if we added a new start and have previously pruned goals, we may want to readd them.
            if (addedStart == true && prunedGoalVertices_.empty() == false)
            {
                // Variable
                // An iterator to the vector of pruned goals
                auto prunedGoalIter = prunedGoalVertices_.begin();
                // The end point of the vector to consider. We will delete by swapping elements to the end, moving this
                // iterator towards the start, and then erasing once at the end.
                auto prunedGoalEnd = prunedGoalVertices_.end();

                // Consider each one
                while (prunedGoalIter != prunedGoalEnd)
                {
                    // Mark as unpruned
                    (*prunedGoalIter)->markUnpruned();

                    // Check if it should be readded (i.e., would it be pruned *now*?)
                    if (queuePtr_->vertexPruneCondition(*prunedGoalIter) == true)
                    {
                        // It would be pruned, so remark as pruned
                        (*prunedGoalIter)->markPruned();

                        // and move onto the next:
                        ++prunedGoalIter;
                    }
                    else
                    {
                        // It would not be pruned now, so readd it!
                        // Add back to the vector:
                        goalVertices_.push_back(*prunedGoalIter);

                        // Add as a sample
                        this->addSample(*prunedGoalIter);

                        // Mark what we've added:
                        addedGoal = true;

                        // Remove this goal from the vector of pruned vertices.
                        // Swap it to the element before our *new* end
                        if (prunedGoalIter != (prunedGoalEnd - 1))
                        {
                            std::swap(*prunedGoalIter, *(prunedGoalEnd - 1));
                        }

                        // Move the end forward by one, marking it to be deleted
                        --prunedGoalEnd;

                        // Leave the iterator where it is, as we need to recheck this element that we pulled from the
                        // back

                        // Just like the other new goals, we will need to rebuild the queue if any starts have
                        // previously been added. Which was a condition to be here in the first place
                        rebuildQueue = true;
                    }
                }

                // Erase any elements moved to the "new end" of the vector
                if (prunedGoalEnd != prunedGoalVertices_.end())
                {
                    prunedGoalVertices_.erase(prunedGoalEnd, prunedGoalVertices_.end());
                }
                // No else, nothing to delete
            }

            // Similarly, if we added a goal and have previously pruned starts, we will have to do the same on those
            if (addedGoal == true && prunedStartVertices_.empty() == false)
            {
                // Variable
                // An iterator to the vector of pruned starts
                auto prunedStartIter = prunedStartVertices_.begin();
                // The end point of the vector to consider. We will delete by swapping elements to the end, moving this
                // iterator towards the start, and then erasing once at the end.
                auto prunedStartEnd = prunedStartVertices_.end();

                // Consider each one
                while (prunedStartIter != prunedStartEnd)
                {
                    // Mark as unpruned
                    (*prunedStartIter)->markUnpruned();

                    // Check if it should be readded (i.e., would it be pruned *now*?)
                    if (queuePtr_->vertexPruneCondition(*prunedStartIter) == true)
                    {
                        // It would be pruned, so remark as pruned
                        (*prunedStartIter)->markPruned();

                        // and move onto the next:
                        ++prunedStartIter;
                    }
                    else
                    {
                        // It would not be pruned, readd it!
                        // Add it back to the vector
                        startVertices_.push_back(*prunedStartIter);

                        // Add this start vertex to the queue. It is not a sample, so skip that step:
                        queuePtr_->enqueueVertex(*prunedStartIter, false);

                        // Mark what we've added:
                        addedStart = true;

                        // Remove this start from the vector of pruned vertices.
                        // Swap it to the element before our *new* end
                        if (prunedStartIter != (prunedStartEnd - 1))
                        {
                            std::swap(*prunedStartIter, *(prunedStartEnd - 1));
                        }

                        // Move the end forward by one, marking it to be deleted
                        --prunedStartEnd;

                        // Leave the iterator where it is, as we need to recheck this element that we pulled from the
                        // back
                    }
                }

                // Erase any elements moved to the "new end" of the vector
                if (prunedStartEnd != prunedStartVertices_.end())
                {
                    prunedStartVertices_.erase(prunedStartEnd, prunedStartVertices_.end());
                }
                // No else, nothing to delete
            }

            // If we've added anything, we have some updating to do.
            if (addedGoal == true || addedStart == true)
            {
                // Update the minimum cost
                for (const auto &startVertex : startVertices_)
                {
                    // Take the better of the min cost so far and the cost-to-go from this start
                    minCost_ = costHelpPtr_->betterCost(minCost_, costHelpPtr_->costToGoHeuristic(startVertex));
                }

                // If we have at least one start and goal, allocate a sampler
                if (startVertices_.size() > 0u && goalVertices_.size() > 0u)
                {
                    // There is a start and goal, allocate
                    sampler_ = costHelpPtr_->getOptObj()->allocInformedStateSampler(
                        pdef_, std::numeric_limits<unsigned int>::max());
                }
                // No else, this will get allocated when we get the updated start/goal.

                // Was there an existing queue that needs to be rebuilt?
                if (rebuildQueue == true)
                {
                    // There was, inform
                    OMPL_INFORM("%s: Added new starts and/or goals to the problem. Rebuilding the queue.",
                                nameFunc_().c_str());

                    // Flag the queue as unsorted downstream from every existing start.
                    for (const auto &startVertex : startVertices_)
                    {
                        queuePtr_->markVertexUnsorted(startVertex);
                    }

                    // Resort the queue.
                    queuePtr_->resort();
                }
                // No else

                // Iterate through the existing vertices and find the current best approximate solution (if enabled)
                if (hasExactSolution_ == false && findApprox_ == true)
                {
                    this->findVertexClosestToGoal();
                }
            }
            // No else, why were we called?

            // Make sure that if we have a goal, we also have a start, since there's no way to wait for more *starts*
            if (goalVertices_.empty() == false && startVertices_.empty() == true)
            {
                OMPL_WARN("%s (ImplicitGraph): The problem has a goal but not a start. Since PlannerInputStates "
                          "provides no method to "
                          "wait for a _start_ state, BIT* will probably not work at all.",
                          nameFunc_().c_str());
            }
            // No else
        }

        void BITstar::ImplicitGraph::addNewSamples(const unsigned int &numSamples)
        {
            this->confirmSetup();

            // Set the cost sampled to the minimum
            costSampled_ = minCost_;

            // Store the number of samples being used in this batch
            samplesInThisBatch_ = numSamples;

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

            // Reuse the recycled samples as new samples
            for (auto &sample : recycledSamples_)
            {
                this->addSample(sample);
            }

            // These recycled samples are our only new samples
            newSamples_ = recycledSamples_;

            // And there are no longer and recycled samples
            recycledSamples_.clear();

            // We don't add *new* samples until the next time "nearSamples" is called. This is to support JIT sampling.
        }

        std::pair<unsigned int, unsigned int> BITstar::ImplicitGraph::prune(double prunedMeasure)
        {
            this->confirmSetup();

#ifdef BITSTAR_DEBUG
            if (hasExactSolution_ == false)
            {
                throw ompl::Exception("A graph cannot be pruned until a solution is found");
            }
#endif  // BITSTAR_DEBUG

            // Variable
            std::pair<unsigned int, unsigned int> numPruned(0u, 0u);

            // Store the measure of the problem I'm approximating
            approximationMeasure_ = prunedMeasure;

            // Prune the starts/goals
            numPruned = numPruned + this->pruneStartsGoals();

            // Prune the samples
            numPruned.second = numPruned.second + this->pruneSamples();

            return numPruned;
        }

        void BITstar::ImplicitGraph::addSample(const VertexPtr &newSample)
        {
            this->confirmSetup();

            // NO COUNTER. generated samples are counted at the sampler.

            // Mark as new
            newSample->markNew();

            // Add to the vector of new samples
            newSamples_.push_back(newSample);

            // Add to the NN structure:
            freeStateNN_->add(newSample);
        }

        void BITstar::ImplicitGraph::removeSample(const VertexPtr &oldSample)
        {
            this->confirmSetup();

            // Variable:
            // Create a copy of the vertex pointer so we don't delete it out from under ourselves.
            VertexPtr sampleToDelete(oldSample);

            // Increment our counter
            ++numFreeStatesPruned_;

            // Remove from the set of samples
            freeStateNN_->remove(sampleToDelete);

            // Mark the sample as pruned
            sampleToDelete->markPruned();
        }

        void BITstar::ImplicitGraph::addVertex(const VertexPtr &newVertex, bool removeFromFree)
        {
            this->confirmSetup();

#ifdef BITSTAR_DEBUG
            // Make sure it's connected first, so that the queue gets updated properly.
            // This is a day of debugging I'll never get back
            if (newVertex->isInTree() == false)
            {
                throw ompl::Exception("Vertices must be connected to the graph before adding");
            }
#endif  // BITSTAR_DEBUG

            // Increment the number of vertices added:
            ++numVertices_;

            // Remove the vertex from the set of samples (if it even existed)
            if (removeFromFree == true)
            {
                freeStateNN_->remove(newVertex);
            }
            // No else

            // Add to the NN structure:
            vertexNN_->add(newVertex);

            // Update the nearest vertex to the goal (if tracking)
            if (hasExactSolution_ == false && findApprox_ == true)
            {
                this->testClosestToGoal(newVertex);
            }
        }

        unsigned int BITstar::ImplicitGraph::removeVertex(const VertexPtr &oldVertex, bool moveToFree)
        {
            this->confirmSetup();

            // Variable:
            // A copy of the vertex pointer to be removed so we can't delete it out from under ourselves (occurs when
            // this function is given an element of the maintained set as the argument)
            VertexPtr vertexToDelete(oldVertex);

            // Increment our counter
            ++numVerticesDisconnected_;

            // Remove from the nearest-neighbour structure
            vertexNN_->remove(vertexToDelete);

            // Add back as sample, if that would be beneficial
            if (moveToFree == true && queuePtr_->samplePruneCondition(vertexToDelete) == false)
            {
                // Yes, the vertex is still useful as a sample. Track as recycled so they are reused as samples in the
                // next batch.
                recycledSamples_.push_back(vertexToDelete);

                // Return that the vertex was recycled
                return 0u;
            }
            else
            {
                // No, the vertex is not useful anymore. Mark as pruned. This functions as a lock to prevent accessing
                // anything about the vertex.
                vertexToDelete->markPruned();

                // Return that the vertex was completely pruned
                return 1u;
            }
        }
        /////////////////////////////////////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////////////////////////////////////
        // Private functions:
        void BITstar::ImplicitGraph::updateSamples(const VertexConstPtr &vertex)
        {
            // Variable
            // The required cost to contain the neighbourhood of this vertex:
            ompl::base::Cost costReqd = this->neighbourhoodCost(vertex);

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
                    sampleDensity = static_cast<double>(samplesInThisBatch_) / approximationMeasure_;

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
                    totalReqdSamples = numSamples_ + samplesInThisBatch_;
                }

                // Actually generate the new samples
                while (numSamples_ < totalReqdSamples)
                {
                    // Variable
                    // The new state:
                    auto newState = std::make_shared<Vertex>(si_, costHelpPtr_->getOptObj());

                    // Sample in the interval [costSampled_, costReqd):
                    sampler_->sampleUniform(newState->state(), costSampled_, costReqd);

                    // If the state is collision free, add it to the set of free states
                    ++numStateCollisionChecks_;
                    if (si_->isValid(newState->stateConst()) == true)
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

        void BITstar::ImplicitGraph::findVertexClosestToGoal()
        {
            if (static_cast<bool>(vertexNN_) == true)
            {
                // Variable
                // The vertices in the graph
                VertexPtrVector vertices;

                // Get the vector of vertices
                vertexNN_->list(vertices);

                // Iterate through them testing which is the closest to the goal.
                for (const auto &vertex : vertices)
                {
                    this->testClosestToGoal(vertex);
                }
            }
            // No else, I do nothing.
        }

        std::pair<unsigned int, unsigned int> BITstar::ImplicitGraph::pruneStartsGoals()
        {
            // Variable
            // The number of starts/goals disconnected from the tree/pruned
            std::pair<unsigned int, unsigned int> numPruned(0u, 0u);

            // Are there superfluous starts to prune?
            if (startVertices_.size() > 1u)
            {
                // Yes, Iterate through the vector

                // Variable
                // The iterator to the start:
                auto startIter = startVertices_.begin();
                // The end point of the vector to consider. We will delete by swapping elements to the end, moving this
                // iterator towards the start, and then erasing once at the end.
                auto startEnd = startVertices_.end();

                // Run until at the end:
                while (startIter != startEnd)
                {
                    // Check if this start has met the criteria to be pruned
                    if (queuePtr_->vertexPruneCondition(*startIter) == true)
                    {
                        // It has, remove the start vertex DO NOT consider it as a sample. It is marked as a root node,
                        // so having it as a sample would cause all kinds of problems, also it shouldn't be possible for
                        // it to ever be useful as a sample anyway, unless there is a very weird cost function in play.
                        numPruned.second = numPruned.second + this->removeVertex(*startIter, false);

                        // Count as a disconnected vertex
                        ++numPruned.first;

                        // Remove it from the queue
                        queuePtr_->unqueueVertex(*startIter);

                        // Store the start vertex in the pruned vector, in case it later needs to be readded:
                        prunedStartVertices_.push_back(*startIter);

                        // Remove this start from the vector.
                        // Swap it to the element before our *new* end
                        if (startIter != (startEnd - 1))
                        {
                            std::swap(*startIter, *(startEnd - 1));
                        }

                        // Move the end forward by one, marking it to be deleted
                        --startEnd;

                        // Leave the iterator where it is, as we need to recheck this element that we pulled from the
                        // back
                    }
                    else
                    {
                        // Still valid, move to the next one:
                        ++startIter;
                    }
                }

                // Erase any elements moved to the "new end" of the vector
                if (startEnd != startVertices_.end())
                {
                    startVertices_.erase(startEnd, startVertices_.end());
                }
                // No else, nothing to delete
            }
            // No else, can't prune 1 start.

            // Are there superfluous goals to prune?
            if (goalVertices_.size() > 1u)
            {
                // Yes, Iterate through the vector

                // Variable
                // The iterator to the start:
                auto goalIter = goalVertices_.begin();
                // The end point of the vector to consider. We will delete by swapping elements to the end, moving this
                // iterator towards the start, and then erasing once at the end.
                auto goalEnd = goalVertices_.end();

                // Run until at the end:
                while (goalIter != goalEnd)
                {
                    // Check if this start has met the criteria to be pruned
                    if (queuePtr_->vertexPruneCondition(*goalIter) == true)
                    {
                        // It has, remove the goal vertex completely
                        // Check if this vertex is in the tree
                        if ((*goalIter)->isInTree() == true)
                        {
                            // It is, remove it from the queue
                            queuePtr_->unqueueVertex(*goalIter);

                            // and as a vertex, allowing it to move to the set of samples.
                            numPruned.second = numPruned.second + this->removeVertex(*goalIter, true);

                            // Count it as a disconnected vertex
                            ++numPruned.first;
                        }
                        else
                        {
                            // It is not, so just it like a sample
                            this->removeSample(*goalIter);

                            // Count a pruned sample
                            ++numPruned.second;
                        }

                        // Store the start vertex in the pruned vector, in case it later needs to be readded:
                        prunedGoalVertices_.push_back(*goalIter);

                        // Remove this goal from the vector.
                        // Swap it to the element before our *new* end
                        if (goalIter != (goalEnd - 1))
                        {
                            std::swap(*goalIter, *(goalEnd - 1));
                        }

                        // Move the end forward by one, marking it to be deleted
                        --goalEnd;

                        // Leave the iterator where it is, as we need to recheck this element that we pulled from the
                        // back
                    }
                    else
                    {
                        // The goal is still valid, get the next
                        ++goalIter;
                    }
                }

                // Erase any elements moved to the "new end" of the vector
                if (goalEnd != goalVertices_.end())
                {
                    goalVertices_.erase(goalEnd, goalVertices_.end());
                }
                // No else, nothing to delete
            }
            // No else, can't prune 1 goal.

            // We don't need to update our approximate solution (if we're providing one) as we will only prune once a
            // solution exists.

            // Return the amount of work done
            return numPruned;
        }

        unsigned int BITstar::ImplicitGraph::pruneSamples()
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

                // and increasing our global counter
                numFreeStatesPruned_ = numFreeStatesPruned_ + numPruned;
            }
            else
            {
                // Variable:
                // The vector of samples:
                VertexPtrVector samples;

                // Get the vector of samples
                freeStateNN_->list(samples);

                // Iterate through the vector and remove any samples that should not be in the queue
                for (const auto &freeSample : samples)
                {
                    // Check if this state should be pruned:
                    if (queuePtr_->samplePruneCondition(freeSample) == true)
                    {
                        // Yes, remove it
                        this->removeSample(freeSample);

                        // and increment the counter
                        ++numPruned;
                    }
                    // No else, keep.
                }
            }

            return numPruned;
        }

        void BITstar::ImplicitGraph::testClosestToGoal(const VertexConstPtr &newVertex)
        {
            // Variable
            // The distance from this vertex to the goal:
            double distFromGoal;

            // Find the shortest distance between the given vertex and a goal
            pdef_->getGoal()->isSatisfied(newVertex->stateConst(), &distFromGoal);

            // Compare to the current best approximate solution
            if (distFromGoal < closestDistToGoal_)
            {
                // Better, update the approximate solution
                closestVertexToGoal_ = newVertex;
                closestDistToGoal_ = distFromGoal;
            }
            // No else, don't update if worse
        }

        ompl::base::Cost BITstar::ImplicitGraph::neighbourhoodCost(const VertexConstPtr &vertex) const
        {
            // Are we using JIT sampling?
            if (useJustInTimeSampling_ == true)
            {
                // We are, return the maximum heuristic cost that represents a sample in the neighbourhood of the given
                // vertex.
                // There is no point generating samples worse the best solution (maxCost_) even if those samples are in
                // this vertex's neighbourhood.
                return costHelpPtr_->betterCost(
                    maxCost_, costHelpPtr_->combineCosts(costHelpPtr_->lowerBoundHeuristicVertex(vertex),
                                                         ompl::base::Cost(2.0 * r_)));
            }
            else
            {
                // We are not, return the maximum cost we'd ever want to sample
                return maxCost_;
            }
        }

        void BITstar::ImplicitGraph::updateNearestTerms()
        {
            // Variables:
            // The number of uniformly distributed states:
            unsigned int N;

            // Calculate N, are we dropping samples?
            if (dropSamplesOnPrune_ == true)
            {
                // We are, so we've been tracking the number of uniform states, just use that
                N = numUniformStates_;
            }
            else
            {
                // We are not, so then all vertices and samples are uniform, use that
                N = vertexNN_->size() + freeStateNN_->size();
            }

            // If this is the first batch, we will be calculating the connection limits from only the starts and goals
            // for an RGG with m samples. That will be a complex graph. In this case, let us calculate the connection
            // limits considering the samples about to be generated. Doing so is equivalent to setting an upper-bound on
            // the radius, which RRT* does with it's min(maxEdgeLength, RGG-radius).
            if (N == (startVertices_.size() + goalVertices_.size()))
            {
                N = N + samplesInThisBatch_;
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

        double BITstar::ImplicitGraph::calculateR(unsigned int N) const
        {
            // Variables
            // The dimension cast as a double for readibility;
            double dimDbl = static_cast<double>(si_->getStateDimension());
            // The size of the graph
            double cardDbl = static_cast<double>(N);

            // Calculate the term and return
            return rewireFactor_ * this->minimumRggR() * std::pow(std::log(cardDbl) / cardDbl, 1 / dimDbl);
        }

        unsigned int BITstar::ImplicitGraph::calculateK(unsigned int N) const
        {
            // Calculate the term and return
            return std::ceil(rewireFactor_ * k_rgg_ * std::log(static_cast<double>(N)));
        }

        double BITstar::ImplicitGraph::minimumRggR() const
        {
            // Variables
            // The dimension cast as a double for readibility;
            double dimDbl = static_cast<double>(si_->getStateDimension());

            // Calculate the term and return
            return 2.0 *
                   std::pow((1.0 + 1.0 / dimDbl) * (approximationMeasure_ / unitNBallMeasure(si_->getStateDimension())),
                            1.0 / dimDbl);  // RRG radius (biggest for unit-volume problem)
            /*
            return std::pow(2.0 * (1.0 + 1.0 / dimDbl) * (approximationMeasure_ /
            unitNBallMeasure(si_->getStateDimension())), 1.0 / dimDbl); //RRT* radius (smaller for unit-volume problem)
            return 2.0 * std::pow((1.0 / dimDbl) * (approximationMeasure_ / unitNBallMeasure(si_->getStateDimension())),
            1.0 / dimDbl); //FMT* radius (smallest for R2, equiv to RRT* for R3 and then middle for higher d. All
            unit-volume)
            */
        }

        double BITstar::ImplicitGraph::minimumRggK() const
        {
            // Variables
            // The dimension cast as a double for readibility;
            double dimDbl = static_cast<double>(si_->getStateDimension());

            // Calculate the term and return
            return boost::math::constants::e<double>() +
                   (boost::math::constants::e<double>() / dimDbl);  // RRG k-nearest
        }

        void BITstar::ImplicitGraph::confirmSetup() const
        {
#ifdef BITSTAR_DEBUG
            if (isSetup_ == false)
            {
                throw ompl::Exception("BITstar::ImplicitGraph was used before it was setup.");
            }
#endif  // BITSTAR_DEBUG
        }
        /////////////////////////////////////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////////////////////////////////////
        // Boring sets/gets (Public):
        bool BITstar::ImplicitGraph::hasAGoal() const
        {
            return (goalVertices_.empty() == false);
        }

        BITstar::VertexPtrVector::const_iterator BITstar::ImplicitGraph::startVerticesBeginConst() const
        {
            return startVertices_.cbegin();
        }

        BITstar::VertexPtrVector::const_iterator BITstar::ImplicitGraph::startVerticesEndConst() const
        {
            return startVertices_.cend();
        }

        BITstar::VertexPtrVector::const_iterator BITstar::ImplicitGraph::goalVerticesBeginConst() const
        {
            return goalVertices_.cbegin();
        }

        BITstar::VertexPtrVector::const_iterator BITstar::ImplicitGraph::goalVerticesEndConst() const
        {
            return goalVertices_.cend();
        }

        ompl::base::Cost BITstar::ImplicitGraph::minCost() const
        {
            return minCost_;
        }

        bool BITstar::ImplicitGraph::hasInformedMeasure() const
        {
            return sampler_->hasInformedMeasure();
        }

        double BITstar::ImplicitGraph::getInformedMeasure(const ompl::base::Cost &cost) const
        {
            return sampler_->getInformedMeasure(cost);
        }

        BITstar::VertexConstPtr BITstar::ImplicitGraph::closestVertexToGoal() const
        {
#ifdef BITSTAR_DEBUG
            if (findApprox_ == false)
            {
                throw ompl::Exception("Approximate solutions are not being tracked.");
            }
#endif  // BITSTAR_DEBUG
            return closestVertexToGoal_;
        }

        double BITstar::ImplicitGraph::smallestDistanceToGoal() const
        {
#ifdef BITSTAR_DEBUG
            if (findApprox_ == false)
            {
                throw ompl::Exception("Approximate solutions are not being tracked.");
            }
#endif  // BITSTAR_DEBUG
            return closestDistToGoal_;
        }

        unsigned int BITstar::ImplicitGraph::getConnectivityK() const
        {
#ifdef BITSTAR_DEBUG
            if (useKNearest_ == false)
            {
                throw ompl::Exception("Using an r-disc graph.");
            }
#endif  // BITSTAR_DEBUG
            return k_;
        }

        double BITstar::ImplicitGraph::getConnectivityR() const
        {
#ifdef BITSTAR_DEBUG
            if (useKNearest_ == true)
            {
                throw ompl::Exception("Using a k-nearest graph.");
            }
#endif  // BITSTAR_DEBUG
            return r_;
        }

        void BITstar::ImplicitGraph::setRewireFactor(double rewireFactor)
        {
            // Store
            rewireFactor_ = rewireFactor;

            // Check if there's things to update
            if (isSetup_ == true)
            {
                // Reinitialize the terms:
                this->updateNearestTerms();
            }
        }

        double BITstar::ImplicitGraph::getRewireFactor() const
        {
            return rewireFactor_;
        }

        void BITstar::ImplicitGraph::setUseKNearest(bool useKNearest)
        {
            // Assure that we're not trying to enable k-nearest with JIT sampling already on
            if (useKNearest == true && useJustInTimeSampling_ == true)
            {
                OMPL_WARN("%s (ImplicitGraph): The k-nearest variant of BIT* cannot be used with JIT sampling, "
                          "continuing to use the r-disc variant.",
                          nameFunc_().c_str());
            }
            else
            {
                // Store
                useKNearest_ = useKNearest;

                // Check if there's things to update
                if (isSetup_ == true)
                {
                    // Calculate the current term:
                    this->updateNearestTerms();
                }
            }
        }

        bool BITstar::ImplicitGraph::getUseKNearest() const
        {
            return useKNearest_;
        }

        void BITstar::ImplicitGraph::setJustInTimeSampling(bool useJit)
        {
            // Assure that we're not trying to enable k-nearest with JIT sampling already on
            if (useKNearest_ == true && useJit == true)
            {
                OMPL_WARN("%s (ImplicitGraph): Just-in-time sampling cannot be used with the k-nearest variant of "
                          "BIT*, continuing to use regular sampling.",
                          nameFunc_().c_str());
            }
            else
            {
                // Store
                useJustInTimeSampling_ = useJit;

                // Announce limitation:
                if (useJit == true)
                {
                    OMPL_INFORM("%s (ImplicitGraph): Just-in-time sampling is currently only implemented for problems "
                                "seeking to minimize path-length.",
                                nameFunc_().c_str());
                }
                // No else
            }
        }

        bool BITstar::ImplicitGraph::getJustInTimeSampling() const
        {
            return useJustInTimeSampling_;
        }

        void BITstar::ImplicitGraph::setDropSamplesOnPrune(bool dropSamples)
        {
            // Make sure we're not already setup
            if (isSetup_ == true)
            {
                OMPL_WARN("%s (ImplicitGraph): Periodic sample removal cannot be changed once BIT* is setup. "
                          "Continuing to use the previous setting.",
                          nameFunc_().c_str());
            }
            else
            {
                // Store
                dropSamplesOnPrune_ = dropSamples;
            }
        }

        bool BITstar::ImplicitGraph::getDropSamplesOnPrune() const
        {
            return dropSamplesOnPrune_;
        }

        void BITstar::ImplicitGraph::setTrackApproximateSolutions(bool findApproximate)
        {
            // Is the flag changing?
            if (findApproximate != findApprox_)
            {
                // Store the flag
                findApprox_ = findApproximate;

                // Check if we are enabling or disabling approximate solution support
                if (findApprox_ == false)
                {
                    // We're turning it off, clear the approximate solution variables:
                    closestDistToGoal_ = std::numeric_limits<double>::infinity();
                    closestVertexToGoal_.reset();
                }
                else
                {
                    // We are turning it on, do we have an exact solution?
                    if (hasExactSolution_ == false)
                    {
                        // We don't, find our current best approximate solution:
                        this->findVertexClosestToGoal();
                    }
                    // No else, exact is better than approximate.
                }
            }
            // No else, no change.
        }

        bool BITstar::ImplicitGraph::getTrackApproximateSolutions() const
        {
            return findApprox_;
        }

        template <template <typename T> class NN>
        void BITstar::ImplicitGraph::setNearestNeighbors()
        {
            // Check if the problem is already setup, if so, the NN structs have data in them and you can't really
            // change them:
            if (isSetup_ == true)
            {
                OMPL_WARN("%s (ImplicitGraph): The nearest neighbour datastructures cannot be changed once the problem "
                          "is setup. Continuing to use the existing containers.",
                          nameFunc_().c_str());
            }
            else
            {
                // The problem isn't setup yet, create NN structs of the specified type:
                freeStateNN_ = std::make_shared<NN<VertexPtr>>();
                vertexNN_ = std::make_shared<NN<VertexPtr>>();
            }
        }

        unsigned int BITstar::ImplicitGraph::numFreeSamples() const
        {
            return freeStateNN_->size();
        }

        unsigned int BITstar::ImplicitGraph::numConnectedVertices() const
        {
            return vertexNN_->size();
        }

        unsigned int BITstar::ImplicitGraph::numStatesGenerated() const
        {
            return numSamples_;
        }

        unsigned int BITstar::ImplicitGraph::numVerticesConnected() const
        {
            return numVertices_;
        }

        unsigned int BITstar::ImplicitGraph::numFreeStatesPruned() const
        {
            return numFreeStatesPruned_;
        }

        unsigned int BITstar::ImplicitGraph::numVerticesDisconnected() const
        {
            return numVerticesDisconnected_;
        }

        unsigned int BITstar::ImplicitGraph::numNearestLookups() const
        {
            return numNearestNeighbours_;
        }

        unsigned int BITstar::ImplicitGraph::numStateCollisionChecks() const
        {
            return numStateCollisionChecks_;
        }
        /////////////////////////////////////////////////////////////////////////////////////////////
    }  // geometric
}  // ompl
