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

// My definition:
#include "ompl/geometric/planners/informedtrees/bitstar/ImplicitGraph.h"

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
// A collection of common helper functions
#include "ompl/geometric/planners/informedtrees/bitstar/HelperFunctions.h"
// The vertex class:
#include "ompl/geometric/planners/informedtrees/bitstar/Vertex.h"
// The cost-helper class:
#include "ompl/geometric/planners/informedtrees/bitstar/CostHelper.h"
// The search queue class
#include "ompl/geometric/planners/informedtrees/bitstar/SearchQueue.h"

// Debug macros
#ifdef BITSTAR_DEBUG
/** \brief A debug-only call to assert that the object is setup. */
#define ASSERT_SETUP this->assertSetup();
#else
#define ASSERT_SETUP
#endif  // BITSTAR_DEBUG

namespace ompl
{
    namespace geometric
    {
        /////////////////////////////////////////////////////////////////////////////////////////////
        // Public functions:
        BITstar::ImplicitGraph::ImplicitGraph(NameFunc nameFunc)
          : nameFunc_(std::move(nameFunc)), approximationId_(std::make_shared<unsigned int>(1u))
        {
        }

        void BITstar::ImplicitGraph::setup(const ompl::base::SpaceInformationPtr &spaceInformation,
                                           const ompl::base::ProblemDefinitionPtr &problemDefinition,
                                           CostHelper *costHelper, SearchQueue *searchQueue,
                                           const ompl::base::Planner *plannerPtr,
                                           ompl::base::PlannerInputStates &inputStates)
        {
            // Store that I am setup so that any debug-level tests will pass. This requires assuring that this function
            // is ordered properly.
            isSetup_ = true;

            // Store arguments
            spaceInformation_ = spaceInformation;
            problemDefinition_ = problemDefinition;
            costHelpPtr_ = costHelper;
            queuePtr_ = searchQueue;

            // Configure the nearest-neighbour constructs.
            // Only allocate if they are empty (as they can be set to a specific version by a call to
            // setNearestNeighbors)
            if (!static_cast<bool>(samples_))
            {
                samples_.reset(ompl::tools::SelfConfig::getDefaultNearestNeighbors<VertexPtr>(plannerPtr));
            }
            // No else, already allocated (by a call to setNearestNeighbors())

            // Configure:
            NearestNeighbors<VertexPtr>::DistanceFunction distanceFunction(
                [this](const VertexConstPtr &a, const VertexConstPtr &b) { return distance(a, b); });
            samples_->setDistanceFunction(distanceFunction);

            // Set the min, max and sampled cost to the proper objective-based values:
            minCost_ = costHelpPtr_->infiniteCost();
            solutionCost_ = costHelpPtr_->infiniteCost();
            sampledCost_ = costHelpPtr_->infiniteCost();

            // Add any start and goals vertices that exist to the queue, but do NOT wait for any more goals:
            this->updateStartAndGoalStates(inputStates, ompl::base::plannerAlwaysTerminatingCondition());

            // Get the measure of the problem
            approximationMeasure_ = spaceInformation_->getSpaceMeasure();

            // Does the problem have finite boundaries?
            if (!std::isfinite(approximationMeasure_))
            {
                // It does not, so let's estimate a measure of the planning problem.
                // A not horrible place to start would be hypercube proportional to the distance between the start and
                // goal. It's not *great*, but at least it sort of captures the order-of-magnitude of the problem.

                // First, some asserts.
                // Check that JIT sampling is on, which is required for infinite problems
                if (!useJustInTimeSampling_)
                {
                    throw ompl::Exception("For unbounded planning problems, just-in-time sampling must be enabled "
                                          "before calling setup.");
                }
                // No else

                // Check that we have a start and goal
                if (startVertices_.empty() || goalVertices_.empty())
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
                        maxDist =
                            std::max(maxDist, spaceInformation_->distance(startVertex->state(), goalVertex->state()));
                    }
                }

                // Calculate an estimate of the problem measure by (hyper)cubing the max distance
                approximationMeasure_ = std::pow(distScale * maxDist, spaceInformation_->getStateDimension());
            }
            // No else, finite problem dimension

            // Finally initialize the nearestNeighbour terms:
            // Calculate the k-nearest constant
            k_rgg_ = this->calculateMinimumRggK();

            // Make the initial k all vertices:
            k_ = startVertices_.size() + goalVertices_.size();

            // Make the initial r infinity
            r_ = std::numeric_limits<double>::infinity();
        }

        void BITstar::ImplicitGraph::reset()
        {
            // Reset everything to the state of construction OTHER than planner name and settings/parameters
            // Keep this in the order of the constructors for easy verification:

            // Mark as cleared
            isSetup_ = false;

            // Pointers given at setup
            spaceInformation_.reset();
            problemDefinition_.reset();
            costHelpPtr_ = nullptr;
            queuePtr_ = nullptr;

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
            if (static_cast<bool>(samples_))
            {
                samples_->clear();
                samples_.reset();
            }
            // No else, not allocated

            // The various calculations and tracked values, same as in the header
            numNewSamplesInCurrentBatch_ = 0u;
            numUniformStates_ = 0u;
            r_ = 0.0;
            k_rgg_ = 0.0;  // This is a double for better rounding later
            k_ = 0u;

            approximationMeasure_ = 0.0;
            minCost_ = ompl::base::Cost(std::numeric_limits<double>::infinity());
            solutionCost_ = ompl::base::Cost(std::numeric_limits<double>::infinity());
            sampledCost_ = ompl::base::Cost(std::numeric_limits<double>::infinity());
            hasExactSolution_ = false;
            closestVertexToGoal_.reset();
            closestDistanceToGoal_ = std::numeric_limits<double>::infinity();

            // The planner property trackers:
            numSamples_ = 0u;
            numVertices_ = 0u;
            numFreeStatesPruned_ = 0u;
            numVerticesDisconnected_ = 0u;
            numNearestNeighbours_ = 0u;
            numStateCollisionChecks_ = 0u;

            // The approximation id.
            *approximationId_ = 1u;

            // The various convenience pointers:
            // DO NOT reset the parameters:
            // rewireFactor_
            // useKNearest_
            // useJustInTimeSampling_
            // dropSamplesOnPrune_
            // findApprox_
        }

        double BITstar::ImplicitGraph::distance(const VertexConstPtr &a, const VertexConstPtr &b) const
        {
            ASSERT_SETUP

#ifdef BITSTAR_DEBUG
            if (!static_cast<bool>(a->state()))
            {
                throw ompl::Exception("a->state is unallocated");
            }
            if (!static_cast<bool>(b->state()))
            {
                throw ompl::Exception("b->state is unallocated");
            }
#endif  // BITSTAR_DEBUG

            // Using RRTstar as an example, this order gives us the distance FROM the queried state TO the other
            // neighbours in the structure.
            // The distance function between two states
            return spaceInformation_->distance(b->state(), a->state());
        }

        double BITstar::ImplicitGraph::distance(const VertexConstPtrPair &vertices) const
        {
            return this->distance(vertices.first, vertices.second);
        }

        void BITstar::ImplicitGraph::nearestSamples(const VertexPtr &vertex, VertexPtrVector *neighbourSamples)
        {
            ASSERT_SETUP

            // Make sure sampling has happened first.
            this->updateSamples(vertex);

            // Keep track of how many times we've requested nearest neighbours.
            ++numNearestNeighbours_;

            if (useKNearest_)
            {
                samples_->nearestK(vertex, k_, *neighbourSamples);
            }
            else
            {
                samples_->nearestR(vertex, r_, *neighbourSamples);
            }
        }

        void BITstar::ImplicitGraph::getGraphAsPlannerData(ompl::base::PlannerData &data) const
        {
            ASSERT_SETUP

            // base::PlannerDataVertex takes a raw pointer to a state. I want to guarantee, that the state lives as long
            // as the program lives.
            static std::set<std::shared_ptr<Vertex>,
                            std::function<bool(const std::shared_ptr<Vertex> &, const std::shared_ptr<Vertex> &)>>
                liveStates([](const auto &lhs, const auto &rhs) { return lhs->getId() < rhs->getId(); });

            // Add samples
            if (static_cast<bool>(samples_))
            {
                // Get the samples as a vector.
                VertexPtrVector samples;
                samples_->list(samples);

                // Iterate through the samples.
                for (const auto &sample : samples)
                {
                    // Make sure the sample is not destructed before BIT* is.
                    liveStates.insert(sample);

                    // Add sample.
                    if (!sample->isRoot())
                    {
                        data.addVertex(ompl::base::PlannerDataVertex(sample->state(), sample->getId()));

                        // Add incoming edge.
                        if (sample->hasParent())
                        {
                            data.addEdge(ompl::base::PlannerDataVertex(sample->getParent()->state(),
                                                                       sample->getParent()->getId()),
                                         ompl::base::PlannerDataVertex(sample->state(), sample->getId()));
                        }
                    }
                    else
                    {
                        data.addStartVertex(ompl::base::PlannerDataVertex(sample->state(), sample->getId()));
                    }
                }
            }
            // No else.
        }

        void BITstar::ImplicitGraph::registerSolutionCost(const ompl::base::Cost &solutionCost)
        {
            ASSERT_SETUP

            // We have a solution!
            hasExactSolution_ = true;

            // Store it's cost as the maximum we'd ever want to sample
            solutionCost_ = solutionCost;

            // Clear the approximate solution
            closestDistanceToGoal_ = std::numeric_limits<double>::infinity();
            closestVertexToGoal_.reset();
        }

        void BITstar::ImplicitGraph::updateStartAndGoalStates(
            ompl::base::PlannerInputStates &inputStates, const base::PlannerTerminationCondition &terminationCondition)
        {
            ASSERT_SETUP

            // Variable
            // Whether we've added a start or goal:
            bool addedGoal = false;
            bool addedStart = false;
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
                const ompl::base::State *newGoal = inputStates.nextGoal(terminationCondition);

                // Check if it's valid
                if (static_cast<bool>(newGoal))
                {
                    // Allocate the vertex pointer
                    goalVertices_.push_back(
                        std::make_shared<Vertex>(spaceInformation_, costHelpPtr_, queuePtr_, approximationId_));

                    // Copy the value into the state
                    spaceInformation_->copyState(goalVertices_.back()->state(), newGoal);

                    // And add this goal to the set of samples:
                    this->addToSamples(goalVertices_.back());

                    // Mark that we've added:
                    addedGoal = true;
                }
                // No else, there was no goal.
            } while (inputStates.haveMoreGoalStates());

            /*
            And then do the same for starts. We do this last as the starts are added to the queue, which uses a
            cost-to-go heuristic in it's ordering, and for that we want all the goals updated. As there is no way to
            wait for new *start* states, this loop can be cleaner There is no need to rebuild the queue when we add
            start vertices, as the queue is ordered on current cost-to-come, and adding a start doesn't change that.
            */
            while (inputStates.haveMoreStartStates())
            {
                // Variable
                // A new start pointer, if  PlannerInputStates finds that it is invalid we will get a nullptr.
                const ompl::base::State *newStart = inputStates.nextStart();

                // Check if it's valid
                if (static_cast<bool>(newStart))
                {
                    // Allocate the vertex pointer:
                    startVertices_.push_back(
                        std::make_shared<Vertex>(spaceInformation_, costHelpPtr_, queuePtr_, approximationId_, true));

                    // Copy the value into the state.
                    spaceInformation_->copyState(startVertices_.back()->state(), newStart);

                    // Add the vertex to the set of vertices.
                    this->addToSamples(startVertices_.back());
                    this->registerAsVertex(startVertices_.back());

                    // Mark that we've added:
                    addedStart = true;
                }
                // No else, there was no start.
            }

            // Now, if we added a new start and have previously pruned goals, we may want to readd them.
            if (addedStart && !prunedGoalVertices_.empty())
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
                    if (this->canVertexBeDisconnected(*prunedGoalIter))
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
                        this->addToSamples(*prunedGoalIter);

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
            if (addedGoal && !prunedStartVertices_.empty())
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
                    if (this->canVertexBeDisconnected(*prunedStartIter))
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

                        // Add this vertex to the queue.
                        // queuePtr_->enqueueVertex(*prunedStartIter);

                        // Add the vertex to the set of vertices.
                        this->registerAsVertex(*prunedStartIter);

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
            if (addedGoal || addedStart)
            {
                // Update the minimum cost
                for (const auto &startVertex : startVertices_)
                {
                    // Take the better of the min cost so far and the cost-to-go from this start
                    minCost_ = costHelpPtr_->betterCost(minCost_, costHelpPtr_->costToGoHeuristic(startVertex));
                }

                // If we have at least one start and goal, allocate a sampler
                if (!startVertices_.empty() && !goalVertices_.empty())
                {
                    // There is a start and goal, allocate
                    sampler_ = costHelpPtr_->getOptObj()->allocInformedStateSampler(
                        problemDefinition_, std::numeric_limits<unsigned int>::max());
                }
                // No else, this will get allocated when we get the updated start/goal.

                // Iterate through the existing vertices and find the current best approximate solution (if enabled)
                if (!hasExactSolution_ && findApprox_)
                {
                    this->updateVertexClosestToGoal();
                }
            }
            // No else, why were we called?

            // Make sure that if we have a goal, we also have a start, since there's no way to wait for more *starts*
            if (!goalVertices_.empty() && startVertices_.empty())
            {
                OMPL_WARN("%s (ImplicitGraph): The problem has a goal but not a start. BIT* cannot find a solution "
                          "since PlannerInputStates provides no method to wait for a valid _start_ state to appear.",
                          nameFunc_().c_str());
            }
            // No else
        }

        void BITstar::ImplicitGraph::addNewSamples(const unsigned int &numSamples)
        {
            ASSERT_SETUP

            // Set the cost sampled to identity
            sampledCost_ = costHelpPtr_->identityCost();

            // Store the number of samples being used in this batch
            numNewSamplesInCurrentBatch_ = numSamples;

            // Update the nearest-neighbour terms for the number of samples we *will* have.
            this->updateNearestTerms();

            // Add the recycled samples to the nearest neighbours struct.
            samples_->add(recycledSamples_);

            // These recycled samples are our only new samples.
            newSamples_ = recycledSamples_;

            // And there are no longer and recycled samples.
            recycledSamples_.clear();

            // Increment the approximation id.
            ++(*approximationId_);

            // We don't add actual *new* samples until the next time "nearestSamples" is called. This is to support JIT
            // sampling.
        }

        std::pair<unsigned int, unsigned int> BITstar::ImplicitGraph::prune(double prunedMeasure)
        {
            ASSERT_SETUP

#ifdef BITSTAR_DEBUG
            if (!hasExactSolution_)
            {
                throw ompl::Exception("A graph cannot be pruned until a solution is found");
            }
#endif  // BITSTAR_DEBUG

            // Store the measure of the problem I'm approximating
            approximationMeasure_ = prunedMeasure;

            // Prune the starts/goals
            std::pair<unsigned int, unsigned int> numPruned = this->pruneStartAndGoalVertices();

            // Prune the samples.
            numPruned = numPruned + this->pruneSamples();

            return numPruned;
        }

        void BITstar::ImplicitGraph::addToSamples(const VertexPtr &sample)
        {
            ASSERT_SETUP

            // NO COUNTER. generated samples are counted at the sampler.

            // Add to the vector of new samples
            newSamples_.push_back(sample);

            // Add to the NN structure:
            samples_->add(sample);
        }

        void BITstar::ImplicitGraph::addToSamples(const VertexPtrVector &samples)
        {
            ASSERT_SETUP

            // NO COUNTER. generated samples are counted at the sampler.

            // Add to the vector of new samples
            newSamples_.insert(newSamples_.end(), samples.begin(), samples.end());

            // Add to the NN structure:
            samples_->add(samples);
        }

        void BITstar::ImplicitGraph::removeFromSamples(const VertexPtr &sample)
        {
            ASSERT_SETUP

            // Remove from the set of samples
            samples_->remove(sample);
        }

        void BITstar::ImplicitGraph::pruneSample(const VertexPtr &sample)
        {
            ASSERT_SETUP

            // Variable:
#ifdef BITSTAR_DEBUG
            // The use count of the passed shared pointer. Used in debug mode to assert that we took ownership of our
            // own copy.
            unsigned int initCount = sample.use_count();
#endif  // BITSTAR_DEBUG
        // A copy of the sample pointer to be removed so we can't delete it out from under ourselves (occurs when
        // this function is given an element of the maintained set as the argument)
            VertexPtr sampleCopy(sample);

#ifdef BITSTAR_DEBUG
            // Assert that the vertexToDelete took it's own copy
            if (sampleCopy.use_count() <= initCount)
            {
                throw ompl::Exception("A code change has prevented ImplicitGraph::removeSample() "
                                      "from taking it's own copy of the given shared pointer. See "
                                      "https://github.com/ompl/ompl/issues/485");
            }
            if (sampleCopy->edgeQueueOutLookupSize() != 0u)
            {
                throw ompl::Exception("Encountered a sample with outgoing edges in the queue.");
            }
#endif  // BITSTAR_DEBUG

            // Remove all incoming edges from the search queue.
            queuePtr_->removeInEdgesConnectedToVertexFromQueue(sampleCopy);

            // Remove from the set of samples
            samples_->remove(sampleCopy);

            // Increment our counter
            ++numFreeStatesPruned_;

            // Mark the sample as pruned
            sampleCopy->markPruned();
        }

        void BITstar::ImplicitGraph::recycleSample(const VertexPtr &sample)
        {
            ASSERT_SETUP

            recycledSamples_.push_back(sample);
        }

        void BITstar::ImplicitGraph::registerAsVertex(const VertexPtr &vertex)
        {
            ASSERT_SETUP
#ifdef BITSTAR_DEBUG
            // Make sure it's connected first, so that the queue gets updated properly.
            // This is a day of debugging I'll never get back
            if (!vertex->isInTree())
            {
                throw ompl::Exception("Vertices must be connected to the graph before adding");
            }
#endif  // BITSTAR_DEBUG

            // Increment the number of vertices added:
            ++numVertices_;

            // Update the nearest vertex to the goal (if tracking)
            if (!hasExactSolution_ && findApprox_)
            {
                this->testClosestToGoal(vertex);
            }
        }

        unsigned int BITstar::ImplicitGraph::removeFromVertices(const VertexPtr &vertex, bool moveToFree)
        {
            ASSERT_SETUP

            // Variable:
#ifdef BITSTAR_DEBUG
            // The use count of the passed shared pointer. Used in debug mode to assert that we took ownership of our
            // own copy.
            unsigned int initCount = vertex.use_count();
#endif  // BITSTAR_DEBUG
        // A copy of the vertex pointer to be removed so we can't delete it out from under ourselves (occurs when
        // this function is given an element of the maintained set as the argument)
            VertexPtr vertexCopy(vertex);

#ifdef BITSTAR_DEBUG
            // Assert that the vertexToDelete took it's own copy
            if (vertexCopy.use_count() <= initCount)
            {
                throw ompl::Exception("A code change has prevented ImplicitGraph::removeVertex() "
                                      "from taking it's own copy of the given shared pointer. See "
                                      "https://github.com/ompl/ompl/issues/485");
            }
#endif  // BITSTAR_DEBUG

            // Increment our counter
            ++numVerticesDisconnected_;

            // Remove from the nearest-neighbour structure
            samples_->remove(vertexCopy);

            // Add back as sample, if that would be beneficial
            if (moveToFree && !this->canSampleBePruned(vertexCopy))
            {
                // Yes, the vertex is still useful as a sample. Track as recycled so they are reused as samples in the
                // next batch.
                recycledSamples_.push_back(vertexCopy);

                // Return that the vertex was recycled
                return 0u;
            }
            else
            {
                // No, the vertex is not useful anymore. Mark as pruned. This functions as a lock to prevent accessing
                // anything about the vertex.
                vertexCopy->markPruned();

                // Return that the vertex was completely pruned
                return 1u;
            }
        }

        std::pair<unsigned int, unsigned int> BITstar::ImplicitGraph::pruneVertex(const VertexPtr &vertex)
        {
            ASSERT_SETUP

            // Variable:
#ifdef BITSTAR_DEBUG
            // The use count of the passed shared pointer. Used in debug mode to assert that we took ownership of our
            // own copy.
            unsigned int initCount = vertex.use_count();
#endif  // BITSTAR_DEBUG
        // A copy of the sample pointer to be removed so we can't delete it out from under ourselves (occurs when
        // this function is given an element of the maintained set as the argument)
            VertexPtr vertexCopy(vertex);

#ifdef BITSTAR_DEBUG
            // Assert that the vertexToDelete took it's own copy
            if (vertexCopy.use_count() <= initCount)
            {
                throw ompl::Exception("A code change has prevented ImplicitGraph::removeSample() "
                                      "from taking it's own copy of the given shared pointer. See "
                                      "https://github.com/ompl/ompl/issues/485");
            }
#endif  // BITSTAR_DEBUG

            // Remove from the set of inconsistent vertices if the vertex is inconsistent.
            if (!vertexCopy->isConsistent())
            {
                queuePtr_->removeFromInconsistentSet(vertexCopy);
            }

            // Disconnect from parent if necessary, not cascading cost updates.
            if (vertexCopy->hasParent())
            {
                this->removeEdgeBetweenVertexAndParent(vertexCopy, false);
            }

            // Remove all children and let them know that their parent is pruned.
            VertexPtrVector children;
            vertexCopy->getChildren(&children);
            for (const auto &child : children)
            {
                // Remove this edge.
                vertexCopy->removeChild(child);
                child->removeParent(false);

                // If the child is inconsistent, it needs to be removed from the set of inconsistent vertices.
                if (!child->isConsistent())
                {
                    queuePtr_->removeFromInconsistentSet(child);
                }

                // If the child has outgoing edges in the queue, they need to be removed.
                queuePtr_->removeOutEdgesConnectedToVertexFromQueue(child);
            }

            // Remove any edges still in the queue.
            queuePtr_->removeAllEdgesConnectedToVertexFromQueue(vertexCopy);

            // Remove this vertex from the set of samples.
            samples_->remove(vertexCopy);

            // This state is now no longer considered a vertex, but could still be useful as sample.
            if (this->canSampleBePruned(vertexCopy))
            {
                // It's not even useful as sample, mark it as pruned.
                vertexCopy->markPruned();
                return {0, 1};  // The vertex is actually removed.
            }
            else
            {
                // It is useful as sample and should be recycled.
                recycleSample(vertexCopy);
                return {1, 0};  // The vertex is only disconnected and recycled as sample.
            }
        }

        void BITstar::ImplicitGraph::removeEdgeBetweenVertexAndParent(const VertexPtr &child, bool cascadeCostUpdates)
        {
#ifdef BITSTAR_DEBUG
            if (!child->hasParent())
            {
                throw ompl::Exception("An orphaned vertex has been passed for disconnection. Something went wrong.");
            }
#endif  // BITSTAR_DEBUG

            // Check if my parent has already been pruned. This can occur if we're cascading child disconnections.
            if (!child->getParent()->isPruned())
            {
                // If not, remove myself from my parent's vector of children, not updating down-stream costs
                child->getParent()->removeChild(child);
            }

            // Remove my parent link, cascading cost updates if requested:
            child->removeParent(cascadeCostUpdates);
        }

        /////////////////////////////////////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////////////////////////////////////
        // Private functions:
        void BITstar::ImplicitGraph::updateSamples(const VertexConstPtr &vertex)
        {
            // The required cost to contain the neighbourhood of this vertex.
            ompl::base::Cost requiredCost = this->calculateNeighbourhoodCost(vertex);

            // Check if we need to generate new samples inorder to completely cover the neighbourhood of the vertex
            if (costHelpPtr_->isCostBetterThan(sampledCost_, requiredCost))
            {
                // The total number of samples we wish to have.
                unsigned int numRequiredSamples = 0u;

                // Get the measure of what we're sampling.
                if (useJustInTimeSampling_)
                {
                    // Calculate the sample density given the number of samples per batch and the measure of this batch
                    // by assuming that this batch will fill the same measure as the previous.
                    double sampleDensity = static_cast<double>(numNewSamplesInCurrentBatch_) / approximationMeasure_;

                    // Convert that into the number of samples needed for this slice.
                    double numSamplesForSlice =
                        sampleDensity * sampler_->getInformedMeasure(sampledCost_, requiredCost);

                    // The integer of the double are definitely sampled
                    numRequiredSamples = numSamples_ + static_cast<unsigned int>(numSamplesForSlice);

                    // And the fractional part represents the probability of one more sample. I like being pedantic.
                    if (rng_.uniform01() <= (numSamplesForSlice - static_cast<double>(numRequiredSamples)))
                    {
                        // One more please.
                        ++numRequiredSamples;
                    }
                    // No else.
                }
                else
                {
                    // We're generating all our samples in one batch. Do it to it.
                    numRequiredSamples = numSamples_ + numNewSamplesInCurrentBatch_;
                }

                // Actually generate the new samples
                VertexPtrVector newStates{};
                newStates.reserve(numRequiredSamples);
                for (std::size_t tries = 0u;
                     tries < averageNumOfAllowedFailedAttemptsWhenSampling_ * numRequiredSamples &&
                     numSamples_ < numRequiredSamples;
                     ++tries)
                {
                    // Variable
                    // The new state:
                    auto newState =
                        std::make_shared<Vertex>(spaceInformation_, costHelpPtr_, queuePtr_, approximationId_);

                    // Sample in the interval [costSampled_, costReqd):
                    if (sampler_->sampleUniform(newState->state(), sampledCost_, requiredCost))
                    {
                        // If the state is collision free, add it to the set of free states
                        ++numStateCollisionChecks_;
                        if (spaceInformation_->isValid(newState->state()))
                        {
                            newStates.push_back(newState);

                            // Update the number of uniformly distributed states
                            ++numUniformStates_;

                            // Update the number of sample
                            ++numSamples_;
                        }
                        // No else
                    }
                }

                // Add the new state as a sample.
                this->addToSamples(newStates);

                // Record the sampled cost space
                sampledCost_ = requiredCost;
            }
            // No else, the samples are up to date
        }

        void BITstar::ImplicitGraph::updateVertexClosestToGoal()
        {
            if (static_cast<bool>(samples_))
            {
                // Get all samples as a vector.
                VertexPtrVector samples;
                samples_->list(samples);

                // Iterate through them testing which of the ones in the tree is the closest to the goal.
                for (const auto &sample : samples)
                {
                    if (sample->isInTree())
                    {
                        this->testClosestToGoal(sample);
                    }
                }
            }
            // No else, there are no vertices.
        }

        std::pair<unsigned int, unsigned int> BITstar::ImplicitGraph::pruneStartAndGoalVertices()
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
                    if (this->canVertexBeDisconnected(*startIter))
                    {
                        // It has, remove the start vertex DO NOT consider it as a sample. It is marked as a root node,
                        // so having it as a sample would cause all kinds of problems, also it shouldn't be possible for
                        // it to ever be useful as a sample anyway, unless there is a very weird cost function in play.
                        numPruned.second = numPruned.second + this->removeFromVertices(*startIter, false);

                        // Count as a disconnected vertex
                        ++numPruned.first;

                        // Disconnect from parent if necessary, cascading cost updates.
                        if ((*startIter)->hasParent())
                        {
                            this->removeEdgeBetweenVertexAndParent(*startIter, true);
                            queuePtr_->removeOutEdgesConnectedToVertexFromQueue(*startIter);
                        }

                        // // Remove it from the vertex queue.
                        // queuePtr_->unqueueVertex(*startIter);

                        // Store the start vertex in the pruned vector, in case it later needs to be readded:
                        prunedStartVertices_.push_back(*startIter);

                        // Remove this start from the vector.
                        // Swap it to the element before our *new* end
                        if (startIter != (startEnd - 1))
                        {
                            using std::swap;  // Enable Koenig Lookup.
                            swap(*startIter, *(startEnd - 1));
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
                    // Check if this goal has met the criteria to be pruned
                    if (this->canSampleBePruned(*goalIter))
                    {
                        // It has, remove the goal vertex completely
                        // Check if this vertex is in the tree
                        if ((*goalIter)->isInTree())
                        {
                            // Disconnect from parent if necessary, cascading cost updates.
                            if ((*goalIter)->hasParent())
                            {
                                this->removeEdgeBetweenVertexAndParent(*goalIter, true);
                                queuePtr_->removeOutEdgesConnectedToVertexFromQueue(*goalIter);

                                // If the goal is inconsistent, it needs to be removed from the set of inconsistent
                                // vertices.
                                if (!(*goalIter)->isConsistent())
                                {
                                    queuePtr_->removeFromInconsistentSet(*goalIter);
                                }
                            }

                            // Remove it from the set of vertices, recycling if necessary.
                            this->removeFromVertices(*goalIter, true);

                            // and as a vertex, allowing it to move to the set of samples.
                            numPruned.second = numPruned.second + this->removeFromVertices(*goalIter, true);

                            // Count it as a disconnected vertex
                            ++numPruned.first;
                        }
                        else
                        {
                            // It is not, so just it like a sample
                            this->pruneSample(*goalIter);

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

        std::pair<unsigned int, unsigned int> BITstar::ImplicitGraph::pruneSamples()
        {
            // The number of samples pruned in this pass:
            std::pair<unsigned int, unsigned int> numPruned{0u, 0u};

            // Get the vector of samples.
            VertexPtrVector samples;
            samples_->list(samples);

            // Are we dropping samples anytime we prune?
            if (dropSamplesOnPrune_)
            {
                std::size_t numFreeStatesPruned = 0u;
                for (const auto &sample : samples)
                {
                    if (!sample->isInTree())
                    {
                        this->pruneSample(sample);
                        ++numPruned.second;
                        ++numFreeStatesPruned;
                    }
                }

                // Store the number of uniform samples.
                numUniformStates_ = 0u;

                // Increase the global counter.
                numFreeStatesPruned_ += numFreeStatesPruned;
            }
            else
            {
                // Iterate through the vector and remove any samples that should not be in the graph.
                for (const auto &sample : samples)
                {
                    if (sample->isInTree())
                    {
                        if (this->canVertexBeDisconnected(sample))
                        {
                            numPruned = numPruned + this->pruneVertex(sample);
                        }
                    }
                    // Check if this state should be pruned.
                    else if (this->canSampleBePruned(sample))
                    {
                        // It should, remove the sample from the NN structure.
                        this->pruneSample(sample);

                        // Keep track of how many are pruned.
                        ++numPruned.second;
                    }
                    // No else, keep sample.
                }
            }

            return numPruned;
        }

        bool BITstar::ImplicitGraph::canVertexBeDisconnected(const VertexPtr &vertex) const
        {
            ASSERT_SETUP

            // Threshold should always be g_t(x_g)

            // Prune the vertex if it could cannot part of a better solution in the current graph.  Greater-than just in
            // case we're using an edge that is exactly optimally connected.
            // g_t(v) + h^(v) > g_t(x_g)?
            return costHelpPtr_->isCostWorseThan(costHelpPtr_->currentHeuristicVertex(vertex), solutionCost_);
        }

        bool BITstar::ImplicitGraph::canSampleBePruned(const VertexPtr &sample) const
        {
            ASSERT_SETUP

#ifdef BITSTAR_DEBUG
            if (sample->isPruned())
            {
                throw ompl::Exception("Asking whether a pruned sample can be pruned.");
            }
#endif  // BITSTAR_DEBUG

            // Threshold should always be g_t(x_g)
            // Prune the unconnected sample if it could never be better of a better solution.
            // g^(v) + h^(v) >= g_t(x_g)?
            return costHelpPtr_->isCostWorseThanOrEquivalentTo(costHelpPtr_->lowerBoundHeuristicVertex(sample),
                                                               solutionCost_);
        }

        void BITstar::ImplicitGraph::testClosestToGoal(const VertexConstPtr &vertex)
        {
            // Find the shortest distance between the given vertex and a goal
            double distance = std::numeric_limits<double>::max();
            problemDefinition_->getGoal()->isSatisfied(vertex->state(), &distance);

            // Compare to the current best approximate solution
            if (distance < closestDistanceToGoal_)
            {
                // Better, update the approximate solution.
                closestVertexToGoal_ = vertex;
                closestDistanceToGoal_ = distance;
            }
            // No else, don't update if worse.
        }

        ompl::base::Cost BITstar::ImplicitGraph::calculateNeighbourhoodCost(const VertexConstPtr &vertex) const
        {
#ifdef BITSTAR_DEBUG
            if (vertex->isPruned())
            {
                throw ompl::Exception("Calculating the neighbourhood cost of a pruned vertex.");
            }
#endif  // BITSTAR_DEBUG
        // Are we using JIT sampling?
            if (useJustInTimeSampling_)
            {
                // We are, return the maximum heuristic cost that represents a sample in the neighbourhood of the given
                // vertex.
                // There is no point generating samples worse the best solution (maxCost_) even if those samples are in
                // this vertex's neighbourhood.
                return costHelpPtr_->betterCost(
                    solutionCost_, costHelpPtr_->combineCosts(costHelpPtr_->lowerBoundHeuristicVertex(vertex),
                                                              ompl::base::Cost(2.0 * r_)));
            }

            // We are not, return the maximum cost we'd ever want to sample
            return solutionCost_;
        }

        void BITstar::ImplicitGraph::updateNearestTerms()
        {
            // The number of uniformly distributed states.
            unsigned int numUniformStates = 0u;

            // Calculate N, are we dropping samples?
            if (dropSamplesOnPrune_)
            {
                // We are, so we've been tracking the number of uniform states, just use that
                numUniformStates = numUniformStates_;
            }
            else if (isPruningEnabled_)
            {
                // We are not dropping samples but pruning is enabled, then all samples are uniform.
                numUniformStates = samples_->size();
            }
            else
            {
                // We don't prune, so we have to check how many samples are in the informed set.
                numUniformStates = computeNumberOfSamplesInInformedSet();
            }

            // If this is the first batch, we will be calculating the connection limits from only the starts and goals
            // for an RGG with m samples. That will be a complex graph. In this case, let us calculate the connection
            // limits considering the samples about to be generated. Doing so is equivalent to setting an upper-bound on
            // the radius, which RRT* does with it's min(maxEdgeLength, RGG-radius).
            if (numUniformStates == (startVertices_.size() + goalVertices_.size()))
            {
                numUniformStates = numUniformStates + numNewSamplesInCurrentBatch_;
            }

            // Now update the appropriate term
            if (useKNearest_)
            {
                k_ = this->calculateK(numUniformStates);
            }
            else
            {
                r_ = this->calculateR(numUniformStates);
            }
        }

        std::size_t BITstar::ImplicitGraph::computeNumberOfSamplesInInformedSet() const
        {
            // Get the samples as a vector.
            std::vector<VertexPtr> samples;
            samples_->list(samples);

            // Return the number of samples that can not be pruned.
            return std::count_if(samples.begin(), samples.end(),
                                 [this](const VertexPtr &sample) { return !canSampleBePruned(sample); });
        }

        double BITstar::ImplicitGraph::calculateR(unsigned int numUniformSamples) const
        {
            // Cast to double for readability. (?)
            auto stateDimension = static_cast<double>(spaceInformation_->getStateDimension());
            auto graphCardinality = static_cast<double>(numUniformSamples);

            // Calculate the term and return.
            return rewireFactor_ * this->calculateMinimumRggR() *
                   std::pow(std::log(graphCardinality) / graphCardinality, 1.0 / stateDimension);
        }

        unsigned int BITstar::ImplicitGraph::calculateK(unsigned int numUniformSamples) const
        {
            // Calculate the term and return
            return std::ceil(rewireFactor_ * k_rgg_ * std::log(static_cast<double>(numUniformSamples)));
        }

        double BITstar::ImplicitGraph::calculateMinimumRggR() const
        {
            // Variables
            // The dimension cast as a double for readibility;
            auto stateDimension = static_cast<double>(spaceInformation_->getStateDimension());

            // Calculate the term and return
            // RRT*
            return std::pow(2.0 * (1.0 + 1.0 / stateDimension) *
                                (approximationMeasure_ / unitNBallMeasure(spaceInformation_->getStateDimension())),
                            1.0 / stateDimension);

            // Relevant work on calculating the minimum radius:
            /*
            // PRM*,RRG radius (biggest for unit-volume problem)
            // See Theorem 34 in Karaman & Frazzoli IJRR 2011
            return 2.0 * std::pow((1.0 + 1.0 / dimDbl) *
                                  (approximationMeasure_ /
                                    unitNBallMeasure(si_->getStateDimension())),
                                  1.0 / dimDbl);

            // FMT* radius (R2: smallest, R3: equiv to RRT*, Higher d: middle).
            // See Theorem 4.1 in Janson et al. IJRR 2015
            return 2.0 * std::pow((1.0 / dimDbl) *
                                  (approximationMeasure_ /
                                    unitNBallMeasure(si_->getStateDimension())),
                                   1.0 / dimDbl);

            // RRT* radius (smallest for unit-volume problems above R3).
            // See Theorem 38 in Karaman & Frazzoli IJRR 2011
            return std::pow(2.0 * (1.0 + 1.0 / dimDbl) *
                                  (approximationMeasure_ /
                                    unitNBallMeasure(si_->getStateDimension())),
                            1.0 / dimDbl);
            */
        }

        double BITstar::ImplicitGraph::calculateMinimumRggK() const
        {
            // The dimension cast as a double for readibility.
            auto stateDimension = static_cast<double>(spaceInformation_->getStateDimension());

            // Calculate the term and return.
            return boost::math::constants::e<double>() +
                   (boost::math::constants::e<double>() / stateDimension);  // RRG k-nearest
        }

        void BITstar::ImplicitGraph::assertSetup() const
        {
            if (!isSetup_)
            {
                throw ompl::Exception("BITstar::ImplicitGraph was used before it was setup.");
            }
        }
        /////////////////////////////////////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////////////////////////////////////
        // Boring sets/gets (Public):
        bool BITstar::ImplicitGraph::hasAStart() const
        {
            return (!startVertices_.empty());
        }

        bool BITstar::ImplicitGraph::hasAGoal() const
        {
            return (!goalVertices_.empty());
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
            if (!findApprox_)
            {
                throw ompl::Exception("Approximate solutions are not being tracked.");
            }
#endif  // BITSTAR_DEBUG
            return closestVertexToGoal_;
        }

        double BITstar::ImplicitGraph::smallestDistanceToGoal() const
        {
#ifdef BITSTAR_DEBUG
            if (!findApprox_)
            {
                throw ompl::Exception("Approximate solutions are not being tracked.");
            }
#endif  // BITSTAR_DEBUG
            return closestDistanceToGoal_;
        }

        unsigned int BITstar::ImplicitGraph::getConnectivityK() const
        {
#ifdef BITSTAR_DEBUG
            if (!useKNearest_)
            {
                throw ompl::Exception("Using an r-disc graph.");
            }
#endif  // BITSTAR_DEBUG
            return k_;
        }

        double BITstar::ImplicitGraph::getConnectivityR() const
        {
#ifdef BITSTAR_DEBUG
            if (useKNearest_)
            {
                throw ompl::Exception("Using a k-nearest graph.");
            }
#endif  // BITSTAR_DEBUG
            return r_;
        }

        BITstar::VertexPtrVector BITstar::ImplicitGraph::getCopyOfSamples() const
        {
            VertexPtrVector samples;
            samples_->list(samples);
            return samples;
        }

        void BITstar::ImplicitGraph::setRewireFactor(double rewireFactor)
        {
            // Store
            rewireFactor_ = rewireFactor;

            // Check if there's things to update
            if (isSetup_)
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
            if (useKNearest && useJustInTimeSampling_)
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
                if (isSetup_)
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
            if (useKNearest_ && useJit)
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
                if (useJit)
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
            if (isSetup_)
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

        void BITstar::ImplicitGraph::setPruning(bool usePruning)
        {
            isPruningEnabled_ = usePruning;
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
                if (!findApprox_)
                {
                    // We're turning it off, clear the approximate solution variables:
                    closestDistanceToGoal_ = std::numeric_limits<double>::infinity();
                    closestVertexToGoal_.reset();
                }
                else
                {
                    // We are turning it on, do we have an exact solution?
                    if (!hasExactSolution_)
                    {
                        // We don't, find our current best approximate solution:
                        this->updateVertexClosestToGoal();
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

        void BITstar::ImplicitGraph::setAverageNumOfAllowedFailedAttemptsWhenSampling(std::size_t number)
        {
            averageNumOfAllowedFailedAttemptsWhenSampling_ = number;
        }

        std::size_t BITstar::ImplicitGraph::getAverageNumOfAllowedFailedAttemptsWhenSampling() const
        {
            return averageNumOfAllowedFailedAttemptsWhenSampling_;
        }

        template <template <typename T> class NN>
        void BITstar::ImplicitGraph::setNearestNeighbors()
        {
            // Check if the problem is already setup, if so, the NN structs have data in them and you can't really
            // change them:
            if (isSetup_)
            {
                OMPL_WARN("%s (ImplicitGraph): The nearest neighbour datastructures cannot be changed once the problem "
                          "is setup. Continuing to use the existing containers.",
                          nameFunc_().c_str());
            }
            else
            {
                // The problem isn't setup yet, create NN structs of the specified type:
                samples_ = std::make_shared<NN<VertexPtr>>();
            }
        }

        unsigned int BITstar::ImplicitGraph::numSamples() const
        {
            return samples_->size();
        }

        unsigned int BITstar::ImplicitGraph::numVertices() const
        {
            VertexPtrVector samples;
            samples_->list(samples);
            return std::count_if(samples.begin(), samples.end(),
                                 [](const VertexPtr &sample) { return sample->isInTree(); });
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
    }  // namespace geometric
}  // namespace ompl
