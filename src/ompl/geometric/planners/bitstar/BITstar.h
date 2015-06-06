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

#ifndef OMPL_GEOMETRIC_PLANNERS_BITSTAR_BITSTAR_
#define OMPL_GEOMETRIC_PLANNERS_BITSTAR_BITSTAR_

#include <ompl/config.h>
#if !OMPL_HAVE_EIGEN3
#error The BITstar class uses Eigen3, which was not detected at build time.
#endif

//STL:
//std::string
#include <string>
//std::pair
#include <utility>
//std::vector
#include <vector>

//OMPL:
//The base-class of planners:
#include "ompl/base/Planner.h"
//The nearest neighbours structure
#include "ompl/datastructures/NearestNeighbors.h"
//The informed sampler structure
#include "ompl/base/samplers/InformedStateSampler.h"
//Planner includes:
//#include "ompl/geometric/planners/PlannerIncludes.h"

//BIT*:
//The helper data classes, Vertex.h and IntegratedQueue.h are included *after* the declaration of the BITstar class as they are member classes of BITstar.



namespace ompl
{
    namespace geometric
    {
        /**
            @anchor gBITstar
            @par Short description
            BIT* (Batch Informed Trees) is an anytime asymptotically optimal sampling-based
            motion planning algorithm that extends Lifelong Planning A* (LPA*) techniques to continuous planning
            problems. BIT* accomplishes this by processing batches of samples with a heuristic.
            In doing so, it strikes a balance between algorithms like RRT* and FMT*.

            @par Associated publications:

            J.D. Gammell, S. S. Srinivasa, T.D. Barfoot, "BIT*: Batch Informed Trees."
            In Proceedings of the Information-based Grasp and Manipulation Planning Workshop at Robotics: Science and Systems (RSS).
            Berkeley, CA, USA, 13 July 2014.
            <a href="http://asrl.utias.utoronto.ca/~tdb/bib/gammell_rss14.pdf">Extended Abstract</a>.
            <a href="http://asrl.utias.utoronto.ca/~tdb/bib/gammell_rss14_poster.pdf">Poster</a>.

            J D. Gammell, S. S. Srinivasa, T. D. Barfoot, "Batch Informed Trees (BIT*): Sampling-based Optimal Planning via the Heuristically Guided Search of Implicit Random Geometric Graphs,"
            In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA).
            Seattle, Washington, USA, 26-30 May 2015.
            <a href="http://arxiv.org/abs/1405.5848">arXiv:1405.5848 [cs.RO]</a>.
            <a href="http://www.youtube.com/watch?v=MRzSfLpNBmA">Illustration video</a>.

            \todo
            - Make k-nearest correct.
            - Extend beyond single goal states to other samplable goals (i.e., goal sets).
            - Generalize heuristics to make proper use of the optimization class.
        */
        /** \brief Batch Informed Trees (BIT*)*/
        class BITstar : public ompl::base::Planner
        {
        public:
            //Forward declarations so that the classes belong to BIT*:
            /** \brief The vertex of implicit and explicit graphs. */
            class Vertex;
            /** \brief A generator of unique vertex IDs. */
            class IdGenerator;
            /** \brief The queue of edges to process as an integrated dual-stage queue (tracks both the expansion of vertices and the resulting edges) */
            class IntegratedQueue;
            //Helpful typedefs:
            /** \brief A vertex shared pointer. */
            typedef boost::shared_ptr<Vertex> VertexPtr;
            /** \brief A \e constant vertex shared pointer. */
            typedef boost::shared_ptr<const Vertex> VertexConstPtr;
            /** \brief A vertex weak pointer. */
            typedef boost::weak_ptr<Vertex> VertexWeakPtr;
            /** \brief An integrated queue shared pointer. */
            typedef boost::shared_ptr<IntegratedQueue> IntegratedQueuePtr;
            /** \brief The vertex id type */
            typedef unsigned int VertexId;

            /** \brief Construct! */
            BITstar(const base::SpaceInformationPtr& si, const std::string& name = "BITstar");

            /** \brief Destruct! */
            virtual ~BITstar();

            /** \brief Setup */
            virtual void setup();

            /** \brief Clear */
            virtual void clear();

            /** \brief Solve */
            base::PlannerStatus solve(const base::PlannerTerminationCondition& ptc);

            /** \brief Get results */
            virtual void getPlannerData(base::PlannerData& data) const;

            /** \brief Get the next edge to be processed. Causes vertices in the queue to be expanded (if necessary) and therefore effects the run timings of the algorithm, but helpful for some videos and debugging. */
            std::pair<const ompl::base::State*, const ompl::base::State*> getNextEdgeInQueue();

            /** \brief Get the value of the next edge to be processed. Causes vertices in the queue to be expanded (if necessary) and therefore effects the run timings of the algorithm, but helpful for some videos and debugging. */
            ompl::base::Cost getNextEdgeValueInQueue();

            /** \brief Get the whole messy set of edges in the queue. Expensive but helpful for some videos */
            void getEdgeQueue(std::vector<std::pair<VertexConstPtr, VertexConstPtr> >* edgesInQueue);

            /** \brief Get the whole set of vertices to be expanded. Expensive but helpful for some videos */
            void getVertexQueue(std::vector<VertexConstPtr>* verticesInQueue);

            /** \brief Set a different nearest neighbours datastructure */
            template<template<typename T> class NN>
            void setNearestNeighbors();
            ///////////////////////////////////////
            // Planner settings:
            /** \brief Set the rewiring scale factor, s, such that r_rrg = s \times r_rrg* */
            void setRewireFactor(double rewireFactor);

            /** \brief Get the rewiring scale factor. */
            double getRewireFactor() const;

            /** \brief Set the number of samplers per batch. */
            void setSamplesPerBatch(unsigned int n);

            /** \brief Get the number of samplers per batch. */
            unsigned int getSamplesPerBatch() const;

            /** \brief Enable a k-nearest search for instead of an r-disc search. */
            void setKNearest(bool useKNearest);

            /** \brief Get whether a k-nearest search is being used.*/
            bool getKNearest() const;

            /** \brief Enable tracking of failed edges. This currently is too expensive to be useful.*/
            void setUseFailureTracking(bool trackFailures);

            /** \brief Get whether a failed edge list is in use.*/
            bool getUseFailureTracking() const;

            /** \brief Enable "strict sorting" of the edge queue.
            Rewirings can change the position in the queue of an edge.
            When strict sorting is enabled, the effected edges are resorted
            immediately, while disabling strict sorting delays this
            resorting until the end of the batch. */
            void setStrictQueueOrdering(bool beStrict);

            /** \brief Get whether strict queue ordering is in use*/
            bool getStrictQueueOrdering() const;

            /** \brief Enable pruning of vertices/samples that CANNOT improve the current solution.
            When a vertex in the graph is pruned, it's descendents are also pruned
            (if they also cannot improve the solution) or placed back in
            the set of free samples (if they could improve the solution).
            This assures that a uniform density is maintained.*/
            void setPruning(bool prune);

            /** \brief Get whether graph and sample pruning is in use.*/
            bool getPruning() const;

            /** \brief Set the fractional change in the solution cost necessary for pruning to occur. */
            void setPruneThresholdFraction(double fractionalChange);

            /** \brief Get the fractional change in the solution cost necessary for pruning to occur. */
            double getPruneThresholdFraction() const;

            /** \brief Delay considering rewiring edges until an initial solution is found. This improves
            the time required to find an initial solution when doing so requires multiple batches and has
            no effects on theoretical asymptotic optimality (as the rewiring edges are eventually considered). */
            void setDelayRewiringUntilInitialSolution(bool delayRewiring);

            /** \brief Get whether BIT* is delaying rewiring until a solution is found. */
            bool getDelayRewiringUntilInitialSolution() const;

            /** \brief Stop the planner each time a solution improvement is found. Useful
            for examining the intermediate solutions found by BIT*. */
            void setStopOnSolnImprovement(bool stopOnChange);

            /** \brief Get whether BIT* stops each time a solution is found. */
            bool getStopOnSolnImprovement() const;
            ///////////////////////////////////////

        protected:
            //Everything is only protected so we can create modifications without duplicating code by deriving from the class:

            //Typedefs:
            /** \brief A pair of vertices, i.e., an edge. */
            typedef std::pair<VertexPtr, VertexPtr> VertexPtrPair;

            /** \brief The OMPL::NearestNeighbors structure. */
            typedef boost::shared_ptr< NearestNeighbors<VertexPtr> > VertexPtrNNPtr;

            //Functions:
            /** \brief A debug function: Estimate the measure of the free/obstace space via sampling. */
            void estimateMeasures();

            ///////////////////////////////////////////////////////////////////
            //BIT* primitives:
            /** \brief A single iteration */
            virtual void iterate();

            /** \brief Initialize variables for a new batch */
            void newBatch();

            /** \brief Update the list of free samples */
            void updateSamples(const VertexPtr& vertex);

            /** \brief Prune the problem. Returns true if pruning was done. */
            virtual bool prune();

            /** \brief Resort the queue. Returns true if any pruning was done. */
            virtual bool resort();

            /** \brief Publish the found solution to the ProblemDefinition*/
            void publishSolution();
            ///////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////////////////////////
            //Helper functions for data manipulation and other low-level functions
            /** \brief Prune all samples with a solution heuristic that is not less than the bestCost_ */
            void pruneSamples();

            /** \brief Checks an edge for collision. A wrapper to SpaceInformation->checkMotion that tracks number of collision checks. */
            bool checkEdge(const VertexPtrPair& edge);

            /** \brief Actually remove a sample from its NN struct: */
            void dropSample(VertexPtr oldSample);

            /** \brief Add an edge from the edge queue to the tree. Will add the state to the vertex queue if it's new to the tree or otherwise replace the parent. Updates solution information if the solution improves. */
            void addEdge(const VertexPtrPair& newEdge, const ompl::base::Cost& edgeCost, const bool& removeFromFree, const bool& updateDescendants);

            /** \brief Replace the parent edge with the given new edge and cost */
            void replaceParent(const VertexPtrPair& newEdge, const ompl::base::Cost& edgeCost, const bool& updateDescendants);

            /** \brief The special work that needs to be done to update the goal vertex is the solution has changed. */
            void updateGoalVertex();

            /** \brief Add a sample */
            void addSample(const VertexPtr& newSample);

            /** \brief Add a vertex to the graph */
            void addVertex(const VertexPtr& newVertex, const bool& removeFromFree);

            /** \brief Get the nearest samples from the freeStateNN_ using the appropriate "near" definition (i.e., k or r). */
            void nearestSamples(const VertexPtr& vertex, std::vector<VertexPtr>* neighbourSamples);

            /** \brief Get the nearest samples from the vertexNN_ using the appropriate "near" definition (i.e., k or r). */
            void nearestVertices(const VertexPtr& vertex, std::vector<VertexPtr>* neighbourVertices);
            ///////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////////////////////////
            //Helper functions for sorting queues/nearest-neighbour structures and the related calculations.
            /** \brief The distance function used for nearest neighbours. Calculates the distance directionally from the given state to all the other states (can be used on states either in our out of the graph).*/
            double nnDistance(const VertexPtr& a, const VertexPtr& b) const;
            ///////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////////////////////////
            //Helper functions for various heuristics.
            /** \brief Calculates a heuristic estimate of the cost of a solution constrained to pass through a vertex, independent of the current cost-to-come. I.e., combines the heuristic estimates of the cost-to-come and cost-to-go. */
            ompl::base::Cost lowerBoundHeuristicVertex(const VertexPtr& edgePair) const;

            /** \brief Calculates a heuristic estimate of the cost of a solution constrained to pass through a vertex, dependent on the current cost-to-come. I.e., combines the current cost-to-come with a heuristic estimate of the cost-to-go. */
            ompl::base::Cost currentHeuristicVertex(const VertexPtr& edgePair) const;

            /** \brief Calculates a heuristic estimate of the cost of a solution constrained to go through an edge, independent of the cost-to-come of the parent state. I.e., combines the heuristic estimates of the cost-to-come, edge cost, and cost-to-go. */
            ompl::base::Cost lowerBoundHeuristicEdge(const VertexPtrPair& edgePair) const;

            /** \brief Calculates a heuristic estimate of the cost of a solution constrained to go through an edge, dependent on the cost-to-come of the parent state. I.e., combines the current cost-to-come with heuristic estimates of the edge cost, and cost-to-go. */
            ompl::base::Cost currentHeuristicEdge(const VertexPtrPair& edgePair) const;

            /** \brief Calculates a heuristic estimate of the cost of a path to the \e target of an edge, dependent on the cost-to-come of the parent state. I.e., combines the current cost-to-come with heuristic estimates of the edge cost. */
            ompl::base::Cost currentHeuristicEdgeTarget(const VertexPtrPair& edgePair) const;

            /** \brief Calculate a heuristic estimate of the cost-to-come for a Vertex */
            ompl::base::Cost costToComeHeuristic(const VertexPtr& vertex) const;

            /** \brief Calculate a heuristic estimate of the cost an edge between two Vertices */
            ompl::base::Cost edgeCostHeuristic(const VertexPtrPair& edgePair) const;

            /** \brief Calculate a heuristic estimate of the cost-to-go for a Vertex */
            ompl::base::Cost costToGoHeuristic(const VertexPtr& vertex) const;

            /** \brief The true cost of an edge, including collisions.*/
            ompl::base::Cost trueEdgeCost(const VertexPtrPair& edgePair) const;

            /** \brief Calculate the max req'd cost to define a neighbourhood around a state. I.e., For path-length problems, the cost equivalent of +2*r. */
            ompl::base::Cost neighbourhoodCost() const;

            /** \brief Compare whether cost a is better than cost b. Ignores the tolerances used by OptimizationObjective::isCostBetterThan */
            bool isCostBetterThan(const ompl::base::Cost& a, const ompl::base::Cost& b) const;

            /** \brief Compare whether cost a is worse than cost b by checking whether b is better than a. */
            bool isCostWorseThan(const ompl::base::Cost& a, const ompl::base::Cost& b) const;

            /** \brief Compare whether cost a and cost b are equivalent by checking that neither a or b is better than the other. */
            bool isCostEquivalentTo(const ompl::base::Cost& a, const ompl::base::Cost& b) const;

            /** \brief Compare whether cost a and cost b are not equivalent by checking if either a or b is better than the other. */
            bool isCostNotEquivalentTo(const ompl::base::Cost& a, const ompl::base::Cost& b) const;

            /** \brief Compare whether cost a is better or equivalent to cost b by checking that b is not better than a. */
            bool isCostBetterThanOrEquivalentTo(const ompl::base::Cost& a, const ompl::base::Cost& b) const;

            /** \brief Compare whether cost a is worse or equivalent to cost b by checking that a is not better than b. */
            bool isCostWorseThanOrEquivalentTo(const ompl::base::Cost& a, const ompl::base::Cost& b) const;

            /** \brief Returns whether the cost is finite or not. By default calls std::isfinite on Cost::value(). */
            bool isFinite(const ompl::base::Cost& cost) const;

            /** \brief The better of two costs.*/
            ompl::base::Cost betterCost(const ompl::base::Cost& a, const ompl::base::Cost& b) const;

            /** \brief Combine 3 costs */
            ompl::base::Cost combineCosts(const ompl::base::Cost& a, const ompl::base::Cost& b, const ompl::base::Cost& c) const;

            /** \brief Combine 4 costs */
            ompl::base::Cost combineCosts(const ompl::base::Cost& a, const ompl::base::Cost& b, const ompl::base::Cost& c, const ompl::base::Cost& d) const;

            /** \brief Calculate the fractional change of cost "newCost" from "oldCost" relative to "oldCost", i.e., (newCost - oldCost)/oldCost. */
            double fractionalChange(const ompl::base::Cost& newCost, const ompl::base::Cost& oldCost) const;

            /** \brief Calculate the fractional change of cost "newCost" from "oldCost" relative to "refCost", i.e., (newCost - oldCost)/refCost. */
            double fractionalChange(const ompl::base::Cost& newCost, const ompl::base::Cost& oldCost, const ompl::base::Cost& refCost) const;
            ///////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////////////////////////
            //Helper functions to calculate parameters:
            /** \brief Initialize the nearest-neighbour terms */
            void initializeNearestTerms();

            /** \brief Update the appropriate nearest-neighbour terms, r_ and k_ */
            virtual void updateNearestTerms();

            /** \brief Calculate the r for r-disc nearest neighbours, a function of the current graph */
            double calculateR(unsigned int N) const;

            /** \brief Calculate the k for k-nearest neighours, a function of the current graph */
            unsigned int calculateK(unsigned int N) const;

            /** \brief Calculate the lower-bounding radius RGG term for asymptotic almost-sure convergence to the optimal path (i.e., r_rrg* in Karaman and Frazzoli IJRR 11). This is a function of the size of the problem domain. */
            double minimumRggR() const;

            /** \brief Calculate the lower-bounding k-nearest RGG term for asymptotic almost-sure convergence to the optimal path (i.e., k_rrg* in Karaman and Frazzoli IJRR 11). This is a function of the state dimension and is left as a double for later accuracy in calculate k */
            double minimumRggK() const;
            ///////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////////////////////////
            //Helper functions for logging
            /** \brief The message printed when a goal is found/improved */
            virtual void goalMessage() const;

            /** \brief The message printed when solve finishes successfully */
            virtual void endSuccessMessage() const;

            /** \brief The message printed when solve finishes unsuccessfully */
            virtual void endFailureMessage() const;

            /** \brief A debug-level status message for debugging. */
            virtual void statusMessage(const ompl::msg::LogLevel& msgLevel, const std::string& status) const;
            ///////////////////////////////////////////////////////////////////

            ///////////////////////////////////////
            // Planner progress property functions
            /** \brief Retrieve the best exact-solution cost found
            as the raw data. (bestCost_) */
            ompl::base::Cost bestCost() const;
            /** \brief Retrieve the best exact-solution cost found
            as a planner-progress property. (bestCost_) */
            std::string bestCostProgressProperty() const;

            /** \brief Retrieve the length of the best exact-solution found
            as a planner-progress property. (bestLength_) */
            std::string bestLengthProgressProperty() const;

            /** \brief Retrieve the current number of free samples
            as a planner-progress property. (size of freeStateNN_) */
            std::string currentFreeProgressProperty() const;

            /** \brief Retrieve the current number of vertices in the graph
            as a planner-progress property. (Size of vertexNN_) */
            std::string currentVertexProgressProperty() const;

            /** \brief Retrieve the current number of vertices in the expansion queue
            as a planner-progress property. (The position of the vertex subqueue of intQueue_) */
            std::string vertexQueueSizeProgressProperty() const;

            /** \brief Retrieve the current number of edges in the search queue
            as a planner-progress property. (The size of the edge subqueue of intQueue_) */
            std::string edgeQueueSizeProgressProperty() const;

            /** \brief Retrieve the number of iterations
            as a planner-progress property. (numIterations_) */
            std::string iterationProgressProperty() const;

            /** \brief Retrieve the number of batches processed
            as the raw data. (numBatches_) */
            unsigned int numBatches() const;
            /** \brief Retrieve the number of batches processed
            as a planner-progress property. (numBatches_) */
            std::string batchesProgressProperty() const;

            /** \brief Retrieve the number of graph prunings performed
            as a planner-progress property. (numPrunings_) */
            std::string pruningProgressProperty() const;

            /** \brief Retrieve the \e total number of states generated
            as a planner-progress property. (numSamples_) */
            virtual std::string totalStatesCreatedProgressProperty() const;

            /** \brief Retrieve the \e total number of vertices added to the graph
            as a planner-progress property. (numVertices_) */
            std::string verticesConstructedProgressProperty() const;

            /** \brief Retrieve the number of states pruned from the problem
            as a planner-progress property. (numFreeStatesPruned_) */
            std::string statesPrunedProgressProperty() const;

            /** \brief Retrieve the number of graph vertices that are disconnected and
            either returned to the set of free samples or deleted completely
            as a planner-progress property. (numVerticesDisconnected_) */
            std::string verticesDisconnectedProgressProperty() const;

            /** \brief Retrieve the number of global-search edges that rewired the graph
            as a planner-progress property. (numRewirings_) */
            std::string rewiringProgressProperty() const;

            /** \brief Retrieve the number of state collisions checks (i.e., calls to SpaceInformation::isValid(...))
            as a planner-progress property. (numStateCollisionChecks_) */
            std::string stateCollisionCheckProgressProperty() const;

            /** \brief Retrieve the number of edge (or motion) collision checks (i.e., calls to SpaceInformation::checkMotion(...))
            as a planner-progress property. (numEdgeCollisionChecks_) */
            std::string edgeCollisionCheckProgressProperty() const;

            /** \brief Retrieve the number of nearest neighbour calls (i.e., NearestNeighbors<T>::nearestK(...) or NearestNeighbors<T>::nearestR(...))
            as a planner-progress property. (numNearestNeighbours_) */
            std::string nearestNeighbourProgressProperty() const;
            ///////////////////////////////////////



            //Variables -- Make sure every one is configured in setup() and reset in clear():
            /** \brief State sampler */
            ompl::base::InformedSamplerPtr                           sampler_;

            /** \brief Optimization objective copied from ProblemDefinition */
            ompl::base::OptimizationObjectivePtr                     opt_;

            /** \brief The start of the problem as a vertex*/
            VertexPtr                                                startVertex_;

            /** \brief The goal of the problem as a vertex*/
            VertexPtr                                                goalVertex_;

            /** \brief The unconnected samples as a nearest-neighbours datastructure. Sorted by nnDistance. Size accessible via currentFreeProgressProperty */
            VertexPtrNNPtr                                          freeStateNN_;

            /** \brief The vertices as a nearest-neighbours data structure. Sorted by nnDistance. Size accessible via currentVertexProgressProperty */
            VertexPtrNNPtr                                          vertexNN_;

            /** \brief The integrated queue of vertices to expand and edges to process ordered on "f-value", i.e., estimated solution cost. Remaining vertex queue "size" and edge queue size are accessible via vertexQueueSizeProgressProperty and edgeQueueSizeProgressProperty, respectively. */
            IntegratedQueuePtr                                       intQueue_;

            /** \brief The resulting sampling density for a batch */
            double                                                   sampleDensity_;

            /** \brief The current r-disc RGG connection radius */
            double                                                   r_;

            /** \brief The minimum k-nearest RGG connection term. Only a function of state dimension, so can be calculated once. Left as a double for later accuracy in calculate k */
            double                                                   k_rgg_;

            /** \brief The current k-nearest RGG connection number */
            unsigned int                                            k_;

            /** \brief The best cost found to date. This is the maximum total-heuristic cost of samples we'll consider. Accessible via bestCostProgressProperty */
            ompl::base::Cost                                         bestCost_;

            /** \brief The number of vertices in the best solution found to date. Accessible via bestLengthProgressProperty */
            unsigned int                                             bestLength_;

            /** \brief The cost to which the graph has been pruned. We will only prune the graph if bestCost_ is less than this value. */
            ompl::base::Cost                                         prunedCost_;

            /** \brief The measure of the problem domain when we pruned the graph. */
            double                                                   prunedMeasure_;

            /** \brief The minimum possible solution cost. I.e., the heuristic value of the goal. */
            ompl::base::Cost                                         minCost_;

            /** \brief The total-heuristic cost up to which we've sampled */
            ompl::base::Cost                                         costSampled_;

            /** \brief If we've found a solution yet */
            bool                                                     hasSolution_;

            /** \brief A manual stop on the solve loop */
            bool                                                     stopLoop_;

            ///////////////////////////////////////

            ///////////////////////////////////////
            //Informational variables - Make sure initialized in setup and reset in clear
            /** \brief If the solution is approximate */
            bool                                                     approximateSoln_;

            /** \brief The distance of the approximate solution, set to -1.0 for non approximate solutions */
            double                                                   approximateDiff_;

            /** \brief The number of iterations run. Accessible via iterationProgressProperty */
            unsigned int                                             numIterations_;

            /** \brief The number of batches processed. Accessible via batchesProgressProperty */
            unsigned int                                             numBatches_;

            /** \brief The number of times the graph/samples have been pruned. Accessible via pruningProgressProperty */
            unsigned int                                             numPrunings_;

            /** \brief The number of states generated through sampling. Accessible via statesFromSamplingProgressProperty */
            unsigned int                                             numSamples_;

            /** \brief The number of vertices generated through smoothing/shortcutting. Accessible via statesFromSmoothingProgressProperty */
            unsigned int                                             numSmoothedVertices_;

            /** \brief The number of vertices ever added to the graph. Will count vertices twice if they spend any time disconnected. Accessible via verticesConstructedProgressProperty */
            unsigned int                                             numVertices_;

            /** \brief The number of free states that have been pruned. Accessible via statesPrunedProgressProperty */
            unsigned int                                             numFreeStatesPruned_;

            /** \brief The number of graph vertices that get disconnected. These either return to being free samples or are pruned completely. Accessible via verticesDisconnectedProgressProperty */
            unsigned int                                             numVerticesDisconnected_;

            /** \brief The number of times a state in the graph was rewired. Accessible via rewiringProgressProperty */
            unsigned int                                             numRewirings_;

            /** \brief The number of state collision checks. Accessible via stateCollisionCheckProgressProperty */
            unsigned int                                             numStateCollisionChecks_;

            /** \brief The number of edge collision checks. Accessible via edgeCollisionCheckProgressProperty */
            unsigned int                                             numEdgeCollisionChecks_;

            /** \brief The number of nearest neighbour calls. Accessible via nearestNeighbourProgressProperty */
            unsigned int                                             numNearestNeighbours_;
            ///////////////////////////////////////

            ///////////////////////////////////////
            //Parameters - Set defaults in construction/setup and DO NOT reset in clear.
            /** \brief Whether to use a strict-queue ordering (param) */
            bool                                                     useStrictQueueOrdering_;

            /** \brief The rewiring factor, s, so that r_rrg = s \times r_rrg* > r_rrg* (param) */
            double                                                   rewireFactor_;

            /** \brief The number of samples per batch (param) */
            unsigned int                                             samplesPerBatch_;

            /** \brief Track edges that have been checked and failed so they never reenter the queue. (param) */
            bool                                                     useFailureTracking_;

            /** \brief Option to use k-nearest search for rewiring (param) */
            bool                                                     useKNearest_;

            /** \brief Whether to use graph pruning (param) */
            bool                                                     usePruning_;

            /** \brief The fractional decrease in solution cost required to trigger pruning (param) */
            double                                                   pruneFraction_;

            /** \brief Whether to delay rewiring until a solution is found (param) */
            bool                                                     delayRewiring_;

            /** \brief Whether to stop the planner as soon as the path changes (param) */
            bool                                                     stopOnSolnChange_;
            ///////////////////////////////////////
        }; //class: BITstar
    } //geometric
} //ompl


//BIT* Includes:
//The Vertex ID generator class
#include "ompl/geometric/planners/bitstar/datastructures/IdGenerator.h"
//My vertex class:
#include "ompl/geometric/planners/bitstar/datastructures/Vertex.h"
//My queue class
#include "ompl/geometric/planners/bitstar/datastructures/IntegratedQueue.h"

#endif //OMPL_GEOMETRIC_PLANNERS_BITSTAR_BITSTAR_
