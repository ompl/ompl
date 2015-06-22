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

// STL:
// std::string
#include <string>
// std::pair
#include <utility>
// std::vector
#include <vector>
// std::list
#include <list>

// OMPL:
// The base-class of planners:
#include "ompl/base/Planner.h"
// The nearest neighbours structure
#include "ompl/datastructures/NearestNeighbors.h"
// The informed sampler structure
#include "ompl/base/samplers/InformedStateSampler.h"
// Planner includes:
//#include "ompl/geometric/planners/PlannerIncludes.h"

// BIT*:
// The helper data classes, Vertex.h, CostHelper.h and IntegratedQueue.h are included *after* the declaration of the
// BITstar class as
// they are member classes of BITstar.

namespace ompl
{
    namespace geometric
    {
        /**
            @anchor gBITstar

            \ref gBITstar "BIT*" (Batch Informed Trees) is an \e anytime asymptotically optimal sampling-based
            planning algorithm. It approaches problems by assuming that a \e simple solution exists and only
            goes onto consider \e complex solutions when that proves incorrect. It accomplishes this by using
            heuristics to search in order of decreasing potential solution quality.

            Both a k-nearest and r-disc version are available, with the k-nearest selected by default. In general,
            the r-disc variant considers more connections than the k-nearest. For a small number of specific planning
            problems, this results in it finding solutions slower than k-nearest (hence the default choice).
            It is recommended that you try both variants, with the r-disc version being recommended *if* it finds an
            initial solution in a suitable amount of time (which it probably will). The difference in this number of
            connections considered is a RGG theory question, and certainly merits further review.

            This implementation of BIT* can handle multiple starts, multiple goals, a variety of optimization objectives
            (e.g., path length), and with \ref gBITstarSetJustInTimeSampling "just-in-time sampling", infinite problem
           domains.
            Note that for some of optimization  objectives, the user must specify a suitable heuristic and that when
            this heuristic is not specified, it will use the conservative/always admissible \e zero-heuristic.

            This implementation also includes some new advancements, including the ability to prioritize exploration
           until an
            initial solution is found (\ref gBITstarSetDelayRewiringUntilInitialSolution "Delayed rewiring"), the
           ability to generate
            samples only when necessary (\ref gBITstarSetJustInTimeSampling "Just-in-time sampling"), and the ability to
           periodically
            remove samples that have yet to be connected to the graph (\ref gBITstarSetDropSamplesOnPrune "Sample
           dropping"). With
            just-in-time sampling, BIT* can even solve planning problems with infinite state space boundaries, i.e.,
           (-inf, inf).


            @par Associated publication:

            J. D. Gammell, S. S. Srinivasa, T. D. Barfoot, "Batch Informed Trees (BIT*): Sampling-based
            Optimal Planning via the Heuristically Guided Search of Implicit Random Geometric Graphs,"
            In Proceedings of the IEEE International Conference on Robotics and Automation (ICRA).
            Seattle, WA, USA, 26-30 May 2015.
            DOI: <a href="http://dx.doi.org/10.1109/ICRA.2015.7139620">10.1109/ICRA.2015.7139620</a>.
            <a href="http://www.youtube.com/watch?v=MRzSfLpNBmA">Illustration video</a>.
        */
        /** \brief Batch Informed Trees (BIT*)*/
        class BITstar : public ompl::base::Planner
        {
        public:
            // Forward declarations so that the classes belong to BIT*:
            /** \brief The vertex of implicit and explicit graphs. */
            class Vertex;
            /** \brief A generator of unique vertex IDs. */
            class IdGenerator;
            /** \brief A helper class to consolidate cost and heuristic calculations. */
            class CostHelper;
            /** \brief The queue of edges to process as an integrated dual-stage queue (tracks both the expansion of
             * vertices and the resulting edges) */
            class IntegratedQueue;
            // Helpful alias declarations:
            /** \brief A vertex shared pointer. */
            typedef std::shared_ptr<Vertex> VertexPtr;
            /** \brief A \e constant vertex shared pointer. */
            typedef std::shared_ptr<const Vertex> VertexConstPtr;
            /** \brief A vertex weak pointer. */
            typedef std::weak_ptr<Vertex> VertexWeakPtr;
            /** \brief A vector of shared pointers. */
            typedef std::vector<VertexPtr> VertexPtrVector;
            /** \brief A vector of shared const pointers. */
            typedef std::vector<VertexConstPtr> VertexConstPtrVector;
            /** \brief A list of shared pointers. */
            typedef std::list<VertexPtr> VertexPtrList;
            /** \brief An integrated queue shared pointer. */
            typedef std::shared_ptr<IntegratedQueue> IntegratedQueuePtr;
            /** \brief The vertex id type */
            typedef unsigned int VertexId;
            /** \brief A pair of vertices, i.e., an edge. */
            typedef std::pair<VertexPtr, VertexPtr> VertexPtrPair;
            /** \brief A pair of const vertices, i.e., an edge. */
            typedef std::pair<VertexConstPtr, VertexConstPtr> VertexConstPtrPair;
            /** \brief A vector of pairs of vertices, i.e., a vector of edges. */
            typedef std::vector<VertexPtrPair> VertexPtrPairVector;
            /** \brief A vector of pairs of const vertices, i.e., a vector of edges. */
            typedef std::vector<VertexConstPtrPair> VertexConstPtrPairVector;
            /** \brief The OMPL::NearestNeighbors structure. */
            typedef std::shared_ptr<NearestNeighbors<VertexPtr>> VertexPtrNNPtr;

            /** \brief Construct! */
            BITstar(const base::SpaceInformationPtr &si, const std::string &name = "BITstar");

            /** \brief Destruct! */
            ~BITstar() override;

            /** \brief Setup */
            void setup() override;

            /** \brief Clear */
            void clear() override;

            /** \brief Solve */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            /** \brief Get results */
            void getPlannerData(base::PlannerData &data) const override;

            ///////////////////////////////////////
            // Planner info for debugging, etc:
            /** \brief Get the next edge to be processed. Causes vertices in the queue to be expanded (if necessary) and
             * therefore effects the run timings of the algorithm, but helpful for some videos and debugging. */
            std::pair<const ompl::base::State *, const ompl::base::State *> getNextEdgeInQueue();

            /** \brief Get the value of the next edge to be processed. Causes vertices in the queue to be expanded (if
             * necessary) and therefore effects the run timings of the algorithm, but helpful for some videos and
             * debugging. */
            ompl::base::Cost getNextEdgeValueInQueue();

            /** \brief Get the whole messy set of edges in the queue. Expensive but helpful for some videos */
            void getEdgeQueue(VertexConstPtrPairVector *edgesInQueue);

            /** \brief Get the whole set of vertices to be expanded. Expensive but helpful for some videos */
            void getVertexQueue(VertexConstPtrVector *verticesInQueue);

            /** \brief Get the number of iterations completed */
            unsigned int numIterations() const;

            /** \brief Retrieve the best exact-solution cost found.*/
            ompl::base::Cost bestCost() const;
            ///////////////////////////////////////

            ///////////////////////////////////////
            // Planner settings:
            /** \brief Set a different nearest neighbours datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors();

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

            /** @anchor gBITstarSetDelayRewiringUntilInitialSolution \brief Delay the consideration of rewiring edges
            until
            an initial solution is found. When multiple batches are required to find an initial solution, this can
            improve the time
            required to do so, by delaying improvements in the cost-to-come to a connected vertex. As the rewiring edges
            are considered
            once an initial solution is found, this has no effect on the theoretical asymptotic optimality of the
            planner. */
            void setDelayRewiringUntilInitialSolution(bool delayRewiring);

            /** \brief Get whether BIT* is delaying rewiring until a solution is found. */
            bool getDelayRewiringUntilInitialSolution() const;

            /** @anchor gBITstarSetJustInTimeSampling \brief Delay the generation of samples until they are \e
            necessary. This only works when using an
            r-disc connection scheme, and is currently only implemented for problems seeking to minimize path length.
            This helps reduce the complexity of
            nearest-neighbour look ups, and can be particularly beneficial in unbounded planning problems where
            selecting an appropriate bounding box is difficult.
            With JIT sampling enabled, BIT* can solve planning problems whose state space has \e infinite \e boundaries.
            When enumerating outgoing edges from
            a vertex, BIT* uses JIT sampling to assure that the area within r of the vertex has been sampled during this
            batch. This is done in a way that
            maintains uniform sample distribution and has no effect on the theoretical asymptotic optimality of the
            planner. */
            void setJustInTimeSampling(bool useJit);

            /** \brief Get whether we're using just-in-time sampling */
            bool getJustInTimeSampling() const;

            /** @anchor gBITstarSetDropSamplesOnPrune \brief Drop \e all unconnected samples when pruning, regardless of
            their heuristic value.
            This provides a method for BIT* to remove samples that have not been connected to the graph and may be
            beneficial in problems where
            portions of the free space are unreachable (i.e., disconnected). BIT* calculates the connection radius for
            each batch from the underlying
            uniform distribution of states. The resulting larger connection radius may be detrimental in areas where the
            graph is dense, but maintains
            the theoretical asymptotic optimality of the planner.
            */
            void setDropSamplesOnPrune(bool dropSamples);

            /** \brief Get whether unconnected samples are dropped on pruning. */
            bool getDropSamplesOnPrune() const;

            /** \brief Stop the planner each time a solution improvement is found. Useful
            for examining the intermediate solutions found by BIT*. */
            void setStopOnSolnImprovement(bool stopOnChange);

            /** \brief Get whether BIT* stops each time a solution is found. */
            bool getStopOnSolnImprovement() const;

            /** \brief Set BIT* to consider approximate solutions during its initial search. */
            void setConsiderApproximateSolutions(bool findApproximate);

            /** \brief Get whether BIT* is considering approximate solutions. */
            bool getConsiderApproximateSolutions() const;
            ///////////////////////////////////////

        protected:
            // Everything is only protected so we can create modifications without duplicating code by deriving from the
            // class:

            // Functions:
            /** \brief A debug function: Estimate the measure of the free/obstace space via sampling. */
            void estimateMeasures();

            ///////////////////////////////////////////////////////////////////
            // BIT* primitives:
            /** \brief A single iteration */
            virtual void iterate();

            /** \brief Initialize variables for a new batch */
            void newBatch();

            /** \brief Update the list of free samples */
            void updateSamples(const VertexConstPtr &vertex);

            /** \brief Prune the problem. Returns true if pruning was done. */
            virtual bool prune();

            /** \brief Resort the queue. Returns true if any pruning was done. */
            virtual bool resort();

            /** \brief Publish the found solution to the ProblemDefinition*/
            void publishSolution();
            ///////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////////////////////////
            // Helper functions for data manipulation and other low-level functions
            /** \brief Extract the best solution, ordered \e from the goal to the \e start and including both the goal
             * and the start. Used by both publishSolution and the ProblemDefinition::IntermediateSolutionCallback */
            std::vector<const ompl::base::State *> bestPathFromGoalToStart() const;

            /** \brief Adds any new goals or starts that have appeared in the problem definition to the list of vertices
             * and the queue. Creates a new informed sampler. Returns true if new starts/goals are created. */
            void updateStartAndGoalStates(const base::PlannerTerminationCondition &ptc);

            /** \brief Prune the starts and goals that have a solution heuristic that is not less than bestCost_.
             * Returns the number pruned */
            std::pair<unsigned int, unsigned int> pruneStartsGoals();

            /** \brief Prune all samples with a solution heuristic that is not less than the bestCost_. Returns the
             * number pruned */
            unsigned int pruneSamples();

            /** \brief Checks an edge for collision. A wrapper to SpaceInformation->checkMotion that tracks number of
             * collision checks. */
            bool checkEdge(const VertexConstPtrPair &edge);

            /** \brief Actually remove a sample from its NN struct.*/
            void dropSample(const VertexPtr &oldSample);

            /** \brief Add an edge from the edge queue to the tree. Will add the state to the vertex queue if it's new
             * to the tree or otherwise replace the parent. Updates solution information if the solution improves. */
            void addEdge(const VertexPtrPair &newEdge, const ompl::base::Cost &edgeCost, const bool &removeFromFree,
                         const bool &updateDescendants);

            /** \brief Replace the parent edge with the given new edge and cost */
            void replaceParent(const VertexPtrPair &newEdge, const ompl::base::Cost &edgeCost,
                               const bool &updateDescendants);

            /** \brief The special work that needs to be done to update the goal vertex if the solution has changed. */
            void updateGoalVertex();

            /** \brief Find an approximate solution from the current graph, if enabled and an exact solution DOES NOT
             * exist. */
            void findNewApproxSoln();

            /** \brief If enabled and an exact solution DOES NOT exist, consider whether the provided vertex improves
             * the approximate solution. */
            void updateApproxSoln(const VertexConstPtr &newVertex);

            /** \brief Add a sample */
            void addSample(const VertexPtr &newSample);

            /** \brief Add a vertex to the graph */
            void addVertex(const VertexPtr &newVertex, const bool &removeFromFree);

            /** \brief Get the nearest samples from the freeStateNN_ using the appropriate "near" definition (i.e., k or
             * r). If using k-nearest, returns the target k, otherwise returns 0u. */
            unsigned int nearestSamples(const VertexPtr &vertex, VertexPtrVector *neighbourSamples);

            /** \brief Get the nearest samples from the vertexNN_ using the appropriate "near" definition (i.e., k or
             * r). If using k-nearest, returns the target k, otherwise returns 0u. */
            unsigned int nearestVertices(const VertexPtr &vertex, VertexPtrVector *neighbourVertices);
            ///////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////////////////////////
            // Helper functions for sorting queues/nearest-neighbour structures and the related calculations.
            /** \brief The distance function used for nearest neighbours. Calculates the distance directionally from the
             * given state to all the other states (can be used on states either in our out of the graph).*/
            double nnDistance(const VertexConstPtr &a, const VertexConstPtr &b) const;
            ///////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////////////////////////
            // Helper functions for various costs.
            /** \brief The true cost of an edge, including collisions.*/
            ompl::base::Cost trueEdgeCost(const VertexConstPtrPair &edgePair) const;

            /** \brief Calculate the max req'd cost to define a neighbourhood around a state. Currently only implemented
             * for path-length problems, for which the neighbourhood cost is the f-value of the vertex plus 2r. */
            ompl::base::Cost neighbourhoodCost(const VertexConstPtr &vertex) const;
            ///////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////////////////////////
            // Helper functions to calculate parameters:
            /** \brief Initialize the nearest-neighbour terms */
            void initializeNearestTerms();

            /** \brief Update the appropriate nearest-neighbour terms, r_ and k_. Performs this calculation considering
             * the "future" samples to be added in this batch, except on the first batch. */
            virtual void updateNearestTerms();

            /** \brief Calculate the r for r-disc nearest neighbours, a function of the current graph */
            double calculateR(unsigned int N) const;

            /** \brief Calculate the k for k-nearest neighours, a function of the current graph */
            unsigned int calculateK(unsigned int N) const;

            /** \brief Calculate the lower-bounding radius RGG term for asymptotic almost-sure convergence to the
             * optimal path (i.e., r_rrg* in Karaman and Frazzoli IJRR 11). This is a function of the size of the
             * problem domain. */
            double minimumRggR() const;

            /** \brief Calculate the lower-bounding k-nearest RGG term for asymptotic almost-sure convergence to the
             * optimal path (i.e., k_rrg* in Karaman and Frazzoli IJRR 11). This is a function of the state dimension
             * and is left as a double for later accuracy in calculate k */
            double minimumRggK() const;
            ///////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////////////////////////
            // Helper functions for logging
            /** \brief The message printed when a goal is found/improved */
            virtual void goalMessage() const;

            /** \brief The message printed when solve finishes successfully */
            virtual void endSuccessMessage() const;

            /** \brief The message printed when solve finishes unsuccessfully */
            virtual void endFailureMessage() const;

            /** \brief A debug-level status message for debugging. */
            virtual void statusMessage(const ompl::msg::LogLevel &msgLevel, const std::string &status) const;
            ///////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////////////////////////
            // Planner progress property functions
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

            /** \brief Retrieve the number of edge (or motion) collision checks (i.e., calls to
            SpaceInformation::checkMotion(...))
            as a planner-progress property. (numEdgeCollisionChecks_) */
            std::string edgeCollisionCheckProgressProperty() const;

            /** \brief Retrieve the number of nearest neighbour calls (i.e., NearestNeighbors<T>::nearestK(...) or
            NearestNeighbors<T>::nearestR(...))
            as a planner-progress property. (numNearestNeighbours_) */
            std::string nearestNeighbourProgressProperty() const;

            /** \brief Retrieve the total number of edges processed from the queue as a planner-progress property.
             * (numEdgesProcessed_) */
            std::string edgesProcessedProgressProperty() const;
            ///////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////////////////////////
            // Variables -- Make sure every one is configured in setup() and reset in clear():
            /** \brief An instance of a random number generator */
            ompl::RNG rng_;

            /** \brief State sampler */
            ompl::base::InformedSamplerPtr sampler_;

            /** \brief Optimization objective copied from ProblemDefinition */
            ompl::base::OptimizationObjectivePtr opt_;

            /** \brief The start states of the problem as vertices. Constructed as a shared_ptr to give easy access to
             * helper classes */
            std::shared_ptr<VertexPtrList> startVerticesPtr_;

            /** \brief The goal states of the problem as vertices. Constructed as a shared_ptr to give easy access to
             * helper classes */
            std::shared_ptr<VertexPtrList> goalVerticesPtr_;

            /** \brief A helper for cost and heuristic calculations */
            std::shared_ptr<CostHelper> costHelpPtr_;

            /** \brief Any start states of the problem that have been pruned */
            VertexPtrList prunedStartVertices_;

            /** \brief Any goal states of the problem that have been pruned */
            VertexPtrList prunedGoalVertices_;

            /** \brief The goal vertex of the current best solution */
            VertexConstPtr curGoalVertex_;

            /** \brief The unconnected samples as a nearest-neighbours datastructure. Sorted by nnDistance. Size
             * accessible via currentFreeProgressProperty */
            VertexPtrNNPtr freeStateNN_;

            /** \brief The vertices as a nearest-neighbours data structure. Sorted by nnDistance. Size accessible via
             * currentVertexProgressProperty */
            VertexPtrNNPtr vertexNN_;

            /** \brief The integrated queue of vertices to expand and edges to process ordered on "f-value", i.e.,
             * estimated solution cost. Remaining vertex queue "size" and edge queue size are accessible via
             * vertexQueueSizeProgressProperty and edgeQueueSizeProgressProperty, respectively. */
            IntegratedQueuePtr intQueue_;

            /** \brief A copy of the new samples from this batch */
            VertexPtrVector newSamples_;

            /** \brief A copy of the vertices recycled into samples during this batch */
            VertexPtrVector recycledSamples_;

            /** \brief The number of states (vertices or samples) that were generated from a uniform distribution. Only
             * valid when refreshSamplesOnPrune_ is true, in which case it's used to calculate the RGG term of the
             * uniform subgraph.*/
            unsigned int numUniformStates_;

            /** \brief The current r-disc RGG connection radius */
            double r_;

            /** \brief The minimum k-nearest RGG connection term. Only a function of state dimension, so can be
             * calculated once. Left as a double for later accuracy in calculate k */
            double k_rgg_;

            /** \brief The current k-nearest RGG connection number */
            unsigned int k_;

            /** \brief The best cost found to date. This is the maximum total-heuristic cost of samples we'll consider.
             * Accessible via bestCostProgressProperty */
            ompl::base::Cost bestCost_;

            /** \brief The number of vertices in the best solution found to date. Accessible via
             * bestLengthProgressProperty */
            unsigned int bestLength_;

            /** \brief The cost to which the graph has been pruned. We will only prune the graph if bestCost_ is less
             * than this value. */
            ompl::base::Cost prunedCost_;

            /** \brief The measure of the problem domain when we pruned the graph. */
            double prunedMeasure_;

            /** \brief The minimum possible solution cost. I.e., the heuristic value of the goal. */
            ompl::base::Cost minCost_;

            /** \brief The total-heuristic cost up to which we've sampled */
            ompl::base::Cost costSampled_;

            /** \brief If we've found an exact solution yet */
            bool hasExactSolution_;

            /** \brief A manual stop on the solve loop */
            bool stopLoop_;

            /** \brief An approximate solution */
            VertexConstPtr approxGoalVertex_;
            ///////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////////////////////////
            // Informational variables - Make sure initialized in setup and reset in clear
            /** \brief The distance of the approximate solution, set to -1.0 for non approximate solutions */
            double approxDiff_;

            /** \brief The number of iterations run. Accessible via iterationProgressProperty */
            unsigned int numIterations_;

            /** \brief The number of batches processed. Accessible via batchesProgressProperty */
            unsigned int numBatches_;

            /** \brief The number of times the graph/samples have been pruned. Accessible via pruningProgressProperty */
            unsigned int numPrunings_;

            /** \brief The number of states generated through sampling. Accessible via
             * statesFromSamplingProgressProperty */
            unsigned int numSamples_;

            /** \brief The number of vertices ever added to the graph. Will count vertices twice if they spend any time
             * disconnected. Accessible via verticesConstructedProgressProperty */
            unsigned int numVertices_;

            /** \brief The number of free states that have been pruned. Accessible via statesPrunedProgressProperty */
            unsigned int numFreeStatesPruned_;

            /** \brief The number of graph vertices that get disconnected. These either return to being free samples or
             * are pruned completely. Accessible via verticesDisconnectedProgressProperty */
            unsigned int numVerticesDisconnected_;

            /** \brief The number of times a state in the graph was rewired. Accessible via rewiringProgressProperty */
            unsigned int numRewirings_;

            /** \brief The number of state collision checks. Accessible via stateCollisionCheckProgressProperty */
            unsigned int numStateCollisionChecks_;

            /** \brief The number of edge collision checks. Accessible via edgeCollisionCheckProgressProperty */
            unsigned int numEdgeCollisionChecks_;

            /** \brief The number of nearest neighbour calls. Accessible via nearestNeighbourProgressProperty */
            unsigned int numNearestNeighbours_;

            /** \brief The number of edges processed, in one way or other, from the queue. Accessible via
             * edgesProcessedProgressProperty */
            unsigned int numEdgesProcessed_;
            ///////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////////////////////////
            // Parameters - Set defaults in construction/setup and DO NOT reset in clear.
            /** \brief Whether to use a strict-queue ordering (param) */
            bool useStrictQueueOrdering_;

            /** \brief The rewiring factor, s, so that r_rrg = s \times r_rrg* > r_rrg* (param) */
            double rewireFactor_;

            /** \brief The number of samples per batch (param) */
            unsigned int samplesPerBatch_;

            /** \brief Option to use k-nearest search for rewiring (param) */
            bool useKNearest_;

            /** \brief Whether to use graph pruning (param) */
            bool usePruning_;

            /** \brief The fractional decrease in solution cost required to trigger pruning (param) */
            double pruneFraction_;

            /** \brief Whether to delay rewiring until a solution is found (param) */
            bool delayRewiring_;

            /** \brief Whether to use just-in-time sampling (param) */
            bool useJustInTimeSampling_;

            /** \brief Whether to refresh (i.e., forget) unconnected samples on pruning (param) */
            bool dropSamplesOnPrune_;

            /** \brief Whether to stop the planner as soon as the path changes (param) */
            bool stopOnSolnChange_;

            /** \brief Whether to consider approximate solutions (param) */
            bool findApprox_;
            ///////////////////////////////////////////////////////////////////
        };  // class: BITstar
    }       // geometric
}  // ompl

// BIT* Includes:
// The Vertex ID generator class
#include "ompl/geometric/planners/bitstar/datastructures/IdGenerator.h"
// My vertex class:
#include "ompl/geometric/planners/bitstar/datastructures/Vertex.h"
// My cost & heuristic helper class
#include "ompl/geometric/planners/bitstar/datastructures/CostHelper.h"
// My queue class
#include "ompl/geometric/planners/bitstar/datastructures/IntegratedQueue.h"

#endif  // OMPL_GEOMETRIC_PLANNERS_BITSTAR_BITSTAR_
