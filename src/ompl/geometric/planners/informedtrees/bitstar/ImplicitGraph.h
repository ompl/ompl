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

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BITSTAR_IMPLICITGRAPH_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BITSTAR_IMPLICITGRAPH_

#include "ompl/base/Cost.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/planners/informedtrees/BITstar.h"

namespace ompl
{
    namespace geometric
    {
        /** @anchor ImplicitGraph
        \par Short Description
        An edge-implicit representation of a random geometric graph.
        TODO(Marlin): Separating the search tree from the RGG seems conceptually cleaner. Think about its implications.
        */

        /** \brief A conceptual representation of samples as an edge-implicit random geometric graph. */
        class BITstar::ImplicitGraph
        {
        public:
            // ---
            // Construction, initialization and destruction.
            // ---

            /** \brief Construct an implicit graph. */
            ImplicitGraph(NameFunc nameFunc);

            /** \brief Destruct the graph using default destruction. */
            virtual ~ImplicitGraph() = default;

            /** \brief Setup the ImplicitGraph, must be called before use. Does not take a copy of the
             * PlannerInputStates, but checks it for starts/goals. */
            void setup(const ompl::base::SpaceInformationPtr &spaceInformation,
                       const ompl::base::ProblemDefinitionPtr &problemDefinition, CostHelper *costHelper,
                       SearchQueue *searchQueue, const ompl::base::Planner *plannerPtr,
                       ompl::base::PlannerInputStates &inputStates);

            /** \brief Reset the graph to the state of construction. */
            void reset();

            // ---
            // Information access.
            // ---

            /** \brief Gets whether the graph contains a start or not. */
            bool hasAStart() const;

            /** \brief Gets whether the graph contains a goal or not. */
            bool hasAGoal() const;

            /** \brief Returns a const-iterator to the front of the start-vertex vector. */
            VertexPtrVector::const_iterator startVerticesBeginConst() const;

            /** \brief Returns a const-iterator to the end of the start-vertex vector. */
            VertexPtrVector::const_iterator startVerticesEndConst() const;

            /** \brief Returns a const-iterator to the front of the goal-vertex vector. */
            VertexPtrVector::const_iterator goalVerticesBeginConst() const;

            /** \brief Returns a const-iterator to the end of the goal-vertex vector. */
            VertexPtrVector::const_iterator goalVerticesEndConst() const;

            /** \brief Get the minimum cost solution possible for this problem. */
            ompl::base::Cost minCost() const;

            /** \brief Query whether the underlying state sampler can provide an informed measure. */
            bool hasInformedMeasure() const;

            /** \brief Query the underlying state sampler for the informed measure of the problem. */
            double getInformedMeasure(const ompl::base::Cost &cost) const;

            /** \brief Computes the distance between two states.*/
            double distance(const VertexConstPtr &a, const VertexConstPtr &b) const;

            /** \brief Computes the distance between two states.*/
            double distance(const VertexConstPtrPair &vertices) const;

            /** \brief Get the nearest unconnected samples using the appropriate "near" definition (i.e., k or r). */
            void nearestSamples(const VertexPtr &vertex, VertexPtrVector *neighbourSamples);

            /** \brief Adds the graph to the given PlannerData struct. */
            void getGraphAsPlannerData(ompl::base::PlannerData &data) const;

            /** \brief IF BEING TRACKED, returns the closest vertex in the tree to the goal. */
            VertexConstPtr closestVertexToGoal() const;

            /** \brief IF BEING TRACKED, returns the how close vertices in the tree are to the goal. */
            double smallestDistanceToGoal() const;

            /** \brief Get the k of this k-nearest RGG. */
            unsigned int getConnectivityK() const;

            /** \brief Get the radius of this r-disc RGG. */
            double getConnectivityR() const;

            /** \brief Get a copy of all samples. */
            VertexPtrVector getCopyOfSamples() const;

            // ---
            // Modification.
            // ---

            /** \brief Mark that a solution has been found and that the graph should be limited to the given heuristic
             * value. */
            void registerSolutionCost(const ompl::base::Cost &solutionCost);

            /** \brief Adds any new goals or starts that have appeared in the problem definition to the vector of
             * vertices and the queue. Creates a new informed sampler if necessary. */
            void updateStartAndGoalStates(ompl::base::PlannerInputStates &inputStates,
                                          const base::PlannerTerminationCondition &terminationCondition);

            /** \brief Increase the resolution of the graph-based approximation of the continuous search domain by
             * adding a batch of new samples. */
            void addNewSamples(const unsigned int &numSamples);

            /** \brief Prune the samples to the subproblem of the given measure. Returns the number of vertices
             * disconnected and the number of samples removed. */
            std::pair<unsigned int, unsigned int> prune(double prunedMeasure);

            /** \brief Add an unconnected sample. */
            void addToSamples(const VertexPtr &sample);

            /** \brief Add a vector of unconnected samples. */
            void addToSamples(const VertexPtrVector &samples);

            /** \brief Remove a sample from the sample set. */
            void removeFromSamples(const VertexPtr &sample);

            /** \brief Remove an unconnected sample.*/
            void pruneSample(const VertexPtr &sample);

            /** \brief Insert a sample into the set for recycled samples.*/
            void recycleSample(const VertexPtr &sample);

            /** \brief Add a vertex to the tree, optionally moving it from the set of unconnected samples. */
            void registerAsVertex(const VertexPtr &vertex);

            /** \brief Remove a vertex from the tree, can optionally be allowed to move it to the set of unconnected
             * samples if may still be useful. */
            unsigned int removeFromVertices(const VertexPtr &sample, bool moveToFree);

            /** \brief Remove a vertex and mark as pruned. */
            std::pair<unsigned int, unsigned int> pruneVertex(const VertexPtr &vertex);

            /** \brief Disconnect a vertex from its parent by removing the edges stored in itself, and its parents.
             * Cascades cost updates if requested.*/
            void removeEdgeBetweenVertexAndParent(const VertexPtr &child, bool cascadeCostUpdates);

            // ---
            // Settings.
            // ---

            /** \brief Set the rewiring scale factor, s, such that r_rrg = s \times r_rrg*. */
            void setRewireFactor(double rewireFactor);

            /** \brief Get the rewiring scale factor. */
            double getRewireFactor() const;

            /** \brief Enable a k-nearest search for instead of an r-disc search. */
            void setUseKNearest(bool useKNearest);

            /** \brief Get whether a k-nearest search is being used.*/
            bool getUseKNearest() const;

            /** Enable sampling "just-in-time", i.e., only when necessary for a nearest-neighbour search. */
            void setJustInTimeSampling(bool useJit);

            /** \brief Get whether we're using just-in-time sampling. */
            bool getJustInTimeSampling() const;

            /** \brief Set whether unconnected samples are dropped on pruning. */
            void setDropSamplesOnPrune(bool dropSamples);

            /** \brief Set whether samples that are provably not beneficial should be kept around. */
            void setPruning(bool usePruning);

            /** \brief Get whether unconnected samples are dropped on pruning. */
            bool getDropSamplesOnPrune() const;

            /** \brief Set whether to track approximate solutions during the search. */
            void setTrackApproximateSolutions(bool findApproximate);

            /** \brief Get whether approximate solutions are tracked during the search. */
            bool getTrackApproximateSolutions() const;

            /** \brief Set the average number of allowed failed attempts when sampling. */
            void setAverageNumOfAllowedFailedAttemptsWhenSampling(std::size_t number);

            /** \brief Get the average number of allowed failed attempts when sampling. */
            std::size_t getAverageNumOfAllowedFailedAttemptsWhenSampling() const;

            /** \brief Set a different nearest neighbours datastructure. */
            template <template <typename T> class NN>
            void setNearestNeighbors();

            // ---
            // Progress counters.
            // ---

            /** \brief The number of samples. */
            unsigned int numSamples() const;

            /** \brief The number of vertices in the search tree. */
            unsigned int numVertices() const;

            /** \brief The \e total number of states generated. */
            unsigned int numStatesGenerated() const;

            /** \brief The \e total number of vertices added to the graph. */
            unsigned int numVerticesConnected() const;

            /** \brief The number of states pruned. */
            unsigned int numFreeStatesPruned() const;

            /** \brief The number of tree vertices disconnected. */
            unsigned int numVerticesDisconnected() const;

            /** \brief The number of nearest neighbour calls. */
            unsigned int numNearestLookups() const;

            /** \brief The number of state collision checks. */
            unsigned int numStateCollisionChecks() const;

            // ---
            // General helper functions.
            // ---

            /** \brief Returns whether the vertex can be pruned, i.e., whether it could provide a better solution given.
             * the current graph. The check should always be g_t(v) + h^(v) >= g_t(x_g). */
            bool canVertexBeDisconnected(const VertexPtr &vertex) const;

            /** \brief Returns whether the sample can be pruned, i.e., whether it could ever provide a better solution.
             * The check should always be g^(v) + h^(v) >= g_t(x_g). */
            bool canSampleBePruned(const VertexPtr &sample) const;

        private:
            // ---
            // High-level primitives updating the graph.
            // ---

            /** \brief Update the set of free samples such that the neighbourhood of the given vertex is sufficiently
             * sampled. */
            void updateSamples(const VertexConstPtr &vertex);

            /** \brief Iterates through all the vertices in the tree and finds the one that is closes to the goal. This
             * is only necessary to find approximate solutions and should otherwise not be called. */
            void updateVertexClosestToGoal();

            // ---
            // High-level primitives pruning the graph.
            // ---

            /** \brief Prune any starts/goals that provably cannot provide a better solution than the current best
             * solution. This is done via the prune conditions of the SearchQueue. Returns the number of vertices
             * disconnected and the number of samples removed. */
            std::pair<unsigned int, unsigned int> pruneStartAndGoalVertices();

            /** \brief Prune any samples that provably cannot provide a better solution than the current best solution.
             * Returns the number of samples removed. */
            std::pair<unsigned int, unsigned int> pruneSamples();

            // ---
            // Low-level random geometric graph helper and calculations
            // ---

            /** \brief Tests and updates whether the given vertex is closer to the goal than the known-closest vertex.
             * This is only necessary to find approximate solutions and should otherwise not be called. */
            void testClosestToGoal(const VertexConstPtr &vertex);

            /** \brief Calculate the max req'd cost to define a neighbourhood around a state. Currently only implemented
             * for path-length problems, for which the neighbourhood cost is the f-value of the vertex plus 2r. */
            ompl::base::Cost calculateNeighbourhoodCost(const VertexConstPtr &vertex) const;

            /** \brief Update the appropriate nearest-neighbour terms, r_ and k_. */
            virtual void updateNearestTerms();

            /** \brief Computes the number of samples in the informed set. */
            std::size_t computeNumberOfSamplesInInformedSet() const;

            /** \brief Calculate the r for r-disc nearest neighbours, a function of the current graph. */
            double calculateR(unsigned int numUniformSamples) const;

            /** \brief Calculate the k for k-nearest neighours, a function of the current graph. */
            unsigned int calculateK(unsigned int numUniformSamples) const;

            /** \brief Calculate the lower-bounding radius RGG term for asymptotic almost-sure convergence to the
             * optimal path (i.e., r_rrg* in Karaman and Frazzoli IJRR 11). This is a function of the size of the
             * problem domain. */
            double calculateMinimumRggR() const;

            /** \brief Calculate the lower-bounding k-nearest RGG term for asymptotic almost-sure convergence to the
             * optimal path (i.e., k_rrg* in Karaman and Frazzoli IJRR 11). This is a function of the state dimension
             * and is left as a double for later accuracy in calculate k. */
            double calculateMinimumRggK() const;

            // ---
            // Debug helpers.
            // ---

            /** \brief Test if the class is setup and throw if not. */
            void assertSetup() const;

            // ---
            // Member variables (Make all are configured in setup() and reset in reset()).
            // ---

            /** \brief A function pointer to the planner name, for better OMPL_INFO, etc. output. */
            NameFunc nameFunc_;

            /** \brief Whether the class is setup. */
            bool isSetup_{false};

            /** \brief The state space used by the planner. */
            ompl::base::SpaceInformationPtr spaceInformation_{nullptr};

            /** \brief The problem definition. */
            ompl::base::ProblemDefinitionPtr problemDefinition_{nullptr};

            /** \brief A cost/heuristic helper class. As this is a copy of the version owned by BITstar.cpp it can be
             * reset in a clear(). */
            CostHelper *costHelpPtr_{nullptr};

            /** \brief The queue class. As this is a copy of the version owned by BITstar.cpp it can be reset in a
             * clear(). */
            SearchQueue *queuePtr_{nullptr};

            /** \brief An instance of a random number generator. */
            ompl::RNG rng_;

            /** \brief State sampler */
            ompl::base::InformedSamplerPtr sampler_{nullptr};

            /** \brief The start states of the problem as vertices. Constructed as a shared_ptr to give easy access to
             * helper classes. */
            VertexPtrVector startVertices_;

            /** \brief The goal states of the problem as vertices. Constructed as a shared_ptr to give easy access to
             * helper classes. */
            VertexPtrVector goalVertices_;

            /** \brief Any start states of the problem that have been pruned. */
            VertexPtrVector prunedStartVertices_;

            /** \brief Any goal states of the problem that have been pruned. */
            VertexPtrVector prunedGoalVertices_;

            /** \brief A copy of the new samples of the most recently added batch. */
            VertexPtrVector newSamples_;

            /** \brief The samples as a nearest-neighbours datastructure. Sorted by nnDistance. */
            VertexPtrNNPtr samples_{nullptr};

            /** \brief A copy of the vertices recycled into samples during the most recently added batch. */
            VertexPtrVector recycledSamples_;

            /** \brief The number of samples in this batch. */
            unsigned int numNewSamplesInCurrentBatch_{0u};

            /** \brief The number of states (vertices or samples) that were generated from a uniform distribution. Only
             * valid when refreshSamplesOnPrune_ is true, in which case it's used to calculate the RGG term of the
             * uniform subgraph. */
            unsigned int numUniformStates_{0u};

            /** \brief The current r-disc RGG connection radius. */
            double r_{0.};

            /** \brief The minimum k-nearest RGG connection term. Only a function of state dimension, so can be
             * calculated once. Left as a double for later accuracy in calculate k. */
            double k_rgg_{0.};

            /** \brief The current k-nearest RGG connection number. */
            unsigned int k_{0u};

            /** \brief The measure of the continuous problem domain which we are approximating with samples. This is
             * initially the problem domain but can shrink as we focus the search. */
            double approximationMeasure_{0.};

            /** \brief The minimum possible solution cost. I.e., the heuristic value of the start. */
            ompl::base::Cost minCost_{std::numeric_limits<double>::infinity()};

            /** \brief The maximum heuristic cost to sample (i.e., the best solution found to date). */
            ompl::base::Cost solutionCost_{std::numeric_limits<double>::infinity()};

            /** \brief The total-heuristic cost up to which we've sampled. */
            ompl::base::Cost sampledCost_{std::numeric_limits<double>::infinity()};

            /** \brief If we've found an exact solution yet. */
            bool hasExactSolution_{false};

            /** \brief If being tracked, the vertex closest to the goal (this represents an "approximate" solution). */
            VertexConstPtr closestVertexToGoal_{nullptr};

            /** \brief If being tracked, the smallest distance of vertices in the tree to a goal (this represents
             * tolerance of an "approximate" solution). */
            double closestDistanceToGoal_{std::numeric_limits<double>::infinity()};

            /** \brief The number of states generated through sampling. Accessible via numStatesGenerated. */
            unsigned int numSamples_{0u};

            /** \brief The number of vertices ever added to the tree. Accessible via numVerticesConnected. */
            unsigned int numVertices_{0u};

            /** \brief The number of free states that have been pruned. Accessible via numStatesPruned. */
            unsigned int numFreeStatesPruned_{0u};

            /** \brief The number of graph vertices that get disconnected. Accessible via numVerticesDisconnected. */
            unsigned int numVerticesDisconnected_{0u};

            /** \brief The number of nearest neighbour calls. Accessible via numNearestLookups. */
            unsigned int numNearestNeighbours_{0u};

            /** \brief The number of state collision checks. Accessible via numStateCollisionChecks. */
            unsigned int numStateCollisionChecks_{0u};

            /** \brief The current approximation id. */
            const std::shared_ptr<unsigned int> approximationId_;

            // ---
            // Parameters - Set defaults in construction/setup and do not reset in clear.
            // ---

            /** \brief The rewiring factor, s, so that r_rgg = s \times r_rgg* > r_rgg*. */
            double rewireFactor_{1.1};

            /** \brief Option to use k-nearest search for rewiring. */
            bool useKNearest_{true};

            /** \brief Whether to use just-in-time sampling. */
            bool useJustInTimeSampling_{false};

            /** \brief Whether to refresh (i.e., forget) unconnected samples on pruning. */
            bool dropSamplesOnPrune_{false};

            /** \brief Whether the graph is being pruned or not. */
            bool isPruningEnabled_{true};

            /** \brief Whether to consider approximate solutions. */
            bool findApprox_{false};

            /** \brief The average number of allowed failed attempts before giving up on a sample when sampling a new
             * batch. */
            std::size_t averageNumOfAllowedFailedAttemptsWhenSampling_{2u};
        };  // class ImplicitGraph
    }       // namespace geometric
}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BITSTAR_IMPLICITGRAPH_
