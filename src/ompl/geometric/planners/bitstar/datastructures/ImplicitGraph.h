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

#ifndef OMPL_GEOMETRIC_PLANNERS_BITSTAR_DATASTRUCTURES_IMPLICITGRAPH_
#define OMPL_GEOMETRIC_PLANNERS_BITSTAR_DATASTRUCTURES_IMPLICITGRAPH_

// OMPL:
// The cost class:
#include "ompl/base/Cost.h"
// The optimization objective class:
#include "ompl/base/OptimizationObjective.h"
// The nearest neighbours structure
#include "ompl/datastructures/NearestNeighbors.h"

// BIT*:
// I am member class of the BITstar class (i.e., I am in it's namespace), so I need to include it's definition to be
// aware of the class BITstar. It has a forward declaration to me and the other helper classes but I will need to
// include any I use in my .cpp (to avoid dependency loops).
#include "ompl/geometric/planners/bitstar/BITstar.h"

namespace ompl
{
    namespace geometric
    {
        /** @anchor ImplicitGraph
        \par Short Description
        An edge-implicit representation of a random geometric graph.
        */

        /** \brief A conceptual representation of samples as an edge-implicit random geometric graph. */
        class BITstar::ImplicitGraph
        {
        public:
            ////////////////////////////////
            // Public functions:
            // Construction and initialization
            /** \brief Construct an implicit graph. */
            ImplicitGraph(NameFunc nameFunc);

            virtual ~ImplicitGraph() = default;

            /** \brief Setup the ImplicitGraph, must be called before use. Does not take a copy of the
             * PlannerInputStates, but checks it for starts/goals. */
            void setup(const ompl::base::SpaceInformationPtr &si, const ompl::base::ProblemDefinitionPtr &pdef,
                       CostHelper *costHelper, SearchQueue *searchQueue,
                       const ompl::base::Planner *plannerPtr, ompl::base::PlannerInputStates &pis);

            /** \brief Clear the graph to the state of construction. */
            void clear();
            //////////////////

            //////////////////
            // Graph access:
            /** \brief Gets whether the graph contains a start or not. */
            bool hasAStart() const;

            /** \brief Gets whether the graph contains a goal or not. */
            bool hasAGoal() const;

            /** \brief Returns a const-iterator to the front of the start-vertex vector */
            VertexPtrVector::const_iterator startVerticesBeginConst() const;

            /** \brief Returns a const-iterator to the end of the start-vertex vector */
            VertexPtrVector::const_iterator startVerticesEndConst() const;

            /** \brief Returns a const-iterator to the front of the goal-vertex vector */
            VertexPtrVector::const_iterator goalVerticesBeginConst() const;

            /** \brief Returns a const-iterator to the end of the goal-vertex vector */
            VertexPtrVector::const_iterator goalVerticesEndConst() const;

            /** \brief Get the minimum cost solution possible for this problem. */
            ompl::base::Cost minCost() const;

            /** \brief Query whether the underlying state sampler can provide an informed measure. */
            bool hasInformedMeasure() const;

            /** \brief Query the underlying state sampler for the informed measure of the problem. */
            double getInformedMeasure(const ompl::base::Cost &cost) const;

            /** \brief The distance function. Calculates the distance directionally from the given state to all the
             * other states (can be used on states either in our out of the graph).*/
            double distanceFunction(const VertexConstPtr &a, const VertexConstPtr &b) const;

            /** \brief Get the nearest unconnected samples using the appropriate "near" definition (i.e., k or r). */
            void nearestSamples(const VertexPtr &vertex, VertexPtrVector *neighbourSamples);

            /** \brief Get the nearest samples from the vertexNN_ using the appropriate "near" definition (i.e., k or
             * r). */
            void nearestVertices(const VertexPtr &vertex, VertexPtrVector *neighbourVertices);

            /** \brief Adds the graph to the given PlannerData struct */
            void getGraphAsPlannerData(ompl::base::PlannerData &data) const;

            /** \brief IF BEING TRACKED, returns the closest vertex in the tree to the goal. */
            VertexConstPtr closestVertexToGoal() const;

            /** \brief IF BEING TRACKED, returns the how close vertices in the tree are to the goal. */
            double smallestDistanceToGoal() const;

            /** \brief Get the k of this k-nearest RGG. */
            unsigned int getConnectivityK() const;

            /** \brief Get the radius of this r-disc RGG. */
            double getConnectivityR() const;
            //////////////////

            //////////////////
            // Graph modification:
            /** \brief Mark that a solution has been found and that the graph should be limited to the given heuristic
             * value */
            void hasSolution(const ompl::base::Cost &solnCost);

            /** \brief Adds any new goals or starts that have appeared in the problem definition to the vector of
             * vertices and the queue. Creates a new informed sampler if necessary. */
            void updateStartAndGoalStates(ompl::base::PlannerInputStates &pis,
                                          const base::PlannerTerminationCondition &ptc);

            /** \brief Increase the resolution of the graph-based approximation of the continuous search domain by
             * adding a batch of new samples. */
            void addNewSamples(const unsigned int &numSamples);

            /** \brief Prune the samples to the subproblem of the given measure. Pruning is performed by using the prune
             * conditions of the SearchQueue. Returns the number of vertices disconnected and the number of samples
             * removed. */
            std::pair<unsigned int, unsigned int> prune(double prunedMeasure);
            //////////////////

            //////////////////
            // Adding/remove individual states:
            /** \brief Add an unconnected sample */
            void addSample(const VertexPtr &newSample);

            /** \brief Remove an unconnected sample.*/
            void removeSample(const VertexPtr &oldSample);

            /** \brief Add a vertex to the tree, optionally moving it from the set of unconnected samples. */
            void addVertex(const VertexPtr &newVertex, bool removeFromFree);

            /** \brief Remove a vertex from the tree, can optionally be allowed to move it to the set of unconnected
             * samples if may still be useful. */
            unsigned int removeVertex(const VertexPtr &oldSample, bool moveToFree);
            //////////////////

            //////////////////
            // General helper functions
            void assertValidSample(const VertexConstPtr &sample, bool mustBeNew);
            //////////////////

            //////////////////
            // Graph settings:
            /** \brief Set the rewiring scale factor, s, such that r_rrg = s \times r_rrg* */
            void setRewireFactor(double rewireFactor);

            /** \brief Get the rewiring scale factor. */
            double getRewireFactor() const;

            /** \brief Enable a k-nearest search for instead of an r-disc search. */
            void setUseKNearest(bool useKNearest);

            /** \brief Get whether a k-nearest search is being used.*/
            bool getUseKNearest() const;

            /** Enable sampling "just-in-time", i.e., only when necessary for a nearest-neighbour search. */
            void setJustInTimeSampling(bool useJit);

            /** \brief Get whether we're using just-in-time sampling */
            bool getJustInTimeSampling() const;

            /** \brief Set whether unconnected samples are dropped on pruning. */
            void setDropSamplesOnPrune(bool dropSamples);

            /** \brief Get whether unconnected samples are dropped on pruning. */
            bool getDropSamplesOnPrune() const;

            /** \brief Set whether to track approximate solutions during the search. */
            void setTrackApproximateSolutions(bool findApproximate);

            /** \brief Get whether approximate solutions are tracked during the search. */
            bool getTrackApproximateSolutions() const;

            /** \brief Set a different nearest neighbours datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors();
            //////////////////

            //////////////////
            // Get the progress property counters
            /** \brief The number of free samples (size of freeStateNN_). */
            unsigned int numFreeSamples() const;

            /** \brief The number of vertices in the tree (Size of vertexNN_). */
            unsigned int numConnectedVertices() const;

            /** \brief The \e total number of states generated (numSamples_). */
            unsigned int numStatesGenerated() const;

            /** \brief The \e total number of vertices added to the graph (numVertices_). */
            unsigned int numVerticesConnected() const;

            /** \brief The number of states pruned (numFreeStatesPruned_). */
            unsigned int numFreeStatesPruned() const;

            /** \brief The number of tree vertices disconnected (numVerticesDisconnected_). */
            unsigned int numVerticesDisconnected() const;

            /** \brief The number of nearest neighbour calls (numNearestNeighbours_). */
            unsigned int numNearestLookups() const;

            /** \brief The number of state collision checks (numStateCollisionChecks_). */
            unsigned int numStateCollisionChecks() const;
            //////////////////
            ////////////////////////////////

        private:
            ////////////////////////////////
            // High-level primitives updating the graph:
            /** \brief Update the set of free samples such that the neighbourhood of the given vertex is sufficiently
             * sampled. */
            void updateSamples(const VertexConstPtr &vertex);

            /** \brief Iterates through all the vertices in the tree and finds the one that is closes to the goal. This
             * is only necessary to find approximate solutions and should otherwise not be called. */
            void findVertexClosestToGoal();
            ////////////////////////////////

            ////////////////////////////////
            // High-level primitives pruning the graph:
            /** \brief Prune any starts/goals that provably cannot provide a better solution than the current best
             * solution. This is done via the prune conditions of the SearchQueue. Returns the number of vertices
             * disconnected and the number of samples removed. */
            std::pair<unsigned int, unsigned int> pruneStartsGoals();

            /** \brief Prune any samples that provably cannot provide a better solution than the current best solution.
             * This is done via the prune conditions of the SearchQueue. Removes the number of samples removed.*/
            unsigned int pruneSamples();
            ////////////////////////////////

            ////////////////////////////////
            // Low-level random geometric graph helper and calculations
            /** \brief Tests and updates whether the given vertex is closer to the goal than the known-closest vertex.
             * This is only necessary to find approximate solutions and should otherwise not be called. */
            void testClosestToGoal(const VertexConstPtr &newVertex);

            /** \brief Calculate the max req'd cost to define a neighbourhood around a state. Currently only implemented
             * for path-length problems, for which the neighbourhood cost is the f-value of the vertex plus 2r. */
            ompl::base::Cost neighbourhoodCost(const VertexConstPtr &vertex) const;

            /** \brief Update the appropriate nearest-neighbour terms, r_ and k_. */
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
            ////////////////////////////////

            ////////////////////////////////
            // Debug
            /** \brief Test if the class is setup and throw if not. */
            void assertSetup() const;
            ////////////////////////////////

            ////////////////////////////////
            // Variables -- Make sure every one is configured in setup() and reset in clear():
            /** \brief A function pointer to the planner name, for better OMPL_INFO, etc. output */
            NameFunc nameFunc_;

            /** \brief Whether the class is setup */
            bool isSetup_{false};

            /** \brief The state space used by the planner */
            ompl::base::SpaceInformationPtr si_{nullptr};

            /** \brief The problem definition */
            ompl::base::ProblemDefinitionPtr pdef_{nullptr};

            /** \brief A cost/heuristic helper class. As this is a copy of the version owned by BITstar.cpp it can be
             * reset in a clear(). */
            CostHelper *costHelpPtr_{nullptr};

            /** \brief The queue class. As this is a copy of the version owned by BITstar.cpp it can be reset in a
             * clear(). */
            SearchQueue *queuePtr_{nullptr};

            /** \brief An instance of a random number generator */
            ompl::RNG rng_;

            /** \brief State sampler */
            ompl::base::InformedSamplerPtr sampler_{nullptr};

            /** \brief The start states of the problem as vertices. Constructed as a shared_ptr to give easy access to
             * helper classes */
            VertexPtrVector startVertices_;

            /** \brief The goal states of the problem as vertices. Constructed as a shared_ptr to give easy access to
             * helper classes */
            VertexPtrVector goalVertices_;

            /** \brief Any start states of the problem that have been pruned */
            VertexPtrVector prunedStartVertices_;

            /** \brief Any goal states of the problem that have been pruned */
            VertexPtrVector prunedGoalVertices_;

            /** \brief A copy of the new samples from this batch */
            VertexPtrVector newSamples_;

            /** \brief A copy of the vertices recycled into samples during this batch */
            VertexPtrVector recycledSamples_;

            /** \brief The samples as a nearest-neighbours datastructure. Sorted by nnDistance. Size accessible via
             * numFreeSamples */
            VertexPtrNNPtr freeStateNN_{nullptr};

            /** \brief The vertices as a nearest-neighbours data structure. Sorted by nnDistance. Size accessible via
             * numConnectedVertices */
            VertexPtrNNPtr vertexNN_{nullptr};

            /** \brief The number of samples in this batch */
            unsigned int samplesInThisBatch_{0u};

            /** \brief The number of states (vertices or samples) that were generated from a uniform distribution. Only
             * valid when refreshSamplesOnPrune_ is true, in which case it's used to calculate the RGG term of the
             * uniform subgraph.*/
            unsigned int numUniformStates_{0u};

            /** \brief The current r-disc RGG connection radius */
            double r_{0.};

            /** \brief The minimum k-nearest RGG connection term. Only a function of state dimension, so can be
             * calculated once. Left as a double for later accuracy in calculate k */
            double k_rgg_{0.};

            /** \brief The current k-nearest RGG connection number */
            unsigned int k_{0u};

            /** \brief The measure of the continuous problem domain which we are approximating with samples. This is
             * initially the problem domain but can shrink as we focus the search. */
            double approximationMeasure_{0.};

            /** \brief The minimum possible solution cost. I.e., the heuristic value of the start. */
            ompl::base::Cost minCost_{std::numeric_limits<double>::infinity()};

            /** \brief The maximum heuristic cost to sample (i.e., the best solution found to date). */
            ompl::base::Cost maxCost_{std::numeric_limits<double>::infinity()};

            /** \brief The total-heuristic cost up to which we've sampled */
            ompl::base::Cost costSampled_{std::numeric_limits<double>::infinity()};

            /** \brief If we've found an exact solution yet */
            bool hasExactSolution_{false};

            /** \brief IF BEING TRACKED, the vertex closest to the goal (this represents an "approximate" solution) */
            VertexConstPtr closestVertexToGoal_{nullptr};

            /** \brief IF BEING TRACKED, the smallest distance of vertices in the tree to a goal (this represents
             * tolerance of an "approximate" solution) */
            double closestDistToGoal_{std::numeric_limits<double>::infinity()};
            ///////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////////////////////////
            // Informational variables - Make sure initialized in setup and reset in clear
            /** \brief The number of states generated through sampling. Accessible via numStatesGenerated */
            unsigned int numSamples_{0u};

            /** \brief The number of vertices ever added to the tree. Accessible via numVerticesConnected */
            unsigned int numVertices_{0u};

            /** \brief The number of free states that have been pruned. Accessible via numStatesPruned */
            unsigned int numFreeStatesPruned_{0u};

            /** \brief The number of graph vertices that get disconnected. Accessible via numVerticesDisconnected */
            unsigned int numVerticesDisconnected_{0u};

            /** \brief The number of nearest neighbour calls. Accessible via numNearestLookups */
            unsigned int numNearestNeighbours_{0u};

            /** \brief The number of state collision checks. Accessible via numStateCollisionChecks */
            unsigned int numStateCollisionChecks_{0u};
            ///////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////////////////////////
            // Parameters - Set defaults in construction/setup and DO NOT reset in clear.
            /** \brief The rewiring factor, s, so that r_rgg = s \times r_rgg* > r_rgg* (param) */
            double rewireFactor_{1.1};

            /** \brief Option to use k-nearest search for rewiring (param) */
            bool useKNearest_{true};

            /** \brief Whether to use just-in-time sampling (param) */
            bool useJustInTimeSampling_{false};

            /** \brief Whether to refresh (i.e., forget) unconnected samples on pruning (param) */
            bool dropSamplesOnPrune_{false};

            /** \brief Whether to consider approximate solutions (param) */
            bool findApprox_{false};
            ///////////////////////////////////////////////////////////////////
        };  // class: ImplicitGraph
    }       // geometric
}  // ompl
#endif  // OMPL_GEOMETRIC_PLANNERS_BITSTAR_DATASTRUCTURES_IMPLICITGRAPH_
