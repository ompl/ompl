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

#ifndef OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BITSTAR_
#define OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BITSTAR_

#include <string>
#include <utility>
#include <vector>

#include "ompl/base/Planner.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/datastructures/NearestNeighbors.h"

// Defining BITSTAR_DEBUG enables (significant) debug output. Do not enable unless necessary.
// #define BITSTAR_DEBUG

namespace ompl
{
    namespace geometric
    {
        /**
        @anchor gBITstar

        \ref gBITstar "BIT*" (Batch Informed Trees) is an \e anytime asymptotically optimal sampling-based planning
        algorithm. It approaches problems by assuming that a \e simple solution exists and only goes onto consider \e
        complex solutions when that proves incorrect. It accomplishes this by using heuristics to search in order of
        decreasing potential solution quality.

        Both a k-nearest and r-disc version are available, with the k-nearest selected by default. In general, the
        r-disc variant considers more connections than the k-nearest. For a small number of specific planning problems,
        this results in it finding solutions slower than k-nearest (hence the default choice). It is recommended that
        you try both variants, with the r-disc version being recommended *if* it finds an initial solution in a suitable
        amount of time (which it probably will). The difference in this number of connections considered is a RGG theory
        question, and certainly merits further review.

        This implementation of BIT* can handle multiple starts, multiple goals, a variety of optimization objectives
        (e.g., path length), and with \ref gBITstarSetJustInTimeSampling "just-in-time sampling", infinite problem
        domains. Note that for some of optimization  objectives, the user must specify a suitable heuristic and that
        when this heuristic is not specified, it will use the conservative/always admissible \e zero-heuristic.

        This implementation also includes some new advancements, including the ability to prioritize exploration until
        an initial solution is found (\ref gBITstarSetDelayRewiringUntilInitialSolution "Delayed rewiring"), the ability
        to generate samples only when necessary (\ref gBITstarSetJustInTimeSampling "Just-in-time sampling"), and the
        ability to periodically remove samples that have yet to be connected to the graph (\ref
        gBITstarSetDropSamplesOnPrune "Sample dropping"). With just-in-time sampling, BIT* can even solve planning
        problems with infinite state space boundaries, i.e., (-inf, inf).


        @par Associated publication:

        J. D. Gammell, T. D. Barfoot, S. S. Srinivasa,  "Batch Informed Trees (BIT*): Informed asymptotically optimal
        anytime search." The International Journal of Robotics Research (IJRR), 39(5): 543-567, Apr. 2020.
        DOI: <a href="https://doi.org/10.1177/0278364919890396">10.1177/0278364919890396</a>.
        arXiv: <a href="https://arxiv.org/pdf/1707.01888">1707.01888 [cs.RO]</a>.
        <a href="https://www.youtube.com/watch?v=MRzSfLpNBmA">Illustration video</a>.
        */
        /** \brief Batch Informed Trees (BIT*)*/
        class BITstar : public ompl::base::Planner
        {
        public:
            // ---
            // Forward declarations.
            // ---

            /** \brief The vertex of implicit and explicit graphs. */
            class Vertex;
            /** \brief A generator of unique vertex IDs. */
            class IdGenerator;
            /** \brief A helper class to consolidate cost and heuristic calculations. */
            class CostHelper;
            /** \brief The samples viewed as an edge-implicit random geometric graph. */
            class ImplicitGraph;
            /** \brief The queue of edges to process as a dual-stage queue (tracks both the expansion of vertices and
             * the resulting edges). */
            class SearchQueue;

            // ---
            // Aliases.
            // ---

            /** \brief A shared pointer to a vertex. */
            using VertexPtr = std::shared_ptr<Vertex>;

            /** \brief A shared pointer to a \e const vertex. */
            using VertexConstPtr = std::shared_ptr<const Vertex>;

            /** \brief A weak pointer to a vertex. */
            using VertexWeakPtr = std::weak_ptr<Vertex>;

            /** \brief A vector of shared pointers to vertices. */
            using VertexPtrVector = std::vector<VertexPtr>;

            /** \brief A vector of shared pointers to \e const vertices. */
            using VertexConstPtrVector = std::vector<VertexConstPtr>;

            /** \brief The vertex id type. */
            using VertexId = unsigned int;

            /** \brief A pair of vertices, i.e., an edge. */
            using VertexPtrPair = std::pair<VertexPtr, VertexPtr>;

            /** \brief A pair of const vertices, i.e., an edge. */
            using VertexConstPtrPair = std::pair<VertexConstPtr, VertexConstPtr>;

            /** \brief A vector of pairs of vertices, i.e., a vector of edges. */
            using VertexPtrPairVector = std::vector<VertexPtrPair>;

            /** \brief A vector of pairs of const vertices, i.e., a vector of edges. */
            using VertexConstPtrPairVector = std::vector<VertexConstPtrPair>;

            /** \brief The OMPL::NearestNeighbors structure. */
            using VertexPtrNNPtr = std::shared_ptr<NearestNeighbors<VertexPtr>>;

            /** \brief A utility functor for ImplicitGraph and SearchQueue. */
            using NameFunc = std::function<std::string()>;

            /** \brief Construct with a pointer to the space information and an optional name. */
            BITstar(const base::SpaceInformationPtr &spaceInfo, const std::string &name = "kBITstar");

            /** \brief Destruct using the default destructor. */
            virtual ~BITstar() override = default;

            /** \brief Setup the algorithm. */
            void setup() override;

            /** \brief Clear the algorithm's internal state. */
            void clear() override;

            /** \brief Solve the problem given a termination condition. */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &terminationCondition) override;

            /** \brief Get results. */
            void getPlannerData(base::PlannerData &data) const override;

            // ---
            // Debugging info.
            // ---

            /** \brief Get the next edge to be processed. Causes vertices in the queue to be expanded (if necessary) and
             * therefore effects the run timings of the algorithm, but helpful for some videos and debugging. */
            std::pair<const ompl::base::State *, const ompl::base::State *> getNextEdgeInQueue();

            /** \brief Get the value of the next edge to be processed. Causes vertices in the queue to be expanded (if
             * necessary) and therefore effects the run timings of the algorithm, but helpful for some videos and
             * debugging. */
            ompl::base::Cost getNextEdgeValueInQueue();

            /** \brief Get the whole messy set of edges in the queue. Expensive but helpful for some videos. */
            void getEdgeQueue(VertexConstPtrPairVector *edgesInQueue);

            /** \brief Get the number of iterations completed. */
            unsigned int numIterations() const;

            /** \brief Retrieve the best exact-solution cost found.*/
            ompl::base::Cost bestCost() const;

            /** \brief Retrieve the number of batches processed as the raw data. */
            unsigned int numBatches() const;

            // ---
            // Settings.
            // ---

            /** \brief Set the rewiring scale factor, s, such that r_rrg = s \times r_rrg*. */
            void setRewireFactor(double rewireFactor);

            /** \brief Get the rewiring scale factor. */
            double getRewireFactor() const;

            /** \brief Set the average number of allowed failed attempts when sampling. */
            void setAverageNumOfAllowedFailedAttemptsWhenSampling(std::size_t number);

            /** \brief Get the average number of allowed failed attempts when sampling. */
            std::size_t getAverageNumOfAllowedFailedAttemptsWhenSampling() const;

            /** \brief Set the number of samplers per batch. */
            void setSamplesPerBatch(unsigned int n);

            /** \brief Get the number of samplers per batch. */
            unsigned int getSamplesPerBatch() const;

            /** \brief Enable a k-nearest search for instead of an r-disc search. */
            void setUseKNearest(bool useKNearest);

            /** \brief Get whether a k-nearest search is being used.*/
            bool getUseKNearest() const;

            /** \brief Enable "strict sorting" of the edge queue. Rewirings can change the position in the queue of an
             * edge. When strict sorting is enabled, the effected edges are resorted immediately, while disabling strict
             * sorting delays this resorting until the end of the batch. */
            void setStrictQueueOrdering(bool beStrict);

            /** \brief Get whether strict queue ordering is in use. */
            bool getStrictQueueOrdering() const;

            /** \brief Enable pruning of vertices/samples that CANNOT improve the current solution. When a vertex in the
             * graph is pruned, it's descendents are also pruned (if they also cannot improve the solution) or placed
             * back in the set of free samples (if they could improve the solution). This assures that a uniform density
             * is maintained. */
            void setPruning(bool prune);

            /** \brief Get whether graph and sample pruning is in use. */
            bool getPruning() const;

            /** \brief Set the fractional change in the solution cost AND problem measure necessary for pruning to
             * occur. */
            void setPruneThresholdFraction(double fractionalChange);

            /** \brief Get the fractional change in the solution cost AND problem measure necessary for pruning to
             * occur. */
            double getPruneThresholdFraction() const;

            /** @anchor gBITstarSetDelayRewiringUntilInitialSolution \brief Delay the consideration of rewiring edges
             * until an initial solution is found. When multiple batches are required to find an initial solution, this
             * can improve the time required to do so, by delaying improvements in the cost-to-come to a connected
             * vertex. As the rewiring edges are considered once an initial solution is found, this has no effect on the
             * theoretical asymptotic optimality of the planner. */
            void setDelayRewiringUntilInitialSolution(bool delayRewiring);

            /** \brief Get whether BIT* is delaying rewiring until a solution is found. */
            bool getDelayRewiringUntilInitialSolution() const;

            /** @anchor gBITstarSetJustInTimeSampling \brief Delay the generation of samples until they are \e
             * necessary. This only works when using an r-disc connection scheme, and is currently only implemented for
             * problems seeking to minimize path length. This helps reduce the complexity of nearest-neighbour look ups,
             * and can be particularly beneficial in unbounded planning problems where selecting an appropriate bounding
             * box is difficult. With JIT sampling enabled, BIT* can solve planning problems whose state space has \e
             * infinite \e boundaries. When enumerating outgoing edges from a vertex, BIT* uses JIT sampling to assure
             * that the area within r of the vertex has been sampled during this batch. This is done in a way that
             * maintains uniform sample distribution and has no effect on the theoretical asymptotic optimality of the
             * planner. */
            void setJustInTimeSampling(bool useJit);

            /** \brief Get whether we're using just-in-time sampling. */
            bool getJustInTimeSampling() const;

            /** @anchor gBITstarSetDropSamplesOnPrune \brief Drop \e all unconnected samples when pruning, regardless of
             * their heuristic value. This provides a method for BIT* to remove samples that have not been connected to
             * the graph and may be beneficial in problems where portions of the free space are unreachable (i.e.,
             * disconnected). BIT* calculates the connection radius for each batch from the underlying uniform
             * distribution of states. The resulting larger connection radius may be detrimental in areas where the
             * graph is dense, but maintains the theoretical asymptotic optimality of the planner. */
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

            /** \brief Set a different nearest neighbours datastructure. */
            template <template <typename T> class NN>
            void setNearestNeighbors();

        protected:
            // ---
            // The settings that turn BIT* into ABIT*.
            // ---

            /** \brief Set the inflation for the initial search of RGG approximation. See ABIT*'s class description for
             * more details about the inflation factor update policy. */
            void setInitialInflationFactor(double factor);

            /** \brief The parameter that scales the inflation factor on the second search of each RGG approximation.
             * See ABIT*'s class description for more details about the inflation factor update policy. */
            void setInflationScalingParameter(double parameter);

            /** \brief Sets the parameter that scales the truncation factor for the searches of each RGG approximation.
             * See ABIT*'s class description for more details about the truncation factor update policy. */
            void setTruncationScalingParameter(double parameter);

            /** \brief Enable the cascading of rewirings. */
            void enableCascadingRewirings(bool enable);

            // ---
            // Getters specific to ABIT*.
            // ---

            /** \brief Get the inflation factor for the initial search. */
            double getInitialInflationFactor() const;

            /** \brief Get the inflation scaling parameter. */
            double getInflationScalingParameter() const;

            /** \brief Get the truncation factor parameter. */
            double getTruncationScalingParameter() const;

            /** \brief Get the inflation factor for the current search. */
            double getCurrentInflationFactor() const;

            /** \brief Get the truncation factor for the current search. */
            double getCurrentTruncationFactor() const;

        private:
            // ---
            // High level primitives.
            // ---

            /** \brief A single iteration. */
            void iterate();

            /** \brief Initialize variables for a new batch. */
            void newBatch();

            /** \brief Prune the problem. */
            void prune();

            /** \brief Publish the found solution to the ProblemDefinition. */
            void publishSolution();

            // ---
            // Low level primitives.
            // ---

            /** \brief Extract the best solution, ordered \e from the goal to the \e start and including both the goal
             * and the start. Used by both publishSolution and the ProblemDefinition::IntermediateSolutionCallback. */
            std::vector<const ompl::base::State *> bestPathFromGoalToStart() const;

            /** \brief Checks an edge for collision. A wrapper to SpaceInformation->checkMotion that tracks number of
             * collision checks. */
            bool checkEdge(const VertexConstPtrPair &edge);

            /** \brief Blacklists an edge (useful if an edge is in collision). */
            void blacklistEdge(const VertexPtrPair &edge) const;

            /** \brief Whitelists an edge (useful if an edge not in collision). */
            void whitelistEdge(const VertexPtrPair &edge) const;

            /** \brief Add an edge from the edge queue to the tree. Will add the state to the vertex queue if it's new
             * to the tree or otherwise replace the parent. Updates solution information if the solution improves. */
            void addEdge(const VertexPtrPair &edge, const ompl::base::Cost &edgeCost);

            /** \brief Replace the parent edge with the given new edge and cost */
            void replaceParent(const VertexPtrPair &edge, const ompl::base::Cost &edgeCost);

            /** \brief The special work that needs to be done to update the goal vertex if the solution has changed. */
            void updateGoalVertex();

            // ---
            // Logging.
            // ---

            // Helper functions for logging
            /** \brief The message printed when a goal is found/improved. */
            void goalMessage() const;

            /** \brief The message printed when solve finishes successfully. */
            void endSuccessMessage() const;

            /** \brief The message printed when solve finishes unsuccessfully. */
            void endFailureMessage() const;

            /** \brief A detailed status message format with debug info. */
            void statusMessage(const ompl::msg::LogLevel &logLevel, const std::string &status) const;

            // ---
            // Progress properties.
            // ---

            /** \brief Retrieve the best exact-solution cost found as a planner-progress property. */
            std::string bestCostProgressProperty() const;

            /** \brief Retrieve the length of the best exact-solution found as a planner-progress property.
             * (bestLength_) */
            std::string bestLengthProgressProperty() const;

            /** \brief Retrieve the current number of free samples as a planner-progress property. (Size of the free
             * states in graphPtr_.) */
            std::string currentFreeProgressProperty() const;

            /** \brief Retrieve the current number of vertices in the graph as a planner-progress property. (Size of the
             * connected states in graphPtr_.) */
            std::string currentVertexProgressProperty() const;

            /** \brief Retrieve the current number of edges in the search queue as a planner-progress property. (The
             * size of the edge subqueue of queuePtr_.) */
            std::string edgeQueueSizeProgressProperty() const;

            /** \brief Retrieve the number of iterations as a planner-progress property. (numIterations_) */
            std::string iterationProgressProperty() const;

            /** \brief Retrieve the number of batches processed as a planner-progress property. (numBatches_) */
            std::string batchesProgressProperty() const;

            /** \brief Retrieve the number of graph prunings performed as a planner-progress property. (numPrunings_) */
            std::string pruningProgressProperty() const;

            /** \brief Retrieve the \e total number of states generated as a planner-progress property. (From graphPtr_)
             */
            std::string totalStatesCreatedProgressProperty() const;

            /** \brief Retrieve the \e total number of vertices added to the graph as a planner-progress property. (From
             * graphPtr_) */
            std::string verticesConstructedProgressProperty() const;

            /** \brief Retrieve the number of states pruned from the problem as a planner-progress property. (From
             * graphPtr_) */
            std::string statesPrunedProgressProperty() const;

            /** \brief Retrieve the number of graph vertices that are disconnected and either returned to the set of
             * free samples or deleted completely as a planner-progress property. (From graphPtr_) */
            std::string verticesDisconnectedProgressProperty() const;

            /** \brief Retrieve the number of global-search edges that rewired the graph as a planner-progress property.
             * (numRewirings_) */
            std::string rewiringProgressProperty() const;

            /** \brief Retrieve the number of state collisions checks (i.e., calls to SpaceInformation::isValid(...)) as
             * a planner-progress property. (From graphPtr_) */
            std::string stateCollisionCheckProgressProperty() const;

            /** \brief Retrieve the number of edge (or motion) collision checks (i.e., calls to
             * SpaceInformation::checkMotion(...)) as a planner-progress property. (numEdgeCollisionChecks_) */
            std::string edgeCollisionCheckProgressProperty() const;

            /** \brief Retrieve the number of nearest neighbour calls (i.e., NearestNeighbors<T>::nearestK(...) or
             * NearestNeighbors<T>::nearestR(...)) as a planner-progress property. (From graphPtr_) */
            std::string nearestNeighbourProgressProperty() const;

            /** \brief Retrieve the total number of edges processed from the queue as a planner-progress property. (From
             * queuePtr_) */
            std::string edgesProcessedProgressProperty() const;

            // ---
            // Member variables (Make all are configured in setup() and reset in reset()).
            // ---

            /** \brief A helper for cost and heuristic calculations. */
            std::shared_ptr<CostHelper> costHelpPtr_{nullptr};

            /** \brief The samples represented as an edge-implicit graph. */
            std::shared_ptr<ImplicitGraph> graphPtr_{nullptr};

            /** \brief The queue of vertices to expand and edges to process ordered on "f-value", i.e., estimated
             * solution cost. */
            std::shared_ptr<SearchQueue> queuePtr_{nullptr};

            /** \brief The inflation factor for the initial search. */
            double initialInflationFactor_{1.0};

            /** \brief The truncation factor for the search. */
            double truncationFactor_{1.0};

            /** \brief The truncation factor parameter for the update policy. */
            double truncationScalingParameter_{0.0};

            /** \brief The inflation factor parameter for the update policy. */
            double inflationScalingParameter_{0.0};

            /** \brief The goal vertex of the current best solution. */
            VertexConstPtr curGoalVertex_{nullptr};

            /** \brief The best cost found to date. This is the maximum total-heuristic cost of samples we'll consider.
             * Accessible via bestCostProgressProperty */
            // Gets set in setup to the proper calls from OptimizationObjective
            ompl::base::Cost bestCost_;

            /** \brief The number of vertices in the best solution found to date. Accessible via
             * bestLengthProgressProperty. */
            unsigned int bestLength_{0u};

            /** \brief The cost to which the problem has been pruned. We will only prune the graph when a new solution
             * is sufficiently less than this value. */
            // Gets set in setup to the proper calls from OptimizationObjective.
            ompl::base::Cost prunedCost_{std::numeric_limits<double>::infinity()};

            /** \brief The measure to which the problem has been pruned. We will only prune the graph when the resulting
             * measure of a new solution is sufficiently less than this value. */
            // Gets set in setup with the proper call to Planner::si_->getSpaceMeasure()
            double prunedMeasure_{0.0};

            bool isFinalSearchOnBatch_{false};

            /** \brief If we've found an exact solution yet. */
            bool hasExactSolution_{false};

            /** \brief The flag whether the current search is done. */
            bool isSearchDone_{false};

            /** \brief A manual stop on the solve loop. */
            bool stopLoop_{false};

            /** \brief The number of batches processed. Accessible via batchesProgressProperty. */
            unsigned int numBatches_{0u};

            /** \brief The number of times the graph/samples have been pruned. Accessible via pruningProgressProperty.
             */
            unsigned int numPrunings_{0u};

            /** \brief The number of iterations run. Accessible via iterationProgressProperty. */
            unsigned int numIterations_{0u};

            /** \brief The number of times a state in the graph was rewired. Accessible via rewiringProgressProperty. */
            unsigned int numRewirings_{0u};

            /** \brief The number of edge collision checks. Accessible via edgeCollisionCheckProgressProperty. */
            unsigned int numEdgeCollisionChecks_{0u};

            // ---
            // Parameters - Set defaults in construction/setup and do not reset in clear.
            // ---

            /** \brief The number of samples per batch. */
            unsigned int samplesPerBatch_{100u};

            /** \brief Whether to use graph pruning. */
            bool isPruningEnabled_{true};

            /** \brief The fractional decrease in solution cost required to trigger pruning. */
            double pruneFraction_{0.05};

            /** \brief Whether to stop the planner as soon as the path changes. */
            bool stopOnSolutionChange_{false};
        };  // class BITstar
    }       // namespace geometric
}  // namespace ompl

#endif  // OMPL_GEOMETRIC_PLANNERS_INFORMEDTREES_BITSTAR_
