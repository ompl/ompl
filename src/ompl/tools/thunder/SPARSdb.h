/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
*  All Rights Reserved.
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
*   * Neither the name of Rutgers University nor the names of its
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

/* Author: Andrew Dobson, Dave Coleman */

#ifndef OMPL_TOOLS_THUNDER_SPARS_DB_
#define OMPL_TOOLS_THUNDER_SPARS_DB_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/util/Time.h"
#include "ompl/util/Hash.h"

#include <boost/range/adaptor/map.hpp>
#include <unordered_map>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <thread>
#include <iostream>
#include <fstream>
#include <utility>
#include <vector>
#include <map>

namespace ompl
{
    namespace geometric
    {
        /**
           @anchor gSPARSdb
           @par Short description
           SPARSdb is a variant of the SPARS algorithm which removes the
           dependency on having the dense graph, D.  It works through similar
           mechanics, but uses a different approach to identifying interfaces
           and computing shortest paths through said interfaces.

           This version has been modified for use with Thunder

           @par External documentation
           A. Dobson, K. Bekris,
           Improving Sparse Roadmap Spanners,
           <em>IEEE International Conference on Robotics and Automation (ICRA)</em> May 2013.
           <a href="http://www.cs.rutgers.edu/~kb572/pubs/spars2.pdf">[PDF]</a>
        */

        /** \brief <b> SPArse Roadmap Spanner Version 2.0 </b> */
        class SPARSdb : public base::Planner
        {
        public:
            /** \brief Enumeration which specifies the reason a guard is added to the spanner. */
            enum GuardType
            {
                START,
                GOAL,
                COVERAGE,
                CONNECTIVITY,
                INTERFACE,
                QUALITY,
            };

            ////////////////////////////////////////////////////////////////////////////////////////
            // BOOST GRAPH DETAILS
            ////////////////////////////////////////////////////////////////////////////////////////

            /** \brief The type used internally for representing vertex IDs */
            using VertexIndexType = unsigned long int;

            /** \brief Pair of vertices which support an interface. */
            using VertexPair = std::pair<VertexIndexType, VertexIndexType>;

            ////////////////////////////////////////////////////////////////////////////////////////
            /** \brief Interface information storage class, which does bookkeeping for criterion four. */
            struct InterfaceData
            {
                /** \brief States which lie inside the visibility region of a vertex and support an interface. */
                base::State *pointA_{nullptr};
                base::State *pointB_{nullptr};

                /** \brief States which lie just outside the visibility region of a vertex and support an interface. */
                base::State *sigmaA_{nullptr};
                base::State *sigmaB_{nullptr};

                /** \brief Last known distance between the two interfaces supported by points_ and sigmas. */
                double d_{std::numeric_limits<double>::infinity()};

                /** \brief Constructor */
                InterfaceData() = default;

                /** \brief Clears the given interface data. */
                void clear(const base::SpaceInformationPtr &si)
                {
                    if (pointA_ != nullptr)
                    {
                        si->freeState(pointA_);
                        pointA_ = nullptr;
                    }
                    if (pointB_ != nullptr)
                    {
                        si->freeState(pointB_);
                        pointB_ = nullptr;
                    }
                    if (sigmaA_ != nullptr)
                    {
                        si->freeState(sigmaA_);
                        sigmaA_ = nullptr;
                    }
                    if (sigmaB_ != nullptr)
                    {
                        si->freeState(sigmaB_);
                        sigmaB_ = nullptr;
                    }
                    d_ = std::numeric_limits<double>::infinity();
                }

                /** \brief Sets information for the first interface (i.e. interface with smaller index vertex). */
                void setFirst(const base::State *p, const base::State *s, const base::SpaceInformationPtr &si)
                {
                    if (pointA_ != nullptr)
                        si->copyState(pointA_, p);
                    else
                        pointA_ = si->cloneState(p);
                    if (sigmaA_ != nullptr)
                        si->copyState(sigmaA_, s);
                    else
                        sigmaA_ = si->cloneState(s);
                    if (pointB_ != nullptr)
                        d_ = si->distance(pointA_, pointB_);
                }

                /** \brief Sets information for the second interface (i.e. interface with larger index vertex). */
                void setSecond(const base::State *p, const base::State *s, const base::SpaceInformationPtr &si)
                {
                    if (pointB_ != nullptr)
                        si->copyState(pointB_, p);
                    else
                        pointB_ = si->cloneState(p);
                    if (sigmaB_ != nullptr)
                        si->copyState(sigmaB_, s);
                    else
                        sigmaB_ = si->cloneState(s);
                    if (pointA_ != nullptr)
                        d_ = si->distance(pointA_, pointB_);
                }
            };

            /** \brief the hash which maps pairs of neighbor points to pairs of states */
            using InterfaceHash = std::unordered_map<VertexPair, InterfaceData>;

            ////////////////////////////////////////////////////////////////////////////////////////
            // The InterfaceHash structure is wrapped inside of this struct due to a compilation error on
            // GCC 4.6 with Boost 1.48.  An implicit assignment operator overload does not compile with these
            // components, so an explicit overload is given here.
            // Remove this struct when the minimum Boost requirement is > v1.48.
            struct InterfaceHashStruct
            {
                InterfaceHashStruct &operator=(const InterfaceHashStruct &rhs) = default;
                InterfaceHash interfaceHash;
            };

            ////////////////////////////////////////////////////////////////////////////////////////
            // Vertex properties

            struct vertex_state_t
            {
                using kind = boost::vertex_property_tag;
            };

            struct vertex_color_t
            {
                using kind = boost::vertex_property_tag;
            };

            struct vertex_interface_data_t
            {
                using kind = boost::vertex_property_tag;
            };

            ////////////////////////////////////////////////////////////////////////////////////////
            // Edge properties

            struct edge_collision_state_t
            {
                using kind = boost::edge_property_tag;
            };

            /** \brief Possible collision states of an edge */
            enum EdgeCollisionState
            {
                NOT_CHECKED,
                IN_COLLISION,
                FREE
            };

            /** \brief Struct for passing around partially solved solutions */
            struct CandidateSolution
            {
                bool isApproximate_;
                base::PathPtr path_;
                // Edge 0 is edge from vertex 0 to vertex 1. Thus, there is n-1 edges for n vertices
                std::vector<EdgeCollisionState> edgeCollisionStatus_;
                // TODO save the collision state of the vertexes also?

                std::size_t getStateCount()
                {
                    return static_cast<ompl::geometric::PathGeometric &>(*path_).getStateCount();
                }

                ompl::geometric::PathGeometric &getGeometricPath()
                {
                    return static_cast<ompl::geometric::PathGeometric &>(*path_);
                }
            };

            ////////////////////////////////////////////////////////////////////////////////////////
            /**
             @brief The underlying roadmap graph.

             @par Any BGL graph representation could be used here. Because we
             expect the roadmap to be sparse (m<n^2), an adjacency_list is more
             appropriate than an adjacency_matrix. Edges are undirected.

             *Properties of vertices*
             - vertex_state_t: an ompl::base::State* is required for OMPL
             - vertex_predecessor_t: The incremental connected components algorithm requires it
             - vertex_rank_t: The incremental connected components algorithm requires it
             - vertex_color_t - TODO
             - vertex_interface_data_t - needed by SPARS2 for maintainings its sparse properties

             Note: If boost::vecS is not used for vertex storage, then there must also
             be a boost:vertex_index_t property manually added.

             *Properties of edges*
             - edge_weight_t - cost/distance between two vertices
             - edge_collision_state_t - used for lazy collision checking, determines if an edge has been checked
                  already for collision. 0 = not checked/unknown, 1 = in collision, 2 = free
             */

            /** Wrapper for the vertex's multiple as its property. */
            using VertexProperties = boost::property<
                vertex_state_t, base::State *,
                boost::property<
                    boost::vertex_predecessor_t, VertexIndexType,
                    boost::property<boost::vertex_rank_t, VertexIndexType,
                                    boost::property<vertex_color_t, GuardType,
                                                    boost::property<vertex_interface_data_t, InterfaceHashStruct>>>>>;

            /** Wrapper for the double assigned to an edge as its weight property. */
            using EdgeProperties =
                boost::property<boost::edge_weight_t, double, boost::property<edge_collision_state_t, int>>;

            /** The underlying boost graph type (undirected weighted-edge adjacency list with above properties). */
            using Graph = boost::adjacency_list<boost::vecS,  // store in std::vector
                                                boost::vecS,  // store in std::vector
                                                boost::undirectedS, VertexProperties, EdgeProperties>;

            /** \brief Vertex in Graph */
            using Vertex = boost::graph_traits<Graph>::vertex_descriptor;

            /** \brief Edge in Graph */
            using Edge = boost::graph_traits<Graph>::edge_descriptor;

            ////////////////////////////////////////////////////////////////////////////////////////
            // Typedefs for property maps

            /** \brief Access map that stores the lazy collision checking status of each edge */
            using EdgeCollisionStateMap = boost::property_map<Graph, edge_collision_state_t>::type;

            ////////////////////////////////////////////////////////////////////////////////////////
            /**
             * Used to artifically supress edges during A* search.
             * @implements ReadablePropertyMapConcept
             */
            class edgeWeightMap
            {
            private:
                const Graph &g_;  // Graph used
                const EdgeCollisionStateMap &collisionStates_;

            public:
                /** Map key type. */
                using key_type = Edge;
                /** Map value type. */
                using value_type = double;
                /** Map auxiliary value type. */
                using reference = double &;
                /** Map type. */
                using category = boost::readable_property_map_tag;

                /**
                 * Construct map for certain constraints.
                 * @param graph         Graph to use
                 */
                edgeWeightMap(const Graph &graph, const EdgeCollisionStateMap &collisionStates);

                /**
                 * Get the weight of an edge.
                 * @param e     the edge
                 * @return infinity if \a e lies in a forbidden neighborhood; actual weight of \a e otherwise
                 */
                double get(Edge e) const;
            };

            ////////////////////////////////////////////////////////////////////////////////////////
            /**
             * Thrown to stop the A* search when finished.
             */
            class foundGoalException
            {
            };

            ////////////////////////////////////////////////////////////////////////////////////////
            /**
             * Vertex visitor to check if A* search is finished.
             * @implements AStarVisitorConcept
             */
            class CustomVisitor : public boost::default_astar_visitor
            {
            private:
                Vertex goal;  // Goal Vertex of the search

            public:
                /**
                 * Construct a visitor for a given search.
                 * @param goal  goal vertex of the search
                 */
                CustomVisitor(Vertex goal);

                /**
                 * Check if we have arrived at the goal.
                 * @param u current vertex
                 * @param g graph we are searching on
                 * @throw foundGoalException if \a u is the goal
                 */
                void examine_vertex(Vertex u, const Graph &g) const;
            };

            ////////////////////////////////////////////////////////////////////////////////////////
            // SPARS MEMBER FUNCTIONS
            ////////////////////////////////////////////////////////////////////////////////////////

            /** \brief Constructor */
            SPARSdb(const base::SpaceInformationPtr &si);

            /** \brief Destructor */
            ~SPARSdb() override;

            void setProblemDefinition(const base::ProblemDefinitionPtr &pdef) override;

            /** \brief Sets the stretch factor */
            void setStretchFactor(double t)
            {
                stretchFactor_ = t;
            }

            /** \brief Sets vertex visibility range as a fraction of max. extent. */
            void setSparseDeltaFraction(double D)
            {
                sparseDeltaFraction_ = D;
                if (sparseDelta_ > 0.0)  // setup was previously called
                    sparseDelta_ = D * si_->getMaximumExtent();
            }

            /** \brief Sets interface support tolerance as a fraction of max. extent. */
            void setDenseDeltaFraction(double d)
            {
                denseDeltaFraction_ = d;
                if (denseDelta_ > 0.0)  // setup was previously called
                    denseDelta_ = d * si_->getMaximumExtent();
            }

            /** \brief Sets the maximum failures until termination */
            void setMaxFailures(unsigned int m)
            {
                maxFailures_ = m;
            }

            /** \brief Retrieve the maximum consecutive failure limit. */
            unsigned int getMaxFailures() const
            {
                return maxFailures_;
            }

            /** \brief Retrieve the dense graph interface support delta. */
            double getDenseDeltaFraction() const
            {
                return denseDeltaFraction_;
            }

            /** \brief Retrieve the sparse graph visibility range delta. */
            double getSparseDeltaFraction() const
            {
                return sparseDeltaFraction_;
            }

            /** \brief Retrieve the spanner's set stretch factor. */
            double getStretchFactor() const
            {
                return stretchFactor_;
            }

            bool getGuardSpacingFactor(double pathLength, double &numGuards, double &spacingFactor);

            /**
             * \brief Calculate the distance that should be used in inserting nodes into the db
             * \param path length - from the trajectory
             * \param num guards - the output result
             * \param spacing factor - what fraction of the sparsedelta should be used in placing guards
             * \return
             */
            bool getGuardSpacingFactor(double pathLength, int &numGuards, double &spacingFactor);

            bool addPathToRoadmap(const base::PlannerTerminationCondition &ptc,
                                  ompl::geometric::PathGeometric &solutionPath);

            bool checkStartGoalConnection(ompl::geometric::PathGeometric &solutionPath);

            bool addStateToRoadmap(const base::PlannerTerminationCondition &ptc, base::State *newState);

            /** \brief Function that can solve the motion planning
                problem. This function can be called multiple times on
                the same problem, without calling clear() in
                between. This allows the planner to continue work for
                more time on an unsolved problem, for example. Start
                and goal states from the currently specified
                ProblemDefinition are cached. This means that between
                calls to solve(), input states are only added, not
                removed. When using PRM as a multi-query planner, the
                input states should be however cleared, without
                clearing the roadmap itself. This can be done using
                the clearQuery() function. */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            /** \brief Clear the query previously loaded from the ProblemDefinition.
                Subsequent calls to solve() will reuse the previously computed roadmap,
                but will clear the set of input states constructed by the previous call to solve().
                This enables multi-query functionality for PRM. */
            void clearQuery() override;

            void clear() override;

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Vertex>>();
                if (isSetup())
                    setup();
            }

            /**
             * \brief Search the roadmap for the best path close to the given start and goal states that is valid
             * \param nearestK - unused
             * \param start
             * \param goal
             * \param geometricSolution - the resulting path
             * \return
             */
            bool getSimilarPaths(int nearestK, const base::State *start, const base::State *goal,
                                 CandidateSolution &candidateSolution, const base::PlannerTerminationCondition &ptc);

            void setup() override;

            /** \brief Retrieve the computed roadmap. */
            const Graph &getRoadmap() const
            {
                return g_;
            }

            /** \brief Get the number of vertices in the sparse roadmap. */
            unsigned int getNumVertices() const
            {
                return boost::num_vertices(g_);
            }

            /** \brief Get the number of edges in the sparse roadmap. */
            unsigned int getNumEdges() const
            {
                return boost::num_edges(g_);
            }

            /** \brief Get the number of disjoint sets in the sparse roadmap. */
            unsigned int getNumConnectedComponents() const
            {
                // Make sure graph is populated
                if (getNumVertices() == 0u)
                    return 0;

                std::vector<int> components(boost::num_vertices(g_));

                // it always overcounts by 1, i think because it is missing vertex 0 which is the new state insertion
                // component
                return boost::connected_components(g_, &components[0]) - 1;
            }

            /** \brief Get the number of times a path was inserted into the database and it failed to have connectivity
             */
            unsigned int getNumPathInsertionFailed() const
            {
                return numPathInsertionFailures_;
            }

            /** \brief description */
            unsigned int getNumConsecutiveFailures() const
            {
                return consecutiveFailures_;
            }

            /** \brief Get the number of iterations the algorithm performed */
            long unsigned int getIterations() const
            {
                return iterations_;
            }

            /**
             * \brief Convert astar results to correctly ordered path
             * \param vertexPath - in reverse
             * \param start - actual start that is probably not included in new path
             * \param goal - actual goal that is probably not included in new path
             * \param path - returned solution
             * \param disableCollisionWarning - if the func should ignore edges that are not checked
             * \return true on success
             */
            bool convertVertexPathToStatePath(std::vector<Vertex> &vertexPath, const base::State *actualStart,
                                              const base::State *actualGoal, CandidateSolution &candidateSolution,
                                              bool disableCollisionWarning = false);

            void getPlannerData(base::PlannerData &data) const override;

            /**
             * \brief Set the sparse graph from file
             * \param a pre-built graph
             */
            void setPlannerData(const base::PlannerData &data);

            /** \brief Returns whether we have reached the iteration failures limit, maxFailures_ */
            bool reachedFailureLimit() const;

            /** \brief Print debug information about planner */
            void printDebug(std::ostream &out = std::cout) const;

            /** \brief Clear all past edge state information about in collision or not */
            void clearEdgeCollisionStates();

        protected:
            /** \brief Free all the memory allocated by the planner */
            void freeMemory();

            /** \brief Check that the query vertex is initialized (used for internal nearest neighbor searches) */
            void checkQueryStateInitialization();

            /** \brief Checks to see if the sample needs to be added to ensure coverage of the space */
            bool checkAddCoverage(const base::State *qNew, std::vector<Vertex> &visibleNeighborhood);

            /** \brief Checks to see if the sample needs to be added to ensure connectivity */
            bool checkAddConnectivity(const base::State *qNew, std::vector<Vertex> &visibleNeighborhood);

            /** \brief Checks to see if the current sample reveals the existence of an interface, and if so, tries to
             * bridge it. */
            bool checkAddInterface(const base::State *qNew, std::vector<Vertex> &graphNeighborhood,
                                   std::vector<Vertex> &visibleNeighborhood);

            /** \brief Checks vertex v for short paths through its region and adds when appropriate. */
            bool checkAddPath(Vertex v);

            /** \brief A reset function for resetting the failures count */
            void resetFailures();

            /** \brief Finds visible nodes in the graph near state */
            void findGraphNeighbors(base::State *state, std::vector<Vertex> &graphNeighborhood,
                                    std::vector<Vertex> &visibleNeighborhood);

            /**
             * \brief Finds nodes in the graph near state NOTE: note tested for visibility
             * \param state - vertex to find neighbors around
             * \param result
             * \return false is no neighbors found
             */
            bool findGraphNeighbors(const base::State *state, std::vector<Vertex> &graphNeighborhood);

            /** \brief Approaches the graph from a given vertex */
            void approachGraph(Vertex v);

            /** \brief Finds the representative of the input state, st  */
            Vertex findGraphRepresentative(base::State *st);

            /** \brief Finds representatives of samples near qNew_ which are not his representative */
            void findCloseRepresentatives(base::State *workState, const base::State *qNew, Vertex qRep,
                                          std::map<Vertex, base::State *> &closeRepresentatives,
                                          const base::PlannerTerminationCondition &ptc);

            /** \brief High-level method which updates pair point information for repV_ with neighbor r */
            void updatePairPoints(Vertex rep, const base::State *q, Vertex r, const base::State *s);

            /** \brief Computes all nodes which qualify as a candidate v" for v and vp */
            void computeVPP(Vertex v, Vertex vp, std::vector<Vertex> &VPPs);

            /** \brief Computes all nodes which qualify as a candidate x for v, v', and v" */
            void computeX(Vertex v, Vertex vp, Vertex vpp, std::vector<Vertex> &Xs);

            /** \brief Rectifies indexing order for accessing the vertex data */
            VertexPair index(Vertex vp, Vertex vpp);

            /** \brief Retrieves the Vertex data associated with v,vp,vpp */
            InterfaceData &getData(Vertex v, Vertex vp, Vertex vpp);

            /** \brief Performs distance checking for the candidate new state, q against the current information */
            void distanceCheck(Vertex rep, const base::State *q, Vertex r, const base::State *s, Vertex rp);

            /** \brief When a new guard is added at state st, finds all guards who must abandon their interface
             * information and deletes that information */
            void abandonLists(base::State *st);

            /** \brief Construct a guard for a given state (\e state) and store it in the nearest neighbors data
             * structure */
            Vertex addGuard(base::State *state, GuardType type);

            /** \brief Connect two guards in the roadmap */
            void connectGuards(Vertex v, Vertex vp);

            /** \brief Check if there exists a solution, i.e., there exists a pair of milestones such that the first is
             * in \e start and the second is in \e goal, and the two milestones are in the same connected component. If
             * a solution is found, the path is saved. */
            bool getPaths(const std::vector<Vertex> &candidateStarts, const std::vector<Vertex> &candidateGoals,
                          const base::State *actualStart, const base::State *actualGoal,
                          CandidateSolution &candidateSolution, const base::PlannerTerminationCondition &ptc);

            /**
             * \brief Repeatidly search through graph for connection then check for collisions then repeat
             * \return true if a valid path is found
             */
            bool lazyCollisionSearch(const Vertex &start, const Vertex &goal, const base::State *actualStart,
                                     const base::State *actualGoal, CandidateSolution &candidateSolution,
                                     const base::PlannerTerminationCondition &ptc);

            /** \brief Check recalled path for collision and disable as needed */
            bool lazyCollisionCheck(std::vector<Vertex> &vertexPath, const base::PlannerTerminationCondition &ptc);

            /** Thread that checks for solution */
            void checkForSolution(const base::PlannerTerminationCondition &ptc, base::PathPtr &solution);

            /** \brief Given two milestones from the same connected component, construct a path connecting them and set
             * it as the solution
             *  \param start
             *  \param goal
             *  \param vertexPath
             *  \return true if candidate solution found
             */
            bool constructSolution(Vertex start, Vertex goal, std::vector<Vertex> &vertexPath) const;

            /** \brief Check if two milestones (\e m1 and \e m2) are part of the same connected component. This is not a
             * const function since we use incremental connected components from boost */
            bool sameComponent(Vertex m1, Vertex m2);

            /** \brief Compute distance between two milestones (this is simply distance between the states of the
             * milestones) */
            double distanceFunction(const Vertex a, const Vertex b) const
            {
                return si_->distance(stateProperty_[a], stateProperty_[b]);
            }

            /** \brief Sampler user for generating valid samples in the state space */
            base::ValidStateSamplerPtr sampler_;

            /** \brief Nearest neighbors data structure */
            std::shared_ptr<NearestNeighbors<Vertex>> nn_;

            /** \brief Connectivity graph */
            Graph g_;

            /** \brief Array of start milestones */
            std::vector<Vertex> startM_;

            /** \brief Array of goal milestones */
            std::vector<Vertex> goalM_;

            /** \brief Vertex for performing nearest neighbor queries. */
            Vertex queryVertex_;

            /** \brief Stretch Factor as per graph spanner literature (multiplicative bound on path quality) */
            double stretchFactor_{3.};

            /** \brief Maximum visibility range for nodes in the graph as a fraction of maximum extent. */
            double sparseDeltaFraction_{.25};

            /** \brief Maximum range for allowing two samples to support an interface as a fraction of maximum extent.
             */
            double denseDeltaFraction_{.001};

            /** \brief The number of consecutive failures to add to the graph before termination */
            unsigned int maxFailures_{5000u};

            /** \brief Track how many solutions fail to have connectivity at end */
            unsigned int numPathInsertionFailures_{0u};

            /** \brief Number of sample points to use when trying to detect interfaces. */
            unsigned int nearSamplePoints_;

            /** \brief A path simplifier used to simplify dense paths added to the graph */
            PathSimplifierPtr psimp_;

            /** \brief Access to the weights of each Edge */
            boost::property_map<Graph, boost::edge_weight_t>::type edgeWeightProperty_;  // TODO: this is not used

            /** \brief Access to the collision checking state of each Edge */
            EdgeCollisionStateMap edgeCollisionStateProperty_;

            /** \brief Access to the internal base::state at each Vertex */
            boost::property_map<Graph, vertex_state_t>::type stateProperty_;

            /** \brief Access to the colors for the vertices */
            boost::property_map<Graph, vertex_color_t>::type colorProperty_;

            /** \brief Access to the interface pair information for the vertices */
            boost::property_map<Graph, vertex_interface_data_t>::type interfaceDataProperty_;

            /** \brief Data structure that maintains the connected components */
            boost::disjoint_sets<boost::property_map<Graph, boost::vertex_rank_t>::type,
                                 boost::property_map<Graph, boost::vertex_predecessor_t>::type> disjointSets_;
            /** \brief Random number generator */
            RNG rng_;

            /** \brief A flag indicating that a solution has been added during solve() */
            bool addedSolution_{false};

            /** \brief A counter for the number of consecutive failed iterations of the algorithm */
            unsigned int consecutiveFailures_{0u};

            /** \brief A counter for the number of iterations of the algorithm */
            long unsigned int iterations_{0ul};

            /** \brief Maximum visibility range for nodes in the graph */
            double sparseDelta_{0.};

            /** \brief Maximum range for allowing two samples to support an interface */
            double denseDelta_{0.};

            /** \brief Used by getSimilarPaths */
            std::vector<Vertex> startVertexCandidateNeighbors_;
            std::vector<Vertex> goalVertexCandidateNeighbors_;

            /** \brief Option to enable debugging output */
            bool verbose_{false};
        };
    }
}

#endif
