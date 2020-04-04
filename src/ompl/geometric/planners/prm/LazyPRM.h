/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

/* Author: Ioan Sucan, Henning Kayser */

#ifndef OMPL_GEOMETRIC_PLANNERS_PRM_LAZY_PRM_
#define OMPL_GEOMETRIC_PLANNERS_PRM_LAZY_PRM_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <utility>
#include <vector>
#include <map>

namespace ompl
{
    namespace base
    {
        // Forward declare for use in implementation
        OMPL_CLASS_FORWARD(OptimizationObjective);
    }

    namespace geometric
    {
        /**
           @anchor gLazyPRM
           @par Short description
           LazyPRM is a planner that constructs a roadmap of milestones
           that approximate the connectivity of the state space, just like PRM does.
           The difference is that the planner uses lazy collision checking.
           @par External documentation
           R. Bohlin and L.E. Kavraki
           Path Planning Using Lazy PRM
           <em>IEEE International Conference on Robotics and Automation</em>, San Francisco, pp. 521–528, 2000.
           DOI: [10.1109/ROBOT.2000.844107](http://dx.doi.org/10.1109/ROBOT.2000.844107)<br>
           [[more]](http://www.kavrakilab.org/robotics/lazyprm.html)
        */

        /** \brief Lazy Probabilistic RoadMap planner */
        class LazyPRM : public base::Planner
        {
        public:
            struct vertex_state_t
            {
                using kind = boost::vertex_property_tag;
            };

            struct vertex_flags_t
            {
                using kind = boost::vertex_property_tag;
            };

            struct vertex_component_t
            {
                using kind = boost::vertex_property_tag;
            };

            struct edge_flags_t
            {
                using kind = boost::edge_property_tag;
            };

            /** @brief The type for a vertex in the roadmap. */
            using Vertex =
                boost::adjacency_list_traits<boost::vecS, boost::listS, boost::undirectedS>::vertex_descriptor;

            /**
             @brief The underlying roadmap graph.

             @par Any BGL graph representation could be used here. Because we
             expect the roadmap to be sparse (m<n^2), an adjacency_list is more
             appropriate than an adjacency_matrix. We use listS for the vertex list
             because vertex descriptors are invalidated by remove operations if using vecS.

             @par Obviously, a ompl::base::State* vertex property is required.
             The incremental connected components algorithm requires
             vertex_predecessor_t and vertex_rank_t properties.
             If boost::vecS is not used for vertex storage, then there must also
             be a boost:vertex_index_t property manually added.

             @par Edges should be undirected and have a weight property.
             */
            using Graph = boost::adjacency_list<
                boost::vecS, boost::listS, boost::undirectedS,
                boost::property<
                    vertex_state_t, base::State *,
                    boost::property<
                        boost::vertex_index_t, unsigned long int,
                        boost::property<vertex_flags_t, unsigned int,
                                        boost::property<vertex_component_t, unsigned long int,
                                                        boost::property<boost::vertex_predecessor_t, Vertex,
                                                                        boost::property<boost::vertex_rank_t,
                                                                                        unsigned long int>>>>>>,
                boost::property<boost::edge_weight_t, base::Cost, boost::property<edge_flags_t, unsigned int>>>;

            /** @brief The type for an edge in the roadmap. */
            using Edge = boost::graph_traits<Graph>::edge_descriptor;

            /** @brief A nearest neighbors data structure for roadmap vertices. */
            using RoadmapNeighbors = std::shared_ptr<NearestNeighbors<Vertex> >;

            /** @brief A function returning the milestones that should be
             * attempted to connect to. */
            using ConnectionStrategy = std::function<const std::vector<Vertex> &(const Vertex)>;

            /** @brief A function that can reject connections.

             This is called after previous connections from the neighbor list
             have been added to the roadmap.
             */
            using ConnectionFilter = std::function<bool (const Vertex &, const Vertex &)>;

            /** \brief Constructor */
            LazyPRM(const base::SpaceInformationPtr &si, bool starStrategy = false);

            /** \brief Constructor */
            LazyPRM(const base::PlannerData &data, bool starStrategy = false);

            ~LazyPRM() override;

            /** \brief Set the maximum length of a motion to be added to the roadmap. */
            void setRange(double distance);

            /** \brief Get the range the planner is using */
            double getRange() const
            {
                return maxDistance_;
            }

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() == 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Vertex>>();
                if (!userSetConnectionStrategy_)
                    setDefaultConnectionStrategy();
                if (isSetup())
                    setup();
            }

            void setProblemDefinition(const base::ProblemDefinitionPtr &pdef) override;

            /** \brief Set the connection strategy function that specifies the
             milestones that connection attempts will be make to for a
             given milestone.

             \par The behavior and performance of PRM can be changed drastically
             by varying the number and properties if the milestones that are
             connected to each other.

             \param pdef A function that takes a milestone as an argument and
             returns a collection of other milestones to which a connection
             attempt must be made. The default connection strategy is to connect
             a milestone's 10 closest neighbors.
             */
            void setConnectionStrategy(const ConnectionStrategy &connectionStrategy)
            {
                connectionStrategy_ = connectionStrategy;
                userSetConnectionStrategy_ = true;
            }
            /** Set default strategy for connecting to nearest neighbors */
            void setDefaultConnectionStrategy();

            /** \brief Convenience function that sets the connection strategy to the
             default one with k nearest neighbors.
             */
            void setMaxNearestNeighbors(unsigned int k);

            /** \brief Set the function that can reject a milestone connection.

             \par The given function is called immediately before a connection
             is checked for collision and added to the roadmap. Other neighbors
             may have already been connected before this function is called.
             This allows certain heuristics that use the structure of the
             roadmap (like connected components or useful cycles) to be
             implemented by changing this function.

             \param connectionFilter A function that takes the new milestone,
             a neighboring milestone and returns whether a connection should be
             attempted.
             */
            void setConnectionFilter(const ConnectionFilter &connectionFilter)
            {
                connectionFilter_ = connectionFilter;
            }

            /** \brief Return the number of milestones currently in the graph */
            unsigned long int milestoneCount() const
            {
                return boost::num_vertices(g_);
            }

            /** \brief Return the number of edges currently in the graph */
            unsigned long int edgeCount() const
            {
                return boost::num_edges(g_);
            }

            void getPlannerData(base::PlannerData &data) const override;

            void setup() override;

            void clear() override;

            /** \brief Clear the query previously loaded from the ProblemDefinition.
                Subsequent calls to solve() will reuse the previously computed roadmap,
                but will clear the set of input states constructed by the previous call to solve().
                This enables multi-query functionality for LazyPRM. */
            void clearQuery() override;

            /** \brief change the validity flag of each node and edge to VALIDITY_UNKNOWN */
            void clearValidity();

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

        protected:
            /** \brief Flag indicating validity of an edge of a vertex */
            static const unsigned int VALIDITY_UNKNOWN = 0;

            /** \brief Flag indicating validity of an edge of a vertex */
            static const unsigned int VALIDITY_TRUE = 1;

            ///////////////////////////////////////
            // Planner progress property functions
            std::string getIterationCount() const
            {
                return std::to_string(iterations_);
            }
            std::string getBestCost() const
            {
                return std::to_string(bestCost_.value());
            }
            std::string getMilestoneCountString() const
            {
                return std::to_string(milestoneCount());
            }
            std::string getEdgeCountString() const
            {
                return std::to_string(edgeCount());
            }

            /** \brief Free all the memory allocated by the planner */
            void freeMemory();

            /** \brief Construct a milestone for a given state (\e state), store it in the nearest neighbors data
               structure
                and then connect it to the roadmap in accordance to the connection strategy. */
            Vertex addMilestone(base::State *state);

            void uniteComponents(Vertex a, Vertex b);

            void markComponent(Vertex v, unsigned long int newComponent);

            /** \brief Check if any pair of a start state and goal state are part of the same connected component.
                If so, return the id of that component. Otherwise, return -1. */
            long int solutionComponent(std::pair<std::size_t, std::size_t> *startGoalPair) const;

            /** \brief Given two milestones from the same connected component, construct a path connecting them and set
             * it as the solution */
            ompl::base::PathPtr constructSolution(const Vertex &start, const Vertex &goal);

            /** \brief Compute distance between two milestones (this is simply distance between the states of the
             * milestones) */
            double distanceFunction(const Vertex a, const Vertex b) const
            {
                return si_->distance(stateProperty_[a], stateProperty_[b]);
            }

            /** \brief Given two vertices, returns a heuristic on the cost of the path connecting them.
                This method wraps OptimizationObjective::motionCostHeuristic */
            base::Cost costHeuristic(Vertex u, Vertex v) const;

            /** \brief Flag indicating whether the default connection strategy is the Star strategy */
            bool starStrategy_;

            /** \brief Function that returns the milestones to attempt connections with */
            ConnectionStrategy connectionStrategy_;

            /** \brief Function that can reject a milestone connection */
            ConnectionFilter connectionFilter_;

            /** \brief Flag indicating whether the employed connection strategy was set by the user (or defaults are
             * assumed) */
            bool userSetConnectionStrategy_{false};

            /** \brief The maximum length of a motion to be added to a tree */
            double maxDistance_{0.};

            /** \brief Sampler user for generating random in the state space */
            base::StateSamplerPtr sampler_;

            /** \brief Nearest neighbors data structure */
            RoadmapNeighbors nn_;

            /** \brief Connectivity graph */
            Graph g_;

            /** \brief Array of start milestones */
            std::vector<Vertex> startM_;

            /** \brief Array of goal milestones */
            std::vector<Vertex> goalM_;

            /** \brief Access to the internal base::state at each Vertex */
            boost::property_map<Graph, boost::vertex_index_t>::type indexProperty_;

            /** \brief Access to the internal base::state at each Vertex */
            boost::property_map<Graph, vertex_state_t>::type stateProperty_;

            /** \brief Access to the weights of each Edge */
            boost::property_map<Graph, boost::edge_weight_t>::type weightProperty_;

            /** \brief Access the connected component of a vertex */
            boost::property_map<Graph, vertex_component_t>::type vertexComponentProperty_;

            /** \brief Access the validity state of a vertex */
            boost::property_map<Graph, vertex_flags_t>::type vertexValidityProperty_;

            /** \brief Access the validity state of an edge */
            boost::property_map<Graph, edge_flags_t>::type edgeValidityProperty_;

            /** \brief Number of connected components created so far. This is used as an ID only,
                does not represent the actual number of components currently in the graph. */
            unsigned long int componentCount_{0};

            /** \brief The number of elements in each component in the LazyPRM roadmap. */
            std::map<unsigned long int, unsigned long int> componentSize_;

            /** \brief Objective cost function for PRM graph edges */
            base::OptimizationObjectivePtr opt_;

            base::Cost bestCost_{std::numeric_limits<double>::quiet_NaN()};

            unsigned long int iterations_{0};
        };
    }
}

#endif
