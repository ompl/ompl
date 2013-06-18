/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
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

/* Author: Andrew Dobson */

#ifndef OMPL_GEOMETRIC_PLANNERS_SPARSE_ROADMAP_SPANNER_
#define OMPL_GEOMETRIC_PLANNERS_SPARSE_ROADMAP_SPANNER_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/util/Time.h"

#include <boost/range/adaptor/map.hpp>
#include <boost/unordered_map.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/function.hpp>
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
           @anchor gSPARS
           @par Short description
           SPARS is an algorithm which operates similarly to the Visibility-based
           PRM.  It has several desirable properties, including asymptotic 
           near-optimality, and a meaningful stopping criterion.
           @par External documentation
         */

        /** \brief Incremental Probabilistic RoadMap planner */
        class SPARS : public base::Planner
        {
        public:

            struct vertex_state_t {
                typedef boost::vertex_property_tag kind;
            };

            struct vertex_total_connection_attempts_t {
                typedef boost::vertex_property_tag kind;
            };

            struct vertex_successful_connection_attempts_t {
                typedef boost::vertex_property_tag kind;
            };
            
            struct vertex_representative_t {
                typedef boost::vertex_property_tag kind;
            };
            
            struct vertex_color_t {
                typedef boost::vertex_property_tag kind;
            };
            
            struct vertex_list_t {
                typedef boost::vertex_property_tag kind;
            };
            
            struct vertex_interface_list_t {
                typedef boost::vertex_property_tag kind;
            };
                        
            typedef boost::unordered_map< unsigned int, std::list< unsigned int >, boost::hash<unsigned int> > interface_hash;

            typedef boost::unordered_map< unsigned int, std::list< unsigned int>, boost::hash< unsigned int > >::iterator IH_iterator;
            typedef boost::unordered_map< unsigned int, std::list< unsigned int>, boost::hash< unsigned int > >::const_iterator const_IH_iterator;
            
            typedef boost::adjacency_list <
                boost::vecS, boost::vecS, boost::undirectedS,
                boost::property < vertex_state_t, base::State*,
                boost::property < vertex_total_connection_attempts_t, unsigned int,
                boost::property < vertex_successful_connection_attempts_t, unsigned int,
                boost::property < boost::vertex_predecessor_t, unsigned long int,
                boost::property < boost::vertex_rank_t, unsigned long int,
                boost::property < vertex_color_t, unsigned int,
                boost::property < vertex_list_t, std::list< unsigned int >,
                boost::property < vertex_interface_list_t, interface_hash > > > > > > > >,
                boost::property < boost::edge_weight_t, double,
                boost::property < boost::edge_index_t, unsigned int> >
            > Spanner;
            
            typedef boost::graph_traits<Spanner>::vertex_descriptor Node;
            typedef boost::graph_traits<Spanner>::edge_descriptor   Link;

            typedef boost::shared_ptr< NearestNeighbors<Node> > SparseNeighbors;

            typedef boost::function<std::vector<Node>&(const Node)> LinkStrategy;
            
            typedef boost::function<bool(const Node, const Node)> LinkFilter;

            /**
             @brief The underlying roadmap graph.

             @par Any BGL graph representation could be used here. Because we
             expect the roadmap to be sparse (m<n^2), an adjacency_list is more
             appropriate than an adjacency_matrix.

             @par Obviously, a ompl::base::State* vertex property is required.
             The incremental connected components algorithm requires
             vertex_predecessor_t and vertex_rank_t properties.
             If boost::vecS is not used for vertex storage, then there must also
             be a boost:vertex_index_t property manually added.

             @par Edges should be undirected and have a weight property.
             */
            typedef boost::adjacency_list <
                boost::vecS, boost::vecS, boost::undirectedS,
                boost::property < vertex_state_t, base::State*,
                boost::property < vertex_total_connection_attempts_t, unsigned int,
                boost::property < vertex_successful_connection_attempts_t, unsigned int,
                boost::property < boost::vertex_predecessor_t, unsigned long int,
                boost::property < boost::vertex_rank_t, unsigned long int,
                boost::property < vertex_representative_t, Node > > > > > >,
                boost::property < boost::edge_weight_t, double,
                boost::property < boost::edge_index_t, unsigned int> >
            > Graph;
            
            typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
            typedef boost::graph_traits<Graph>::edge_descriptor   Edge;

            typedef boost::shared_ptr< NearestNeighbors<Vertex> > DenseNeighbors;

            /** @brief A function returning the milestones that should be
             * attempted to connect to
             *
             * @note Can't use the prefered boost::function syntax here because
             * the Python bindings don't like it.
             */
            typedef boost::function<std::vector<Vertex>&(const Vertex)>
                ConnectionStrategy;

            /** @brief A function that can reject connections.

             This is called after previous connections from the neighbor list
             have been added to the roadmap.
             */
            typedef boost::function<bool(const Vertex, const Vertex)> ConnectionFilter;

            /** \brief Constructor */
            SPARS(const base::SpaceInformationPtr &si, bool starStrategy = true);

            virtual ~SPARS(void);

            virtual void setProblemDefinition(const base::ProblemDefinitionPtr &pdef);

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
            void setConnectionStrategy(const ConnectionStrategy& connectionStrategy)
            {
                connectionStrategy_ = connectionStrategy;
                userSetConnectionStrategy_ = true;
            }

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
            void setConnectionFilter(const ConnectionFilter& connectionFilter)
            {
                connectionFilter_ = connectionFilter;
            }

            virtual void getPlannerData(base::PlannerData &data) const;

            /** \brief Attempt to add a single sample to the roadmap. */
            virtual Vertex addSample();
            
            /** \brief Attempt to add a single sample to the roadmap. */
            virtual Vertex addSample(base::State *workState);
            
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
            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

            /** \brief Clear the query previously loaded from the ProblemDefinition.
                Subsequent calls to solve() will reuse the previously computed roadmap,
                but will clear the set of input states constructed by the previous call to solve().
                This enables multi-query functionality for PRM. */
            void clearQuery(void);

            virtual void clear(void);

            /** \brief Set a different nearest neighbors datastructure */
            template<template<typename T> class NN>
            void setDenseNeighbors(void)
            {
                nn_.reset(new NN<Vertex>());
                if (!userSetConnectionStrategy_)
                    connectionStrategy_.clear();
                if (isSetup())
                    setup();
            }

            /** \brief Set a different nearest neighbors datastructure for the spanner */
            template<template<typename T> class NN>
            void setSparseNeighbors(void)
            {
                snn_.reset(new NN<Node>());
                if (isSetup())
                    setup();
            }

            void setMaxFailures( unsigned int m )
            {
                m_ = m;
            }
            
            void setDenseDelta( double d )
            {
                d_max = d;
            }
            
            void setSparseDelta( double d )
            {
                D_max = d;
            }
            
            void setStretchFactor( double t )
            {
                t_ = t;
            }
            
            void setQueryFilename( std::string inf )
            {
                qFname_ = inf;
            }
            
            virtual void setup(void);

            const Graph& getGraph(void) const
            {
                return g_;
            }
            
            const Spanner& getRoadmap(void) const
            {
                return s_;
            }

            /** \brief Compute distance between two milestones (this is simply distance between the states of the milestones) */
            double distanceFunction(const Vertex a, const Vertex b) const
            {
                return si_->distance(stateProperty_[a], stateProperty_[b]);
            }
            
            /** \brief Compute distance between two Nodes */
            double sparseDistanceFunction( const Node a, const Node b ) const
            {
                return si_->distance( sparseStateProperty_[a], sparseStateProperty_[b] );
            }

            /** \brief Returns the number of milestones added to D */
            unsigned int milestoneCount(void) const
            {
                return boost::num_vertices(g_);
            }
            
            /** \brief Returns the number of guards added to S */
            unsigned int guardCount(void) const
            {
                return boost::num_vertices(s_);
            }

            const DenseNeighbors& getNearestNeighbors(void)
            {
                return nn_;
            }
            
            const SparseNeighbors& getSparseNearestNeighbors(void)
            {
                return snn_;
            }

        protected:

            /** \brief Free all the memory allocated by the planner */
            void freeMemory(void);

            /** \brief Construct a milestone for a given state (\e state) and store it in the nearest neighbors data structure */
            virtual Vertex addMilestone(base::State *state);
            
            /** \brief Construct a node with the given state (\e state) for the spanner and store it in the nn structure */
            virtual Node addGuard(base::State *state, unsigned int type);

            /** \brief Make two milestones (\e m1 and \e m2) be part of the same connected component. The component with fewer elements will get the id of the component with more elements. */
            void uniteComponents(Vertex m1, Vertex m2);

            /** \brief Make two nodes (\e m1 and \e m2) be part of the same connected component. The component with fewer elements will get the id of the component with more elements. */
            void uniteSparseComponents(Node m1, Node m2);

            /** \brief Convenience function for creating an edge in the Spanner Roadmap */
            void connectSparsePoints( Node v, Node vp );
            
            /** \brief Connects points in the dense graph */
            void connectDensePoints( Vertex v, Vertex vp );
            
            /** \brief Checks the latest dense sample for the coverage property, and adds appropriately. */
            bool checkAddCoverage( std::vector<Node> neigh );

            /** \brief Checks the latest dense sample for connectivity, and adds appropriately. */
            bool checkAddConnectivity( std::vector<Node> neigh );

            /** \brief Checks the latest dense sample for bridging an edge-less interface */
            bool checkAddInterface( const std::vector<Vertex>& graphNeighborhood, const std::vector<Vertex>& visibleNeighborhood, Vertex q );

            /** \brief Checks for adding an entire dense path to the Sparse Roadmap */
            bool checkAddPath( Vertex q, const std::vector<Vertex>& neigh );

            /** \brief Returns the average valence of the graph */
            double avgValence( void );
            
            /** \brief reset function for failures */
            void resetFailures( void );
            
            /** \brief Function for approaching the graph. */
            void approachGraph( Vertex v );
            /** \brief Function for approaching the roadmap spanner. */
            void approachSpanner( Node n );
            
            /** \brief Get all nodes in the sparse graph which are within D_max of the given state. */
            std::vector<Node> getSparseNeighbors( base::State* inState );
            
            /** \brief Get the visible neighbors */
            std::vector<Node> getVisibleNeighbors( base::State* inState );
            
            /** \brief Get the first neighbor of q who has representative rep and is within d_max. */
            bool getInterfaceNeighbor( Vertex q, Node rep, Node& ret );
            
            /** \brief Method for adding a pair of nodes to S, with some smoothing applied. */
            bool addPairToSpanner( Vertex v, Vertex vp );

            /** \brief Method for actually adding a dense path to the Roadmap Spanner, S. */
            bool addPathToSpanner( PathGeometric* p, Node vp, Node vpp );

            /** \brief Automatically updates the representatives of all dense samplse within D_max of v */
            void updateReps( Node v );
            
            /** \brief Calculates the representative for a dense sample */
            void calculateRepresentative( Vertex q );
            
            /** \brief Adds a dense sample to the appropriate lists of its representative */
            void addToRep( Vertex q, Node rep, std::vector<Node> oreps );
            
            /** \brief Removes the node from its representative's lists */
            void removeFromRep( Vertex q, Node rep );
            
            /** \brief Computes all nodes which qualify as a candidate v" for v and vp */
            std::vector<Vertex> computeVPP( Vertex v, Vertex vp );
            
            /** \brief Computes all nodes which qualify as a candidate x for v, v', and v" */
            std::vector<Vertex> computeX( Vertex v, Vertex vp, Vertex vpp );

            /** \brief Allocates a midpoint */
            base::State* generateMidpoint( base::State* a, base::State* b );
            
            /** \brief Gets all dense samples which suppport an interface along node q */
            std::vector<Vertex> interfaceCheck( Vertex q );

            /** \brief Gets the representatives of all interfaces that q supports */
            std::vector<Node> getInterfaceNeighborRepresentatives( Vertex q );

            /** \brief Gets the neighbors of q who help it support an interface */
            std::vector<Vertex> getInterfaceNeighborhood( Vertex q );
            
            /** Thread that checks for solution */
            void checkForSolution (const base::PlannerTerminationCondition &ptc, base::PathPtr &solution);

            /** \brief Check if there exists a solution, i.e., there exists a pair of milestones such that the first is in \e start and the second is in \e goal, and the two milestones are in the same connected component. If a solution is found, the path is saved. */
            bool haveSolution(const std::vector<Vertex> &start, const std::vector<Vertex> &goal, base::PathPtr &solution);

            /** \brief Returns the value of the addedSolution_ member. */
            bool addedNewSolution (void) const;

            /** \brief Given two milestones from the same connected component, construct a path connecting them and set it as the solution */
            virtual base::PathPtr constructSolution(const Node start, const Node goal) const;

            /** \brief Constructs the dense path between the start and goal vertices (if connected) */
            PathGeometric* densePath( const Vertex start, const Vertex goal ) const;

            /** \brief Flag indicating whether the default connection strategy is the Star strategy */
            bool                                                   starStrategy_;

            /** \brief Sampler user for generating valid samples in the state space */
            base::ValidStateSamplerPtr                             sampler_;

            /** \brief Sampler user for generating random in the state space */
            base::StateSamplerPtr                                  simpleSampler_;

            /** \brief Nearest neighbors data structure */
            DenseNeighbors                                         nn_;

            /** \brief Nearest Neighbors structure for the sparse roadmap */
            SparseNeighbors                                        snn_;
            
            /** \brief The dense graph, D */
            Graph                                                  g_;
            
            /** \brief The sparse roadmap, S */
            Spanner                                                s_;

            /** \brief Array of start guards */
            std::vector<Node>                                      startM_;

            /** \brief Array of goal guards */
            std::vector<Node>                                      goalM_;

            /** \brief Vertex for performing nearest neighbor queries on the SPARSE roadmap. */
            Vertex                                                 sparse_query_v_;

            /** \brief Vertex for performing nearest neighbor queries on the DENSE graph. */
            Vertex                                                 query_v_;

            /** \brief Access to the internal base::state at each Vertex */
            boost::property_map<Graph, vertex_state_t>::type       stateProperty_;
            
            /** \brief Access to the internal base::State for each Node of S */
            boost::property_map<Spanner, vertex_state_t>::type     sparseStateProperty_;

            /** \brief Access to draw colors for the Nodes of S, to indicate addition type */
            boost::property_map<Spanner, vertex_color_t>::type     sparseColorProperty_;
            
            /** \brief Access to the representatives of the Dense vertices */
            boost::property_map<Graph, vertex_representative_t>::type   representativesProperty_;
            
            /** \brief Access to all non-interface supporting vertices of the sparse nodes */
            boost::property_map<Spanner, vertex_list_t>::type      nonInterfaceListsProperty_;
            
            /** \brief Access to the interface-supporting vertice hashes of the sparse nodes */
            boost::property_map<Spanner, vertex_interface_list_t>::type interfaceListsProperty_;
            
            /** \brief Access to the number of total connection attempts for a vertex */
            boost::property_map<Graph,
                vertex_total_connection_attempts_t>::type          totalConnectionAttemptsProperty_;

            /** \brief Access to the number of successful connection attempts for a vertex */
            boost::property_map<Graph,
                vertex_successful_connection_attempts_t>::type     successfulConnectionAttemptsProperty_;

            /** \brief A path simplifier used to simplify dense paths added to S */
            PathSimplifierPtr                                      psimp_;
            
            /** \brief Access to the weights of each Edge */
            boost::property_map<Graph, boost::edge_weight_t>::type weightProperty_;

            /** \brief Access to the indices of each Edge */
            boost::property_map<Graph, boost::edge_index_t>::type  edgeIDProperty_;

            /** \brief Data structure that maintains the connected components of D */
            boost::disjoint_sets<
                boost::property_map<Graph, boost::vertex_rank_t>::type,
                boost::property_map<Graph, boost::vertex_predecessor_t>::type >
                                                                   disjointSets_;

            boost::unordered_set< unsigned int >                   vertices_;
            std::vector< std::pair< double, Vertex > >             magic_;
            
            /** \brief Data structure that maintains the connected components of S */
            boost::disjoint_sets<
                boost::property_map<Spanner, boost::vertex_rank_t>::type,
                boost::property_map<Spanner, boost::vertex_predecessor_t>::type >
                                                                   sparseDJSets_;

            /** \brief Maximum unique id number used so for for edges */
            unsigned int                                           maxEdgeID_;

            /** \brief Maximum unique id number used so for for links */
            unsigned int                                           maxLinkID_;

            /** \brief Function that returns the milestones to attempt connections with */
            ConnectionStrategy                                     connectionStrategy_;

            /** \brief Function that can reject a milestone connection */
            ConnectionFilter                                       connectionFilter_;

            /** \brief Flag indicating whether the employed connection strategy was set by the user (or defaults are assumed) */
            bool                                                   userSetConnectionStrategy_;

            /** \brief Random number generator */
            RNG                                                    rng_;

            /** \brief A flag indicating that a solution has been added during solve() */
            bool                                                   addedSolution_;
            
            /** \brief A counter for the number of iterations of the algorithm */
            unsigned int                                           iterations_;
            
            /** \brief The stretch factor in terms of graph spanners for SPARS to check against */
            double                                                 t_;
            
            /** \brief The maximum number of failures before terminating the algorithm */
            unsigned int                                           m_;
            
            /** \brief SPARS parameter for dense graph connection distance */
            double                                                 d_max;
            
            /** \brief SPARS parameter for Sparse Roadmap connection distance */
            double                                                 D_max;

            /** \brief A holder for the last state added to D */
            base::State*                                           last_state;
            
            /** \brief The filename to be used for loading query test information */
            std::string                                            qFname_;
        };

    }
}

#endif
