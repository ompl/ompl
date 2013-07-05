/*********************************************************************
*  @copyright Software License Agreement (BSD License)
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
           A. Dobson, A. Krontiris, K. Bekris,
           Sparse Roadmap Spanners,
           <em>Workshop on the Algorithmic Foundations of Robotics (WAFR)</em> 2012.
           <a href="http://www.cs.rutgers.edu/~kb572/pubs/sparse_roadmap_spanner.pdf">[PDF]</a>
         */

        /** \brief <b> SPArse Roadmap Spanner technique. </b> */
        class SPARS : public base::Planner
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

            struct vertex_state_t {
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

            /** \brief Hash for storing interface information. */
            typedef boost::unordered_map< unsigned int, std::list< unsigned int >, boost::hash<unsigned int> > InterfaceHash;

            /**
             @brief The constructed roadmap spanner.

             @par Any BGL graph representation could be used here, but the
             spanner should be very sparse (m<n^2), so we use an adjacency_list.

             @par Nodes in the spanner contain extra information needed by the
             spanner technique, including nodes in the dense graph which nodes
             in the spanner represent.

             @par SparseEdges should be undirected and have a weight property.
             */
            typedef boost::adjacency_list <
                boost::vecS, boost::vecS, boost::undirectedS,
                boost::property < vertex_state_t, base::State*,
                boost::property < boost::vertex_predecessor_t, unsigned long int,
                boost::property < boost::vertex_rank_t, unsigned long int,
                boost::property < vertex_color_t, GuardType,
                boost::property < vertex_list_t, std::list< unsigned int >,
                boost::property < vertex_interface_list_t, InterfaceHash > > > > > >,
                boost::property < boost::edge_weight_t, double >
            > SpannerGraph;

            typedef boost::graph_traits<SpannerGraph>::vertex_descriptor SparseVertex;
            typedef boost::graph_traits<SpannerGraph>::edge_descriptor   SparseEdge;

            /** \brief Nearest neighbor structure which works over the SpannerGraph */
            typedef boost::shared_ptr< NearestNeighbors<SparseVertex> > SparseNeighbors;

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

             @par DenseEdges should be undirected and have a weight property.
             */
            typedef boost::adjacency_list <
                boost::vecS, boost::vecS, boost::undirectedS,
                boost::property < vertex_state_t, base::State*,
                boost::property < boost::vertex_predecessor_t, unsigned long int,
                boost::property < boost::vertex_rank_t, unsigned long int,
                boost::property < vertex_representative_t, SparseVertex > > > >,
                boost::property < boost::edge_weight_t, double >
            > DenseGraph;

            typedef boost::graph_traits<DenseGraph>::vertex_descriptor DenseVertex;
            typedef boost::graph_traits<DenseGraph>::edge_descriptor   DenseEdge;

            typedef boost::shared_ptr< NearestNeighbors<DenseVertex> > DenseNeighbors;

            /** \brief Constructor. */
            SPARS(const base::SpaceInformationPtr &si);
            /** \brief Destructor. */
            virtual ~SPARS(void);

            virtual void setProblemDefinition(const base::ProblemDefinitionPtr &pdef);

            virtual void getPlannerData(base::PlannerData &data) const;

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

            /** \brief Alternate solve call with maximum failures as a
             function parameter.  Overwrites the parameter member maxFailures_.

             \param maxFail Maximum consecutive failures to add to the spanner allowed.*/
            virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc, unsigned int maxFail );

            /** \brief Clear the query previously loaded from the ProblemDefinition.
                Subsequent calls to solve() will reuse the previously computed roadmap,
                but will clear the set of input states constructed by the previous call to solve().
                This enables multi-query functionality for PRM. */
            void clearQuery(void);

            virtual void clear(void);

            /** \brief Set a different nearest neighbors datastructure for the roadmap graph.
                This nearest neighbor structure contains only information on the nodes
                existing in the underlying dense roadmap.  This structure is used for
                near-neighbor queries for the construction of that graph as well as for
                determining which dense samples the sparse roadmap nodes should represent.*/
            template<template<typename T> class NN>
            void setDenseNeighbors(void)
            {
                nn_.reset(new NN<DenseVertex>());
                connectionStrategy_.clear();
                if (isSetup())
                    setup();
            }

            /** \brief Set a different nearest neighbors datastructure for the spanner graph.
                This structure is stores only nodes in the roadmap spanner, and is used in
                the construction of the spanner.  It can also be queried to determine which
                node in the spanner should represent a given state.*/
            template<template<typename T> class NN>
            void setSparseNeighbors(void)
            {
                snn_.reset(new NN<SparseVertex>());
                if (isSetup())
                    setup();
            }

            /** \brief Set the maximum consecutive failures to augment the spanner before termination.
                In general, if the algorithm fails to add to the spanner for M consecutive iterations,
                then we can probabilistically estimate how close to attaining the desired properties
                the SPARS spanner is.*/
            void setMaxFailures( unsigned int m )
            {
                maxFailures_ = m;
            }

            /** \brief Set the delta value for interface detection.  If two nodes in the dense graph
                are more than delta distance appart, then the algorithm cannot consider them to have
                accurately approximated the location of an interface. */
            void setDenseDelta( double d )
            {
                denseDelta_ = d;
            }

            /** \brief Set the delta value for connection distance on the sparse spanner.  This
                value represents the visibility range of sparse samples.  A sparse node
                represents all dense nodes within this distance if it is also the closest sparse
                node to that dense node.*/
            void setSparseDelta( double d )
            {
                sparseDelta_ = d;
            }

            /** \brief Set the roadmap spanner stretch factor.  This value represents a
                multiplicative upper bound on path quality that should be produced by the
                roadmap spanner.  It does not make sense to make this parameter more than 3. */
            void setStretchFactor( double t )
            {
                stretchFactor_ = t;
            }

            /** \brief Retrieve the maximum consecutive failure limit. */
            unsigned getMaxFailures( ) const
            {
                return maxFailures_;
            }

            /** \brief Retrieve the dense graph interface support delta. */
            double getDenseDelta( ) const
            {
                return denseDelta_;
            }

            /** \brief Retrieve the sparse graph visibility range delta. */
            double getSparseDelta( ) const
            {
                return sparseDelta_;
            }

            /** \brief Retrieve the spanner's set stretch factor. */
            double getStretchFactor( ) const
            {
                return stretchFactor_;
            }

            virtual void setup(void);

            /** \brief Retrieve the underlying dense graph structure.  This is built as a PRM*
                and asymptotically approximates best paths through the space. */
            const DenseGraph& getDenseGraph(void) const
            {
                return g_;
            }

            /** \brief Retrieve the sparse roadmap structure.  This is the structure which
                answers given queries, and has the desired property of asymptotic near-optimality.*/
            const SpannerGraph& getRoadmap(void) const
            {
                return s_;
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

        protected:

            /** @brief A function returning the milestones that should be
             * attempted to connect to
             *
             * @note Can't use the prefered boost::function syntax here because
             * the Python bindings don't like it.
             */
            typedef boost::function<std::vector<DenseVertex>&(const DenseVertex)> ConnectionStrategy;

            /** \brief Attempt to add a single sample to the roadmap. */
            virtual DenseVertex addSample();

            /** \brief Attempt to add a single sample to the roadmap. */
            virtual DenseVertex addSample(base::State *workState);

            /** \brief Free all the memory allocated by the planner */
            void freeMemory(void);

            /** \brief Construct a milestone for a given state (\e state) and store it in the nearest neighbors data structure */
            virtual DenseVertex addMilestone(base::State *state);

            /** \brief Construct a node with the given state (\e state) for the spanner and store it in the nn structure */
            virtual SparseVertex addGuard(base::State *state, GuardType type);

            /** \brief Make two nodes (\e m1 and \e m2) be part of the same connected component. The component with fewer elements will get the id of the component with more elements. */
            void uniteSparseComponents(SparseVertex m1, SparseVertex m2);

            /** \brief Convenience function for creating an edge in the Spanner Roadmap */
            void connectSparsePoints( SparseVertex v, SparseVertex vp );

            /** \brief Connects points in the dense graph */
            void connectDensePoints( DenseVertex v, DenseVertex vp );

            /** \brief Checks the latest dense sample for the coverage property, and adds appropriately. */
            bool checkAddCoverage( const std::vector<SparseVertex> & neigh );

            /** \brief Checks the latest dense sample for connectivity, and adds appropriately. */
            bool checkAddConnectivity( const std::vector<SparseVertex> & neigh );

            /** \brief Checks the latest dense sample for bridging an edge-less interface */
            bool checkAddInterface( const std::vector<DenseVertex>& graphNeighborhood, const std::vector<DenseVertex>& visibleNeighborhood, DenseVertex q );

            /** \brief Checks for adding an entire dense path to the Sparse Roadmap */
            bool checkAddPath( DenseVertex q, const std::vector<DenseVertex>& neigh );

            /** \brief Returns the average valence of the graph */
            double avgValence( void ) const;

            /** \brief reset function for failures */
            void resetFailures( void );

            /** \brief Function for approaching the graph. */
            void approachGraph( DenseVertex v );

            /** \brief Function for approaching the roadmap spanner. */
            void approachSpanner( SparseVertex n );

            /** \brief Get the first neighbor of q who has representative rep and is within denseDelta_. */
            DenseVertex getInterfaceNeighbor(DenseVertex q, SparseVertex rep);

            /** \brief Method for actually adding a dense path to the Roadmap Spanner, S. */
            bool addPathToSpanner( const std::deque< base::State* >& p, SparseVertex vp, SparseVertex vpp );

            /** \brief Automatically updates the representatives of all dense samplse within sparseDelta_ of v */
            void updateReps( SparseVertex v );

            /** \brief Calculates the representative for a dense sample */
            void calculateRepresentative( DenseVertex q );

            /** \brief Adds a dense sample to the appropriate lists of its representative */
            void addToRep( DenseVertex q, SparseVertex rep, const std::vector<SparseVertex>& oreps );

            /** \brief Removes the node from its representative's lists */
            void removeFromRep( DenseVertex q, SparseVertex rep );

            /** \brief Computes all nodes which qualify as a candidate v" for v and vp */
            void computeVPP( DenseVertex v, DenseVertex vp );

            /** \brief Computes all nodes which qualify as a candidate x for v, v', and v" */
            void computeX( DenseVertex v, DenseVertex vp, DenseVertex vpp );

            /** \brief Allocates a midpoint */
            base::State* generateMidpoint( const base::State* a, const base::State* b ) const;

            /** \brief Gets the representatives of all interfaces that q supports */
            void getInterfaceNeighborRepresentatives( DenseVertex q );

            /** \brief Gets the neighbors of q who help it support an interface */
            void getInterfaceNeighborhood( DenseVertex q );

            /** \brief Check if there exists a solution, i.e., there exists a pair of milestones such that the first is in \e start and the second is in \e goal, and the two milestones are in the same connected component. If a solution is found, the path is saved. */
            bool haveSolution(const std::vector<DenseVertex> &start, const std::vector<DenseVertex> &goal, base::PathPtr &solution);

            /** \brief Returns whether we have reached the iteration failures limit, maxFailures_ */
            bool reachedFailureLimit (void) const;

            /** \brief Given two milestones from the same connected component, construct a path connecting them and set it as the solution */
            virtual base::PathPtr constructSolution(const SparseVertex start, const SparseVertex goal) const;

            /** \brief Constructs the dense path between the start and goal vertices (if connected) */
            void densePath( const DenseVertex start, const DenseVertex goal ) const;

            /** \brief Compute the length of the path stored in the variable path_ */
            double pathLength() const;

            /** \brief Sampler user for generating valid samples in the state space */
            base::ValidStateSamplerPtr                                          sampler_;

            /** \brief Sampler user for generating random in the state space */
            base::StateSamplerPtr                                               simpleSampler_;

            /** \brief Nearest neighbors data structure */
            DenseNeighbors                                                      nn_;

            /** \brief Nearest Neighbors structure for the sparse roadmap */
            SparseNeighbors                                                     snn_;

            /** \brief The dense graph, D */
            DenseGraph                                                          g_;

            /** \brief The sparse roadmap, S */
            SpannerGraph                                                        s_;

            /** \brief Array of start guards */
            std::vector<SparseVertex>                                           startM_;

            /** \brief Array of goal guards */
            std::vector<SparseVertex>                                           goalM_;

            /** \brief DenseVertex for performing nearest neighbor queries on the SPARSE roadmap. */
            DenseVertex                                                         sparseQueryVertex_;

            /** \brief Vertex for performing nearest neighbor queries on the DENSE graph. */
            DenseVertex                                                         queryVertex_;

            /** \brief The whole neighborhood set which has been most recently computed */
            std::vector< SparseVertex >                                         graphNeighborhood_;

            /** \brief The visible neighborhood set which has been most recently computed */
            std::vector< SparseVertex >                                         visibleNeighborhood_;

            /** \brief Storage for the interface neighborhood, populated by getInterfaceNeighborhood() */
            std::vector< DenseVertex >                                          interfaceNeighborhood_;
            /** \brief Storage for the representatives of interface neighbors, populated by getInterfaceNeighborhoodRepresentatives() */
            std::vector< SparseVertex >                                         interfaceRepresentatives_;

            /** \brief Vector of V double prime vertices, filled in by the computeVPP funciton. */
            std::vector< SparseVertex >                                         VPPs_;

            /** \brief Vector of X vertices, filled in by the computeX function. */
            std::vector< SparseVertex >                                         Xs_;

            /** \brief Path variable which stores the result of the densePath() function. */
            mutable std::deque< base::State* >                                  path_;

            /** \brief Geometric Path variable used for smoothing out paths. */
            PathGeometric                                                       geomPath_;

            /** \brief Access to the internal base::state at each DenseVertex */
            boost::property_map<DenseGraph, vertex_state_t>::type               stateProperty_;

            /** \brief Access to the internal base::State for each SparseVertex of S */
            boost::property_map<SpannerGraph, vertex_state_t>::type             sparseStateProperty_;

            /** \brief Access to draw colors for the SparseVertexs of S, to indicate addition type */
            boost::property_map<SpannerGraph, vertex_color_t>::type             sparseColorProperty_;

            /** \brief Access to the representatives of the Dense vertices */
            boost::property_map<DenseGraph, vertex_representative_t>::type      representativesProperty_;

            /** \brief Access to all non-interface supporting vertices of the sparse nodes */
            boost::property_map<SpannerGraph, vertex_list_t>::type              nonInterfaceListsProperty_;

            /** \brief Access to the interface-supporting vertice hashes of the sparse nodes */
            boost::property_map<SpannerGraph, vertex_interface_list_t>::type    interfaceListsProperty_;

            /** \brief A path simplifier used to simplify dense paths added to S */
            PathSimplifierPtr                                                   psimp_;

            /** \brief Access to the weights of each DenseEdge */
            boost::property_map<DenseGraph, boost::edge_weight_t>::type         weightProperty_;

            /** \brief Data structure that maintains the connected components of S */
            boost::disjoint_sets<
                boost::property_map<SpannerGraph, boost::vertex_rank_t>::type,
                boost::property_map<SpannerGraph, boost::vertex_predecessor_t>::type >
                                                                                sparseDJSets_;

            /** \brief Function that returns the milestones to attempt connections with */
            ConnectionStrategy                                                  connectionStrategy_;

            /** \brief Random number generator */
            RNG                                                                 rng_;

            /** \brief A counter for the number of iterations of the algorithm */
            unsigned int                                                        iterations_;

            /** \brief The stretch factor in terms of graph spanners for SPARS to check against */
            double                                                              stretchFactor_;

            /** \brief The maximum number of failures before terminating the algorithm */
            unsigned int                                                        maxFailures_;

            /** \brief SPARS parameter for dense graph connection distance */
            double                                                              denseDelta_;

            /** \brief SPARS parameter for Sparse Roadmap connection distance */
            double                                                              sparseDelta_;

            /** \brief A holder for the last state added to D */
            base::State*                                                        lastState_;

        private:

            /** \brief Get all nodes in the sparse graph which are within sparseDelta_ of the given state. */
            void getSparseNeighbors( base::State* inState );

            /** \brief Get the visible neighbors */
            void getVisibleNeighbors( base::State* inState );

            /** \brief Compute distance between two milestones (this is simply distance between the states of the milestones) */
            double distanceFunction(const DenseVertex a, const DenseVertex b) const
            {
                return si_->distance(stateProperty_[a], stateProperty_[b]);
            }

            /** \brief Compute distance between two nodes in the sparse roadmap spanner. */
            double sparseDistanceFunction( const SparseVertex a, const SparseVertex b ) const
            {
                return si_->distance( sparseStateProperty_[a], sparseStateProperty_[b] );
            }

        };

    }
}

#endif
