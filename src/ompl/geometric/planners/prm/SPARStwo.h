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

#ifndef OMPL_GEOMETRIC_PLANNERS_SPARS_TWO_
#define OMPL_GEOMETRIC_PLANNERS_SPARS_TWO_

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
           @anchor gSPARStwo
           @par Short description
           SPARStwo is a variant of the SPARS algorithm which removes the
           dependency on having the dense graph, D.  It works through similar
           mechanics, but uses a different approach to identifying interfaces
           and computing shortest paths through said interfaces.
           @par External documentation
        */

        /** \brief <b> SPArse Roadmap Spanner Version 2.0 </b> */
        class SPARStwo : public base::Planner
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
            
            struct vertex_color_t {
                typedef boost::vertex_property_tag kind;
            };
            
            struct vertex_extent_t {
                typedef boost::vertex_property_tag kind;
            };
            /** \brief Maximum extent of a vertex. */
            struct vertex_interface_data_t {
                typedef boost::vertex_property_tag kind;
            };
            
            /** \brief containers for states which keep NULL safety at all times */
            class sstate_t
            {
            public:
                base::State* st;
                
                sstate_t()
                {
                    st = NULL;
                }
                
                sstate_t( base::State* state )
                {
                    st = state;
                }
                
                void operator=( base::State* state )
                {
                    st = state;
                }
                
                base::State* get()
                {
                    return st;
                }
                
                void clear( base::SpaceInformationPtr si )
                {
                    if( st != NULL )
                        si->freeState( st );
                    st = NULL;
                }
            };
            
            typedef std::pair< sstate_t, sstate_t > sstate_pair;
            typedef std::pair< unsigned long, unsigned long > vertex_pair;
            
            struct interface_data
            {
                sstate_pair points;
                sstate_pair sigmas;
                double      d;
                
                interface_data()
                {
                    d = 999999999;
                }
                
                void set_first( sstate_t p, sstate_t s, base::SpaceInformationPtr si )
                {
                    if( points.first.get() != NULL )
                        si->freeState( points.first.get() );
                    points.first = si->cloneState( p.get() );
                    if( sigmas.first.get() != NULL )
                        si->freeState( sigmas.first.get() );
                    sigmas.first = si->cloneState( s.get() );
                    
                    if( points.second.get() != NULL )
                    {
                        d = si->distance( points.first.get(), points.second.get() );
                    }
                }
                
                void set_second( sstate_t p, sstate_t s, base::SpaceInformationPtr si )
                {
                    if( points.second.get() != NULL )
                        si->freeState( points.second.get() );
                    points.second = si->cloneState( p.get() );
                    if( sigmas.second.get() != NULL )
                        si->freeState( sigmas.second.get() );
                    sigmas.second = si->cloneState( s.get() );

                    if( points.first.get() != NULL )
                    {
                        d = si->distance( points.first.get(), points.second.get() );
                    }
                }
                
                void clear( base::SpaceInformationPtr si )
                {
                    points.first.clear( si );
                    points.second.clear( si );
                    sigmas.first.clear( si );
                    sigmas.second.clear( si );
                    d = 999999999;
                }
            };
                        
            /** \brief the hash which maps pairs of neighbor points to pairs of states */
            typedef boost::unordered_map< vertex_pair, interface_data, boost::hash< vertex_pair > > interface_hash;
            
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
                boost::property < vertex_color_t, unsigned int,
                boost::property < vertex_interface_data_t, interface_hash > > > > > > >,
                boost::property < boost::edge_weight_t, double,
                boost::property < boost::edge_index_t, unsigned int> >
            > Graph;

            typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
            typedef boost::graph_traits<Graph>::edge_descriptor   Edge;

            typedef boost::shared_ptr< NearestNeighbors<Vertex> > RoadmapNeighbors;

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
            typedef boost::function<bool(const Vertex&, const Vertex&)> ConnectionFilter;

            /** \brief Constructor */
            SPARStwo(const base::SpaceInformationPtr &si, bool starStrategy = false);

            virtual ~SPARStwo(void);

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

            /** \brief Sets the stretch factor */
            void setStretchFactor( double t )
            {
                t_ = t;
            }
            
            /** \brief Sets vertex visibility range */
            void setSparseDelta( double D )
            {
                DELTA_ = D;
            }
            
            /** \brief Sets interface support tolerance */
            void setDenseDelta( double d )
            {
                delta_ = d;
            }
            
            /** \brief Sets the maximum failures until termination */
            void setMaxFailures( unsigned int m )
            {
                m_ = m;
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

            /** \brief Sample a valid random state, storing it in qNew_ (and returning it) */
            virtual base::State* sample();
            
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
            void setNearestNeighbors(void)
            {
                nn_.reset(new NN< Vertex >());
                if (!userSetConnectionStrategy_)
                    connectionStrategy_.clear();
                if (isSetup())
                    setup();
            }

            virtual void setup(void);

            const Graph& getRoadmap(void) const
            {
                return g_;
            }

            /** \brief Compute distance between two milestones (this is simply distance between the states of the milestones) */
            double distanceFunction(const Vertex a, const Vertex b) const
            {
                return si_->distance(stateProperty_[a], stateProperty_[b]);
            }

            /** \brief Compute distance between two milestones (this is simply distance between the states of the milestones) */
            unsigned int milestoneCount(void) const
            {
                return boost::num_vertices(g_);
            }

            const RoadmapNeighbors& getNearestNeighbors(void)
            {
                return nn_;
            }
            
        protected:
            
            /** \brief Free all the memory allocated by the planner */
            void freeMemory(void);

            /** \brief Checks to see if the sample needs to be added to ensure coverage of the space */
            bool checkAddCoverage();
            
            /** \brief Checks to see if the sample needs to be added to ensure connectivity */
            bool checkAddConnectivity();

            /** \brief Checks to see if the current sample reveals the existence of an interface, and if so, tries to bridge it. */
            bool checkAddInterface();
            
            /** \brief Checks vertex v for short paths through its region and adds when appropriate. */
            bool checkAddPath( Vertex v );
            
            /** \brief A reset function for resetting the failures count */
            void resetFailures();
            
            /** \brief Finds visible nodes in the graph near st */
            void findGraphNeighbors( base::State* st );
            
            /** \brief Finds the two closest nodes in the graph near st */
            std::vector< Vertex > findTwoClosest( base::State* st );
            
            /** \brief Approaches the graph from a given vertex */
            void approachGraph( Vertex v );
            
            /** \brief Finds the representative of the input state, st  */
            void findGraphRepresentative( base::State* st );
            
            /** \brief Finds representatives of samples near qNew_ which are not his representative */
            std::pair< std::vector< Vertex >, std::vector< base::State* > > findCloseRepresentatives();
            
            /** \brief High-level method which updates pair point information for repV_ with neighbor r */
            void updatePairPoints( Vertex rep, sstate_t q, Vertex r, sstate_t s );

            /** \brief Computes all nodes which qualify as a candidate v" for v and vp */
            std::vector<Vertex> computeVPP( Vertex v, Vertex vp );

            /** \brief Computes all nodes which qualify as a candidate x for v, v', and v" */
            std::vector<Vertex> computeX( Vertex v, Vertex vp, Vertex vpp );

            /** \brief Rectifies indexing order for accessing the vertex data */
            vertex_pair index( Vertex vp, Vertex vpp );
            
            /** \brief Retrieves the Vertex data associated with v,vp,vpp */
            interface_data& getData( Vertex v, Vertex vp, Vertex vpp );
            
            void setData( Vertex v, Vertex vp, Vertex vpp, interface_data d );
            
            /** \brief Performs distance checking for the candidate new state, q against the current information */
            void distanceCheck( Vertex rep, sstate_t q, Vertex r, sstate_t s, Vertex rp );

            /** \brief When a new guard is added at state st, finds all guards who must abandon their interface information and deletes that information */
            void abandonLists( base::State* st );
            
            /** \brief Deletes all the states in a vertex's lists */
            void deletePairInfo( Vertex v );
            
            /** \brief Construct a guard for a given state (\e state) and store it in the nearest neighbors data structure */
            virtual Vertex addGuard(base::State *state, unsigned int c = 0);

            /** \brief Connect two guards in the roadmap */
            virtual void connect( Vertex v, Vertex vp );
            
            /** \brief Make two milestones (\e m1 and \e m2) be part of the same connected component. The component with fewer elements will get the id of the component with more elements. */
            void uniteComponents(Vertex m1, Vertex m2);

            /** Thread that checks for solution */
            void checkForSolution (const base::PlannerTerminationCondition &ptc, base::PathPtr &solution);

            /** \brief Check if there exists a solution, i.e., there exists a pair of milestones such that the first is in \e start and the second is in \e goal, and the two milestones are in the same connected component. If a solution is found, the path is saved. */
            bool haveSolution(const std::vector<Vertex> &start, const std::vector<Vertex> &goal, base::PathPtr &solution);

            /** \brief Returns the value of the addedSolution_ member. */
            bool addedNewSolution (void) const;

            /** \brief Given two milestones from the same connected component, construct a path connecting them and set it as the solution */
            virtual base::PathPtr constructSolution(const Vertex start, const Vertex goal) const;
            
            /** \brief Flag indicating whether the default connection strategy is the Star strategy */
            bool                                                   starStrategy_;

            /** \brief Sampler user for generating valid samples in the state space */
            base::ValidStateSamplerPtr                             sampler_;

            /** \brief Sampler user for generating random in the state space */
            base::StateSamplerPtr                                  simpleSampler_;

            /** \brief Nearest neighbors data structure */
            RoadmapNeighbors                                       nn_;

            /** \brief Connectivity graph */
            Graph                                                  g_;

            /** \brief Array of start milestones */
            std::vector<Vertex>                                    startM_;

            /** \brief Array of goal milestones */
            std::vector<Vertex>                                    goalM_;
            
            /** \brief Vertex for performing nearest neighbor queries. */
            Vertex                                                 query_v_;
            
            /** \brief Stretch Factor as per graph spanner literature (multiplicative bound on path quality) */
            double                                                 t_;

            /** \brief Maximum visibility range for nodes in the graph */
            double                                                 DELTA_;

            /** \brief Maximum range for allowing two samples to support an interface */
            double                                                 delta_;

            /** \brief The number of consecutive failures to add to the graph before termination */
            unsigned int                                           m_;
            
            /** \brief A pointer to the most recent sample we have come up with */
            base::State*                                           qNew_;
            
            /** \brief A pointer holding a temporary state used for additional sampling processes */
            base::State*                                           holdState_;
            
            /** \brief The whole neighborhood set which has been most recently computed */
            std::vector< Vertex >                                  graphNeighborhood_;
            
            /** \brief The visible neighborhood set which has been most recently computed */
            std::vector< Vertex >                                  visibleNeighborhood_;
            
            /** \brief A holder to remember who qNew_'s representative in the graph is. */
            Vertex                                                 repV_;
            
            /** \brief Access to the internal base::state at each Vertex */
            boost::property_map<Graph, vertex_state_t>::type       stateProperty_;

            /** \brief Access to the number of total connection attempts for a vertex */
            boost::property_map<Graph,
                vertex_total_connection_attempts_t>::type          totalConnectionAttemptsProperty_;

            /** \brief Access to the number of successful connection attempts for a vertex */
            boost::property_map<Graph,
                vertex_successful_connection_attempts_t>::type     successfulConnectionAttemptsProperty_;

            /** \brief A path simplifier used to simplify dense paths added to the graph */
            PathSimplifierPtr                                      psimp_;

            /** \brief Access to the weights of each Edge */
            boost::property_map<Graph, boost::edge_weight_t>::type weightProperty_;

            /** \brief Access to the indices of each Edge */
            boost::property_map<Graph, boost::edge_index_t>::type  edgeIDProperty_;
            
            /** \brief Access to the colors for the vertices */
            boost::property_map<Graph, vertex_color_t>::type       colorProperty_;
            
            /** \brief Access to the stored extent of each Node in G */
            boost::property_map<Graph, vertex_extent_t>::type      extentProperty_;
            
            /** \brief Access to the interface pair information for the vertices */
            boost::property_map<Graph, vertex_interface_data_t>::type interfaceDataProperty_;

            /** \brief Data structure that maintains the connected components */
            boost::disjoint_sets<
                boost::property_map<Graph, boost::vertex_rank_t>::type,
                boost::property_map<Graph, boost::vertex_predecessor_t>::type >
                                                                   disjointSets_;

            /** \brief Maximum unique id number used so for for edges */
            unsigned int                                           maxEdgeID_;

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

            /** \brief Mutex to guard access to the Graph member (g_) */
            mutable boost::mutex                                   graphMutex_;
            
            /** \brief A counter for the number of iterations of the algorithm */
            unsigned int                                           iterations_;
        };

    }
}

#endif
