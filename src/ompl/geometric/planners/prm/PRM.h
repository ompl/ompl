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

/* Author: Ioan Sucan, James D. Marble */

#ifndef OMPL_GEOMETRIC_PLANNERS_PRM_PRM_
#define OMPL_GEOMETRIC_PLANNERS_PRM_PRM_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/pending/disjoint_sets.hpp>
#include <boost/function.hpp>
#include <utility>
#include <vector>
#include <map>

namespace ompl
{

    namespace geometric
    {

        /**
           @anchor gPRM
           @par Short description
           PRM is a planner that constructs a roadmap of milestones
           that approximate the connectivity of the state space. The
           milestones are valid states in the state space. Near-by
           milestones are connected by valid motions. Finding a motion
           plan that connects two given states is reduced to a
           discrete search (this implementation uses A*) in the
           roadmap.
           @par External documentation
           L.E. Kavraki, P.Švestka, J.-C. Latombe, and M.H. Overmars,
           Probabilistic roadmaps for path planning in high-dimensional configuration spaces,
           <em>IEEE Trans. on Robotics and Automation</em>, vol. 12, pp. 566–580, Aug. 1996.
           DOI: <a href="http://dx.doi.org/10.1109/70.508439">10.1109/70.508439</a><br>
           <a href="http://ieeexplore.ieee.org/ielx4/70/11078/00508439.pdf?tp=&arnumber=508439&isnumber=11078">[PDF]</a>
           <a href="http://www.kavrakilab.org/robotics/prm.html">[more]</a>
        */

        /** \brief Probabilistic RoadMap planner */
        class PRM : public base::Planner
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

            struct vertex_validity_t {
                typedef boost::vertex_property_tag kind;
            };

            struct edge_validity_t {
                typedef boost::edge_property_tag kind;
            };

	    /** \brief Flag indicating validity of an edge of a vertex */
	    enum ValidityType
	    {
		UNKNOWN = 0, VALID = 1, INVALID = 2
	    };
	    struct ValidityDescriptor
	    {
		ValidityDescriptor(ValidityType t = UNKNOWN, unsigned int v = 0) :
		    type(t), version(v)
		{
		}
		unsigned int type : 2;
		unsigned int version : sizeof(unsigned int) * 4 - 2;
	    };

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
                boost::property < vertex_validity_t, ValidityDescriptor,
                boost::property < boost::vertex_predecessor_t, unsigned long int,
		boost::property < boost::vertex_rank_t, unsigned long int > > > > > >,
                boost::property < boost::edge_weight_t, double,
		boost::property < boost::edge_index_t, unsigned int,
		boost::property < edge_validity_t, ValidityDescriptor > > >
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

	    /** \brief Configuration options for PRM */
	    struct Configuration
	    {
		Configuration() : 
		    starStrategy(false),
		    lazy(false)
		{
		}
		
		bool starStrategy;
		bool lazy;
	    };
	    		
            /** \brief Constructor */
            PRM(const base::SpaceInformationPtr &si, const Configuration &config = Configuration());

            virtual ~PRM(void);

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
            void setConnectionFilter(const ConnectionFilter& connectionFilter)
            {
                connectionFilter_ = connectionFilter;
            }

            virtual void getPlannerData(base::PlannerData &data) const;

            /** \brief If the user desires, the roadmap can be
                improved for the given time (seconds). The solve()
                method will also improve the roadmap, as needed.*/
            virtual void growRoadmap(double growTime);

            /** \brief If the user desires, the roadmap can be
                improved until a given condition is true. The solve()
                method will also improve the roadmap, as needed.*/
            virtual void growRoadmap(const base::PlannerTerminationCondition &ptc);

            /** \brief Attempt to connect disjoint components in the roadmap
                using random bouncing motions (the PRM expansion step) for the
                given time (seconds). */
            virtual void expandRoadmap(double expandTime);

            /** \brief Attempt to connect disjoint components in the roadmap
                using random bouncing motions (the PRM expansion step) until the
                given condition evaluates true. */
            virtual void expandRoadmap(const base::PlannerTerminationCondition &ptc);

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

	    /** \brief When running LazyPRM, use this function to notify the planner that the validity of the states in its maintained roadmap
		may have changed. This allows reusing the constructed roadmap structure, but makes the planner re-check previously validated states. */
	    void resetCachedValidityInformation(void);
	    
            virtual void clear(void);

            /** \brief Set a different nearest neighbors datastructure */
            template<template<typename T> class NN>
            void setNearestNeighbors(void)
            {
                nn_.reset(new NN<Vertex>());
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

            /** \brief Construct a milestone for a given state (\e state) and store it in the nearest neighbors data structure */
            virtual Vertex addMilestone(base::State *state);

            /** \brief Make two milestones (\e m1 and \e m2) be part of the same connected component. The component with fewer elements will get the id of the component with more elements. */
            void uniteComponents(Vertex m1, Vertex m2);

            /** \brief Randomly sample the state space, add and connect milestones
                 in the roadmap. Stop this process when the termination condition
                 \e ptc returns true.  Use \e workState as temporary memory. */
            void growRoadmap(const base::PlannerTerminationCondition &ptc, base::State *workState);

            /** \brief Attempt to connect disjoint components in the
                roadmap using random bounding motions (the PRM
                expansion step) */
            void expandRoadmap(const base::PlannerTerminationCondition &ptc, std::vector<base::State*> &workStates);

            /** Thread that checks for solution */
            void checkForSolution (const base::PlannerTerminationCondition &ptc, base::PathPtr &solution);

            /** \brief Check if there exists a solution, i.e., there exists a pair of milestones such that the first is in \e start and the second is in \e goal, and the two milestones are in the same connected component. If a solution is found, the path is saved. */
            bool haveSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals, base::PathPtr &solution);

            /** \brief Returns the value of the addedSolution_ member. */
            bool addedNewSolution (void) const;

            /** \brief Given two milestones from the same connected component, construct a path connecting them and set it as the solution */
            virtual base::PathPtr constructSolution(const Vertex start, const Vertex goal) const;

            /** \brief Flag indicating whether the default connection strategy is the Star strategy */
            bool                                                   starStrategy_;

	    /** \brief Set to true if algorithm should run as LazyPRM */
	    bool                                                   lazy_;
	    
	    /** \brief Between repeated calls to solve(), the validity of known vertices may have changed.
		In that case, all vertices that are marked as valid need to become unknown. Instead of performing that operation
		for all previoulsy checked vertices, a version is associated the the check operation. If the version of a valid 
		vertex is older than the current version, it is considered unknown. This value is reset to 0 (along with clearing the roadmap)
		when clear() is called. The version is increased by calling resetCachedValidityInformation() */
	    unsigned int                                           currentValidityVersion_;
	    
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

            /** \brief Access to the internal base::state at each Vertex */
            boost::property_map<Graph, vertex_state_t>::type       stateProperty_;

            /** \brief Access to the number of total connection attempts for a vertex */
            boost::property_map<Graph,
                vertex_total_connection_attempts_t>::type          totalConnectionAttemptsProperty_;

            /** \brief Access to the number of successful connection attempts for a vertex */
            boost::property_map<Graph,
                vertex_successful_connection_attempts_t>::type     successfulConnectionAttemptsProperty_;

	    /** \brief Access the validity state of a vertex */
	    boost::property_map<Graph, vertex_validity_t>::type    vertexValidityProperty_;
	    
            /** \brief Access to the weights of each Edge */
            boost::property_map<Graph, boost::edge_weight_t>::type weightProperty_;

            /** \brief Access to the indices of each Edge */
            boost::property_map<Graph, boost::edge_index_t>::type  edgeIDProperty_;

	    /** \brief Access the validity state of an edge */
	    boost::property_map<Graph, edge_validity_t>::type      edgeValidityProperty_;
	    
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

        };

    }
}

#endif
