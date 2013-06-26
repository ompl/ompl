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

#include "ompl/geometric/planners/prm/SPARS.h"
#include "ompl/geometric/planners/prm/ConnectionStrategy.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include "ompl/datastructures/NearestNeighborsLinear.h"
#include "ompl/datastructures/PDF.h"
#include <boost/lambda/bind.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH
#define foreach_reverse BOOST_REVERSE_FOREACH

namespace ompl
{
    namespace magic
    {
        /** \brief Maximum number of sampling attempts to find a valid state,
            without checking whether the allowed time elapsed. This value
            should not really be changed. */
        static const unsigned int FIND_VALID_STATE_ATTEMPTS_WITHOUT_TIME_CHECK = 2;

        /** \brief The number of steps to take for a random bounce
            motion generated as part of the expansion step of SPARS. */
        static const unsigned int MAX_RANDOM_BOUNCE_STEPS   = 5;

        /** \brief The number of nearest neighbors to consider by
            default in the construction of the SPARS roadmap */
        static const unsigned int DEFAULT_NEAREST_NEIGHBORS = 100;

        /** \brief The time in seconds for a single roadmap building operation (dt)*/
        static const double ROADMAP_BUILD_TIME = 0.2;
    }
}

ompl::geometric::SPARS::SPARS(const base::SpaceInformationPtr &si) :
    base::Planner(si, "SPARS"),
    stateProperty_(boost::get(vertex_state_t(), g_)),
    sparseStateProperty_(boost::get(vertex_state_t(), s_)),
    sparseColorProperty_(boost::get(vertex_color_t(), s_)),
    representativesProperty_(boost::get(vertex_representative_t(), g_)),
    nonInterfaceListsProperty_(boost::get(vertex_list_t(), s_)),
    interfaceListsProperty_(boost::get(vertex_interface_list_t(), s_)),
    totalConnectionAttemptsProperty_(boost::get(vertex_total_connection_attempts_t(), g_)),
    successfulConnectionAttemptsProperty_(boost::get(vertex_successful_connection_attempts_t(), g_)),
    weightProperty_(boost::get(boost::edge_weight, g_)),
    edgeIDProperty_(boost::get(boost::edge_index, g_)),
    disjointSets_(boost::get(boost::vertex_rank, g_),
                  boost::get(boost::vertex_predecessor, g_)),
    sparseDJSets_(boost::get(boost::vertex_rank, s_),
                  boost::get(boost::vertex_predecessor, s_)),
    maxEdgeID_(0),
    maxLinkID_(0),
    userSetConnectionStrategy_(false),
    addedSolution_(false),
    iterations_(0),
    t_(3),
    m_(1000),
    denseDelta_(0.5),
    sparseDelta_(20)
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;
    
    psimp_.reset(new PathSimplifier(si_));

    Planner::declareParam<double>("stretch_factor", this, &SPARS::setStretchFactor, &SPARS::getStretchFactor, "1.1:0.1:3.0");
    Planner::declareParam<double>("sparse_delta", this, &SPARS::setSparseDelta, &SPARS::getSparseDelta, "1.0:0.5:30.0");
    Planner::declareParam<double>("dense_delta", this, &SPARS::setDenseDelta, &SPARS::getDenseDelta, "0.02:0.02:1.0");
    Planner::declareParam<unsigned int>("max_failures", this, &SPARS::setMaxFailures, &SPARS::getMaxFailures, "100:10:3000");
}

ompl::geometric::SPARS::~SPARS(void)
{
    freeMemory();
}

void ompl::geometric::SPARS::setup(void)
{
    Planner::setup();
    if (!nn_)
        nn_.reset(new NearestNeighborsGNAT<Vertex>());
    nn_->setDistanceFunction(boost::bind(&SPARS::distanceFunction, this, _1, _2));
    if (!snn_)
        snn_.reset(new NearestNeighborsGNAT<Node>());
    snn_->setDistanceFunction(boost::bind(&SPARS::sparseDistanceFunction, this, _1, _2));
    if (!connectionStrategy_)
    {
        connectionStrategy_ = KStarStrategy<Vertex>(boost::bind(&SPARS::milestoneCount, this), nn_, si_->getStateDimension());
    }
    if (!connectionFilter_)
        connectionFilter_ = boost::lambda::constant(true);
}

void ompl::geometric::SPARS::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
    Planner::setProblemDefinition(pdef);
    clearQuery();
}

void ompl::geometric::SPARS::clearQuery(void)
{
    startM_.clear();
    goalM_.clear();
    pis_.restart();
}

void ompl::geometric::SPARS::clear(void)
{
    Planner::clear();
    sampler_.reset();
    simpleSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    if (snn_)
        snn_->clear();
    clearQuery();
    maxEdgeID_ = 0;
}

void ompl::geometric::SPARS::freeMemory(void)
{
    foreach (Vertex v, boost::vertices(g_))
    {
        if( stateProperty_[v] != NULL )
        {
            si_->freeState(stateProperty_[v]);
            stateProperty_[v] = NULL;
        }
    }
    foreach (Node n, boost::vertices(s_))
    {
        if( sparseStateProperty_[n] != NULL )
        {
            si_->freeState(sparseStateProperty_[n]);
            sparseStateProperty_[n] = NULL;
        }
    }
    s_.clear();
    g_.clear();
}

ompl::geometric::SPARS::Vertex ompl::geometric::SPARS::addSample( void )
{
    if (!isSetup())
        setup();
    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();
    
    base::State *workState = si_->allocState();
    Vertex ret = addSample(workState);
    si_->freeState(workState);
    return ret;
}

ompl::geometric::SPARS::Vertex ompl::geometric::SPARS::addSample(base::State *workState)
{
    bool found = false;
    Vertex ret = NULL;
    while (!found)
    {
        unsigned int attempts = 0;
        do 
        {
            found = sampler_->sample(workState);
        } while (!found);
    }
    if (found)
        ret = addMilestone(si_->cloneState(workState));
    return ret;
}

void ompl::geometric::SPARS::checkForSolution (const base::PlannerTerminationCondition &ptc,
                                             base::PathPtr &solution)
{
    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

    // Check for any new goal states
    if (goal->maxSampleCount() > goalM_.size())
    {
        const base::State *st = pis_.nextGoal();
        if (st)
        {
            addMilestone(si_->cloneState(st));
            goalM_.push_back( addGuard( si_->cloneState(st), 0) );
        }
    }

    // Check for a solution
    addedSolution_ = haveSolution (startM_, goalM_, solution);
}

bool ompl::geometric::SPARS::haveSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals, base::PathPtr &solution)
{
    base::Goal *g = pdef_->getGoal().get();
    double sl = -1.0; // cache for solution length
    foreach (Vertex start, starts)
    {
        foreach (Vertex goal, goals)
        {
            if (boost::same_component(start, goal, sparseDJSets_) &&
                g->isStartGoalPairValid(sparseStateProperty_[goal], sparseStateProperty_[start]))
            {
                solution = constructSolution(start, goal);
                return true;
            }
        }
    }

    return false;
}

bool ompl::geometric::SPARS::addedNewSolution (void) const
{
    return addedSolution_;
}

bool ompl::geometric::SPARS::reachedFailureLimit (void) const
{
    return iterations_ >= m_;
}

ompl::base::PlannerStatus ompl::geometric::SPARS::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();

    if( boost::num_vertices( g_ ) < 1 )
    {
        sparseQueryVertex_ = boost::add_vertex( s_ );
        queryVertex_ = boost::add_vertex( g_ );
        sparseStateProperty_[sparseQueryVertex_] = NULL;
        stateProperty_[queryVertex_] = NULL;
    }

    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

    if (!goal)
    {
        OMPL_ERROR("Goal undefined or unknown type of goal");
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    // Add the valid start states as milestones
    while (const base::State *st = pis_.nextStart())
    {
        addMilestone(si_->cloneState(st));
        startM_.push_back(addGuard(si_->cloneState(st), 0));
    }

    if (startM_.size() == 0)
    {
        OMPL_ERROR("There are no valid initial states!");
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("Insufficient states in sampleable goal region");
        return base::PlannerStatus::INVALID_GOAL;
    }

    // Ensure there is at least one valid goal state
    if (goal->maxSampleCount() > goalM_.size() || goalM_.empty())
    {
        const base::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
        if (st)
        {
            addMilestone(si_->cloneState(st));
            goalM_.push_back( addGuard(si_->cloneState(st), 0) );
        }

        if (goalM_.empty())
        {
            OMPL_ERROR("Unable to find any valid goal states");
            return base::PlannerStatus::INVALID_GOAL;
        }
    }

    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();
    if (!simpleSampler_)
        simpleSampler_ = si_->allocStateSampler();

    unsigned int nrStartStates = boost::num_vertices(g_);
    OMPL_INFORM("Starting with %u states", nrStartStates);

    std::vector<base::State*> xstates(magic::MAX_RANDOM_BOUNCE_STEPS);
    si_->allocStates(xstates);
    bool grow = true;

    // Reset addedSolution_ member
    addedSolution_ = false;
    base::PathPtr sol;
    sol.reset();

    //Construct planner termination condition which also takes M into account
    base::PlannerOrTerminationCondition ptcOrFail(ptc,base::PlannerTerminationCondition(boost::bind(&SPARS::reachedFailureLimit, this)));

    while ( !ptcOrFail() )
    {
        //Generate a single sample, and attempt to connect it to nearest neighbors.
        Vertex q = addSample();

        //Now that we've added to D, try adding to S
        //Start by figuring out who our neighbors are
        std::vector<Node> neigh = getSparseNeighbors( last_state );
        std::vector<Node> vis_neigh = getVisibleNeighbors( last_state );
        //Check for addition for Coverage
        if( !checkAddCoverage( neigh ) )
        {
            //If not for Coverage, then Connectivity
            if( !checkAddConnectivity( neigh ) )
            {
                //Check for the existence of an interface
                if( !checkAddInterface( neigh, vis_neigh, q ) )
                {
                    //Then check to see if it's on an interface
                    const std::vector<Vertex>& inf = interfaceCheck( q );
                    if( inf.size() > 0 )
                    {
                        //Check for addition for spanner prop
                        if( !checkAddPath( q, inf ) )
                        {
                            //All of the tests have failed.  Report failure for the sample
                            ++iterations_;
                        }
                    }
                    else
                    {
                        //There's no interface here, so drop it
                        ++iterations_;
                    }
                }
            }
        }
    }

    checkForSolution(ptc, sol);

    if (sol)
    {
        if (addedNewSolution())
            pdef_->addSolutionPath (sol);
        else
            // the solution is exact, but not as short as we'd like it to be
            pdef_->addSolutionPath (sol, true, 0.0);
    }

    si_->freeStates(xstates);

    // Return true if any solution was found.
    return sol ? (addedNewSolution() ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::APPROXIMATE_SOLUTION) : base::PlannerStatus::TIMEOUT;
}

ompl::base::PlannerStatus ompl::geometric::SPARS::solve(const base::PlannerTerminationCondition &ptc, unsigned int maxFail )
{
    m_ = maxFail;
    return solve( ptc );
}

ompl::geometric::SPARS::Vertex ompl::geometric::SPARS::addMilestone(base::State *state)
{
    Vertex m = boost::add_vertex(g_);
    stateProperty_[m] = state;
    totalConnectionAttemptsProperty_[m] = 1;
    successfulConnectionAttemptsProperty_[m] = 0;

    // Initialize to its own (dis)connected component.
    disjointSets_.make_set(m);
    
    // Which milestones will we attempt to connect to?
    if (!connectionStrategy_)
        throw Exception(name_, "No connection strategy!");

    const std::vector<Vertex>& neighbors = connectionStrategy_(m);

    last_state = state;
    
    foreach (Vertex n, neighbors)
    {
        totalConnectionAttemptsProperty_[m]++;
        totalConnectionAttemptsProperty_[n]++;
        if (si_->checkMotion(stateProperty_[m], stateProperty_[n]))
        {
            successfulConnectionAttemptsProperty_[m]++;
            successfulConnectionAttemptsProperty_[n]++;
            const double weight = distanceFunction(m, n);
            const unsigned int id = maxEdgeID_++;
            const Graph::edge_property_type properties(weight, id);
            
            boost::add_edge(m, n, properties, g_);
            uniteComponents(n, m);
        }
    }
    
    nn_->add(m);
    
    //Need to update representative information here...
    calculateRepresentative(m);
    
    addToRep( m, representativesProperty_[m], getInterfaceNeighborRepresentatives(m) );
    foreach( Vertex qp, getInterfaceNeighborhood( m ) )
    {
        removeFromRep( qp, representativesProperty_[qp] );
        addToRep( qp, representativesProperty_[qp], getInterfaceNeighborRepresentatives( qp ) );
    }

    return m;
}

ompl::geometric::SPARS::Node ompl::geometric::SPARS::addGuard(base::State *state, unsigned int type)
{
    Node v = boost::add_vertex( s_ );
    sparseStateProperty_[v] = state;
    sparseColorProperty_[v] = type;
    
    sparseDJSets_.make_set(v);
    
    snn_->add(v);
    updateReps( v );
    
    resetFailures();
    return v;
}

void ompl::geometric::SPARS::uniteComponents(Vertex m1, Vertex m2)
{
    disjointSets_.union_set(m1, m2);
}

void ompl::geometric::SPARS::uniteSparseComponents(Node m1, Node m2)
{
    sparseDJSets_.union_set(m1, m2);
}

void ompl::geometric::SPARS::connectSparsePoints( Node v, Node vp )
{
    const double weight = sparseDistanceFunction(v, vp);
    const unsigned int id = maxLinkID_++;
    const SpannerGraph::edge_property_type properties(weight, id);
    boost::add_edge(v, vp, properties, s_);
    uniteSparseComponents( v, vp );
}

void ompl::geometric::SPARS::connectDensePoints( Vertex v, Vertex vp )
{
    const double weight = distanceFunction(v, vp);
    const unsigned int id = maxEdgeID_++;
    const Graph::edge_property_type properties( weight, id);
    boost::add_edge(v, vp, properties, g_);
    uniteComponents( v, vp );
}

bool ompl::geometric::SPARS::checkAddCoverage(std::vector<Node> neigh)
{
    //For each of these neighbors,
    foreach( Node n, neigh )
    {
        //If path between is free
        PathGeometric *p = new PathGeometric(si_);
        p->append(last_state);
        p->append(sparseStateProperty_[n]);
        if( p->check() )
        {
            //Abort out and return false
            return false;
        }
    }
    //No free paths means we add for coverage
    addGuard( si_->cloneState(last_state), 0 );
    return true;
}

bool ompl::geometric::SPARS::checkAddConnectivity(std::vector<Node> neigh)
{
    std::vector< Node > links;
    //For each neighbor
    for( unsigned int i=0; i<neigh.size(); ++i )
    {
        //For each other neighbor
        for( unsigned int j=i+1; j<neigh.size(); ++j )
        {
            //If they are in different components
            if( !boost::same_component( neigh[i], neigh[j], sparseDJSets_ ) )
            {
                //If the paths between are collision free
                PathGeometric *a = new PathGeometric(si_);
                PathGeometric *b = new PathGeometric(si_);
                
                a->append( last_state );
                b->append( last_state );
                a->append( sparseStateProperty_[neigh[i]] );
                b->append( sparseStateProperty_[neigh[j]] );
                if( a->check() && b->check() )
                {
                    links.push_back( neigh[i] );
                    links.push_back( neigh[j] );
                }                
            }
        }
    }
    
    if( links.size() != 0 )
    {
        //Add the node
        Node g = addGuard( si_->cloneState(last_state), 1 );
        
        for( unsigned int i=0; i<links.size(); ++i )
        {
            //If there's no edge
            if( !boost::edge(g, links[i], s_).second )
            {
                //And the components haven't been united by previous links
                if( !boost::same_component( links[i], g, sparseDJSets_ ) )
                {
                    connectSparsePoints( g, links[i] );
                }
            }
        }
        return true;
    }
    return false;
}

bool ompl::geometric::SPARS::checkAddInterface( const std::vector<ompl::geometric::SPARS::Node>& graphNeighborhood_, const std::vector<ompl::geometric::SPARS::Node>& visibleNeighborhood_, ompl::geometric::SPARS::Vertex q )
{
    //If we have more than 1 neighbor
    if( visibleNeighborhood_.size() > 1 )
    {
        //If our closest neighbors are also visible
        if( graphNeighborhood_[0] == visibleNeighborhood_[0] && graphNeighborhood_[1] == visibleNeighborhood_[1] )
        {
            //If our two closest neighbors don't share an edge
            if( !boost::edge( visibleNeighborhood_[0], visibleNeighborhood_[1], s_ ).second )
            {
                //If they can be directly connected
                if( si_->checkMotion( sparseStateProperty_[visibleNeighborhood_[0]], sparseStateProperty_[visibleNeighborhood_[1]] ) )
                {
                    //Connect them
                    connectSparsePoints( visibleNeighborhood_[0], visibleNeighborhood_[1] );
                    //And report that we added to the roadmap
                    resetFailures();
                    //Report success
                    return true;
                }
                else
                {
                    //Add the new node to the graph, to bridge the interface
                    Node v = addGuard( si_->cloneState( stateProperty_[q] ), 2 );
                    connectSparsePoints( v, visibleNeighborhood_[0] );
                    connectSparsePoints( v, visibleNeighborhood_[1] );
                    //Report success
                    return true;
                }
            }
        }
    }
    return false;
}

bool ompl::geometric::SPARS::checkAddPath(ompl::geometric::SPARS::Vertex q, const std::vector<ompl::geometric::SPARS::Vertex>& neigh )
{
    bool ret = false;
    //Get q's representative => v
    Node v = representativesProperty_[q];
    //Extract the representatives of neigh => n_rep
    std::vector< Node > n_rep;
    
    foreach( Vertex qp, neigh )
    {
        //If we haven't tracked this representative
        if( std::find( n_rep.begin(), n_rep.end(), representativesProperty_[qp] ) == n_rep.end() )
        {
            n_rep.push_back( representativesProperty_[qp] );
        }
    }
    
    //for each v' in n_rep
    for( unsigned int i = 0; i < n_rep.size() && !ret; ++i )
    {
        Node vp = n_rep[i];
        //Identify appropriate v" candidates => vpps
        std::vector< Node > vpps = computeVPP( v, vp );
        
        foreach( Node vpp, vpps )
        {
            double s_max = 0;
            //Find the X nodes to test
            std::vector< Node > xs = computeX( v, vp, vpp );
            //For each x in xs
            foreach( Node x, xs )
            {
                //Compute/Retain MAXimum distance path thorugh S
                double dist = ( si_->distance( sparseStateProperty_[x], sparseStateProperty_[v] ) + si_->distance( sparseStateProperty_[v], sparseStateProperty_[vp] ) )/2.0;
                if( dist > s_max )
                    s_max = dist;
            }
            
            PathGeometric *best_dpath = NULL;
            Vertex best_qpp = NULL;
            double d_min = 9999999; //Insanely big number
                                         //For each vpp in vpps
            for( unsigned int j=0; j<vpps.size() && !ret; ++j )
            {
                Node vpp = vpps[j];
                //For each q", which are stored interface nodes on v for i(vpp,v)
                foreach( Vertex qpp, interfaceListsProperty_[v][vpp] )
                {
                    if( representativesProperty_[qpp] != v )
                    {
                        throw Exception(name_, "Representative mismatch!");
                    }
                    else
                    {
                        //If they happen to be the one and same node
                        if( q == qpp )
                        {
                            Vertex junk;
                            //Do a sanity check to make sure this TRULY is the case
                            if( getInterfaceNeighbor( q, vp, junk ) && getInterfaceNeighbor( q, vpp, junk ) )
                            {
                                best_dpath = new PathGeometric( si_ );
                                best_dpath->append( stateProperty_[q] );
                                best_qpp = qpp;
                                d_min = 0;
                            }
                            else
                            {
                                throw Exception(name_, "I don't even know what happened!");
                            }
                        }
                        else
                        {
                            //Compute/Retain MINimum distance path on D through q, q"
                            PathGeometric *p = densePath( q, qpp );
                            if( p != NULL )
                            {
                                double length = p->length();
                                if( length < d_min )
                                {
                                    d_min = length;
                                    if( best_dpath != NULL )
                                    {
                                        delete best_dpath;
                                    }
                                    best_dpath = p;
                                    best_qpp = qpp;
                                }
                                else
                                {
                                    delete p;
                                }
                            }
                        }
                    }
                }
                
                //If the spanner property is violated for these paths
                if( s_max > t_ * d_min )
                {
                    //Need to augment this path with the appropriate neighbor information
                    Vertex na;
                    bool successa = getInterfaceNeighbor( q, vp, na );
                    Vertex nb;
                    bool successb = getInterfaceNeighbor( best_qpp, vpp, nb );
                    best_dpath->reverse();
                    best_dpath->append( stateProperty_[na] );
                    best_dpath->reverse();
                    best_dpath->append( stateProperty_[nb] );
                    
                    if( representativesProperty_[na] != vp || representativesProperty_[nb] != vpp )
                    {
                        throw Exception(name_, "Inappropriate representatives found.");
                    }
                    //Add the dense path to the spanner
                    addPathToSpanner( best_dpath, vpp, vp );
                    
                    //Report success
                    ret = true;
                }
            }
        }
    }
    return ret;
}

double ompl::geometric::SPARS::avgValence( void )
{
    double degree = 0;
    foreach( Vertex v, boost::vertices(s_))
    {
        degree += (double)boost::out_degree( v, s_ );
    }
    degree /= (double)boost::num_vertices( s_ );
    return degree;
}

void ompl::geometric::SPARS::resetFailures( void )
{
    iterations_ = 0;
}

void ompl::geometric::SPARS::approachGraph( Vertex v )
{
    std::vector< Vertex > hold;
    nn_->nearestR( v, 3*denseDelta_, hold );
    
    std::vector< Vertex > neigh;
    for( unsigned int i=0; i<hold.size(); ++i )
    {
        if( si_->checkMotion( stateProperty_[v], stateProperty_[hold[i]] ) )
        {
            neigh.push_back( hold[i] );
        }
    }
    
    foreach( Vertex vp, neigh )
    {
        connectDensePoints( v, vp );
    }
}

void ompl::geometric::SPARS::approachSpanner( Node n )
{
    std::vector< Node > hold;
    snn_->nearestR( n, sparseDelta_, hold );
    
    std::vector< Node > neigh;
    for( unsigned int i=0; i<hold.size(); ++i )
    {
        if( si_->checkMotion( sparseStateProperty_[n], sparseStateProperty_[hold[i]] ) )
        {
            neigh.push_back( hold[i] );
        }
    }
    
    foreach( Node np, neigh )
    {
        connectSparsePoints( n, np );
    }
}

std::vector<ompl::geometric::SPARS::Node> ompl::geometric::SPARS::getSparseNeighbors( base::State* inState )
{
    sparseStateProperty_[sparseQueryVertex_] = inState;
    
    std::vector< Node > ret;
    snn_->nearestR( sparseQueryVertex_, sparseDelta_, ret );
        
    sparseStateProperty_[sparseQueryVertex_] = NULL;
    
    return ret;
}

std::vector<ompl::geometric::SPARS::Node> ompl::geometric::SPARS::getVisibleNeighbors( base::State* inState )
{
    sparseStateProperty_[sparseQueryVertex_] = inState;
    
    std::vector< Node > hold;
    snn_->nearestR( sparseQueryVertex_, sparseDelta_, hold );
    
    sparseStateProperty_[sparseQueryVertex_] = NULL;
 
    std::vector< Node > ret;
    
    //Now that we got the neighbors from the NN, we must remove any we can't see
    for( unsigned int i=0; i<hold.size(); ++i )
    {
        if( si_->checkMotion( inState, sparseStateProperty_[hold[i]] ) )
        {
            ret.push_back( hold[i] );
        }
    }

    return ret;
}

bool ompl::geometric::SPARS::getInterfaceNeighbor( Vertex q, Node rep, Node& ret )
{
    foreach( Vertex vp, boost::adjacent_vertices( q, g_ ) )
    {
        if( representativesProperty_[vp] == rep )
        {
            if( distanceFunction( q, vp ) <= denseDelta_ )
            {
                ret = vp;
                return true;
            }
        }
    }
    throw Exception(name_, "Vertex has no interface neighbor with given representative");
    return false;
}

bool ompl::geometric::SPARS::addPairToSpanner( Vertex q, Vertex qp )
{
    Node v = representativesProperty_[q];
    Node vp = representativesProperty_[qp];
    
    //Error checking
    if( v == vp )
    {
        //Something went wrong here, we should report an error
        throw Exception(name_, "Pairs report having the same representative");
        return false;
    }
    
    //Try the straight-line path
    PathGeometric *p = new PathGeometric(si_);        
    p->append( sparseStateProperty_[v] );
    p->append( sparseStateProperty_[vp] );
    //If it is valid
    if( p->check() )
    {
        //simply connnect the reps with an edge.
        connectSparsePoints( v, vp );

        delete p;
        resetFailures();
        //Return success
        return true;
    }
    delete p;
    
    //Generate the midpoint: m(q,qp)
    base::State* st = generateMidpoint( stateProperty_[q], stateProperty_[qp] );
    //Create the path through the midpoint
    PathGeometric *pp = new PathGeometric(si_);
    pp->append( sparseStateProperty_[v] );
    pp->append( st );
    pp->append( sparseStateProperty_[vp] );
    //If that path is good
    if( pp->check() )
    {
        //Generate the midpoint connections
        Node mp = addGuard( si_->cloneState(st), 2 );
        connectSparsePoints( v, mp );
        connectSparsePoints( mp, vp );
        si_->freeState( st );
        delete pp;
        //Return success
        return true;
    }
    delete pp;
    
    //Link everything up
    Node ga = addGuard( si_->cloneState(stateProperty_[q]), 2 );
    Node gb = addGuard( si_->cloneState(stateProperty_[qp]), 2 );
    connectSparsePoints( v, ga );
    connectSparsePoints( ga, gb );
    connectSparsePoints( gb, vp );
    //return success
    return true;
}

bool ompl::geometric::SPARS::addPathToSpanner( PathGeometric* dense_path, Node vp, Node vpp )
{
    //Attempt to simplify the path
    psimp_->reduceVertices( *dense_path, 50 );
    //Once we have our final path, add it
    PathGeometric *p = dense_path;
    //First, check to see that the path has length
    if( p->length() == 0 )
    {   
        //The path was simplified away, so simply link the representatives
        connectSparsePoints( vp, vpp );
        resetFailures();
    }
    else
    {
        //Otherwise, do this thing iteratively
        std::vector< Node > added_nodes;
        for( unsigned int i = 0; i<p->getStateCount(); ++i )
        {
            //Add each guard
            Node ng = addGuard( si_->cloneState(p->getState(i)), 3 );
            added_nodes.push_back( ng );
        }
        //Link them up
        for( unsigned int i=0; i<added_nodes.size()-1; ++i )
        {
            connectSparsePoints( added_nodes[i], added_nodes[i+1] );
        }
        //Don't forget to link them to their representatives
        connectSparsePoints( added_nodes[0], vp );
        connectSparsePoints( added_nodes[added_nodes.size()-1], vpp );
    }
    return true;
}

void ompl::geometric::SPARS::updateReps( Node v )
{
    //Get all of the dense samples which may be affected by adding this node
    std::vector< Vertex > dense_points;
    
    stateProperty_[ queryVertex_ ] = sparseStateProperty_[ v ];
    
    nn_->nearestR( queryVertex_, sparseDelta_ + denseDelta_, dense_points );
    
    stateProperty_[ queryVertex_ ] = NULL;
    
    //For each of those points
    for( unsigned int i=0; i<dense_points.size(); ++i )
    {
        //Remove that point from the old representative's list(s)
        removeFromRep( dense_points[i], representativesProperty_[dense_points[i]] );
        //Update that point's representative
        calculateRepresentative( dense_points[i] );
    }
    
    //For each of the points
    for( unsigned int i=0; i<dense_points.size(); ++i )
    {
        //Get it's representative
        Node rep = representativesProperty_[dense_points[i]];
        //Extract the representatives of any interface-sharing neighbors
        std::vector< Node > oreps = getInterfaceNeighborRepresentatives( dense_points[i] );
        //For sanity's sake, make sure we clear ourselves out of what this new rep might think of us
        removeFromRep( dense_points[i], rep );
        //Add this vertex to it's representative's list for the other representatives
        addToRep( dense_points[i], rep, oreps );
    }
}

void ompl::geometric::SPARS::calculateRepresentative( Vertex q )
{
    //Get the nearest neighbors within sparseDelta_
    std::vector<Node> neigh = getSparseNeighbors( stateProperty_[q] );
    
    bool abort = false;
    //For each neighbor
    for( unsigned int i=0; i<neigh.size() && !abort; ++i )
    {
        PathGeometric *p = new PathGeometric(si_);
        //If the path to this neighbor is collision free
        p->append( stateProperty_[q] );
        p->append( sparseStateProperty_[neigh[i]] );
        if( p->check() )
        {
            //update the representative
            representativesProperty_[q] = neigh[i];
            //abort
            abort = true;
        }
        delete p;
    }
}

void ompl::geometric::SPARS::addToRep( Vertex q, Node rep, std::vector<ompl::geometric::SPARS::Node> oreps )
{
    //If this node supports no interfaces
    if( oreps.size() == 0 )
    {
        //Add it to the pool of non-interface nodes
        if( std::find( nonInterfaceListsProperty_[rep].begin(), nonInterfaceListsProperty_[rep].end(), q ) == nonInterfaceListsProperty_[rep].end() )
        {
            nonInterfaceListsProperty_[rep].push_back( q );
        }
        else
        {
            throw Exception(name_, "Node already tracked in non-interface list");
        }
    }
    else
    {
        //otherwise, for every neighbor representative
        foreach( Node v, oreps )
        {
            if( std::find( interfaceListsProperty_[rep][v].begin(), interfaceListsProperty_[rep][v].end(), q ) != interfaceListsProperty_[rep][v].end() )
            {
                throw Exception(name_, "Node already in interface list");
            }
            else
            {
                if( rep != representativesProperty_[q] )
                {
                    throw Exception(name_, "Node has representative different than the list he's being put into.");
                    exit( -1 );
                }
                //Add this node to the list for that representative
                interfaceListsProperty_[rep][v].push_back( q );
            }
        }
    }
}

void ompl::geometric::SPARS::removeFromRep( Vertex q, Node rep )
{
    // Remove the node from the non-interface points (if there)
    nonInterfaceListsProperty_[rep].remove( q );
    if( std::find( nonInterfaceListsProperty_[rep].begin(), nonInterfaceListsProperty_[rep].end(), q ) != nonInterfaceListsProperty_[rep].end() )
    {
        throw Exception(name_, "The point could not be removed?");
    }
    // From each of the interfaces
    foreach( Node vpp, interfaceListsProperty_[rep] | boost::adaptors::map_keys )
    {
        // Remove this node from that list
        interfaceListsProperty_[rep][vpp].remove( q );
        if( std::find( interfaceListsProperty_[rep][vpp].begin(), interfaceListsProperty_[rep][vpp].end(), q ) != interfaceListsProperty_[rep][vpp].end() )
        {
            throw Exception(name_, "Point is some how impossible to remove?");
        }
    }
}

std::vector< ompl::geometric::SPARS::Node > ompl::geometric::SPARS::computeVPP( Node v, Node vp )
{
    std::vector< Node > ret;
    
    foreach( Node cvpp, boost::adjacent_vertices( v, s_ ) )
    {
        if( cvpp != vp )
        {
            if( !boost::edge( cvpp, vp, s_ ).second )
            {
                ret.push_back( cvpp );
            }
        }
    }
    
    return ret;
}

std::vector< ompl::geometric::SPARS::Node > ompl::geometric::SPARS::computeX( Node v, Node vp, Node vpp )
{
    std::vector< Node > ret;
    
    foreach( Node cx, boost::adjacent_vertices( vpp, s_ ) )
    {
        if( boost::edge( cx, v, s_ ).second && !boost::edge( cx, vp, s_ ).second )
        {
            if( interfaceListsProperty_[vpp][cx].size() > 0 )
            {
                ret.push_back( cx );
            }
        }
    }
    ret.push_back( vpp );
    
    return ret;
}

ompl::base::State* ompl::geometric::SPARS::generateMidpoint( ompl::base::State* a, ompl::base::State* b )
{
    base::State* st = si_->allocState();
    si_->getStateSpace()->interpolate( a, b, 0.5, st );
    return st;
}

std::vector<ompl::geometric::SPARS::Vertex> ompl::geometric::SPARS::interfaceCheck(ompl::geometric::SPARS::Vertex q)
{
    std::vector< Vertex > ret;
    //Get our representative
    Node rep = representativesProperty_[q];
    //For each neighbor
    foreach( Vertex n, boost::adjacent_vertices( q, g_ ) )
    {
        //If his representative is not our own
        if( rep != representativesProperty_[n] )
        {
            //If he is within denseDelta_
            if( distanceFunction( q, n ) < denseDelta_ )
            {
                //Push him onto the list
                ret.push_back( n );
            }
        }
    }
    return ret;
}

std::vector< ompl::geometric::SPARS::Node > ompl::geometric::SPARS::getInterfaceNeighborRepresentatives( ompl::geometric::SPARS::Vertex q )
{
    std::vector< Node > ret;
    
    //Get our representative
    Node rep = representativesProperty_[q];
    //For each neighbor we are connected to
    foreach( Vertex n, boost::adjacent_vertices( q, g_ ) )
    {
        //Get his representative
        Node orep = representativesProperty_[n];
        //If that representative is not our own
        if( orep != rep )
        {
            //If he is within denseDelta_
            if( distanceFunction( q, n ) < denseDelta_ )
            {
                //And we haven't tracked him yet
                if( std::find( ret.begin(), ret.end(), orep ) == ret.end() )
                {
                    //Append his rep to the list
                    ret.push_back( orep );
                }
            }
        }
    }
    
    return ret;
}

std::vector< ompl::geometric::SPARS::Vertex > ompl::geometric::SPARS::getInterfaceNeighborhood( ompl::geometric::SPARS::Vertex q )
{
    std::vector< Vertex > ret;
    
    //Get our representative
    Node rep = representativesProperty_[q];
    //For each neighbor we are connected to
    foreach( Vertex n, boost::adjacent_vertices( q, g_ ) )
    {
        //Get his representative
        Node orep = representativesProperty_[n];
        //If that representative is not our own
        if( orep != rep )
        {
            //If he is within denseDelta_
            if( distanceFunction( q, n ) < denseDelta_ )
            {
                //Append him to the list
                ret.push_back( n );
            }
        }
    }
    
    return ret;    
}

ompl::base::PathPtr ompl::geometric::SPARS::constructSolution(const Node start, const Node goal) const
{
    PathGeometric *p = new PathGeometric(si_);

    boost::vector_property_map<Node> prev(boost::num_vertices(s_));

    boost::astar_search(s_, start,
            boost::bind(&SPARS::sparseDistanceFunction, this, _1, goal),
            boost::predecessor_map(prev));

    if (prev[goal] == goal)
        throw Exception(name_, "Could not find solution path");
    else
        for (Node pos = goal; prev[pos] != pos; pos = prev[pos])
            p->append(sparseStateProperty_[pos]);
    p->append(sparseStateProperty_[start]);
    p->reverse();

    return base::PathPtr(p);
}

ompl::geometric::PathGeometric* ompl::geometric::SPARS::densePath( const Vertex start, const Vertex goal ) const
{
    PathGeometric *p = new PathGeometric(si_);
    
    boost::vector_property_map<Vertex> prev(boost::num_vertices(g_));
    
    boost::astar_search(g_, start,
                        boost::bind(&SPARS::distanceFunction, this, _1, goal),
                        boost::predecessor_map(prev));
    
    if (prev[goal] == goal)
    {
        OMPL_WARN("No dense path was found?");
        delete p;
    }
    else
    {
        for (Vertex pos = goal; prev[pos] != pos; pos = prev[pos])
            p->append(stateProperty_[pos]);
        p->append(stateProperty_[start]);
        p->reverse();
    }
    return p;
}

void ompl::geometric::SPARS::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    // Explicitly add start and goal states:
    for (size_t i = 0; i < startM_.size(); ++i)
        data.addStartVertex(base::PlannerDataVertex(sparseStateProperty_[startM_[i]], 4));

    for (size_t i = 0; i < goalM_.size(); ++i)
        data.addGoalVertex(base::PlannerDataVertex(sparseStateProperty_[goalM_[i]], 4));

    // Adding edges and all other vertices simultaneously
    foreach(const Link e, boost::edges(s_))
    {
        const Node v1 = boost::source(e, s_);
        const Node v2 = boost::target(e, s_);
        data.addEdge(base::PlannerDataVertex(sparseStateProperty_[v1], sparseColorProperty_[v1]),
                     base::PlannerDataVertex(sparseStateProperty_[v2], sparseColorProperty_[v2]));

        // Add the reverse edge, since we're constructing an undirected roadmap
        data.addEdge(base::PlannerDataVertex(sparseStateProperty_[v2], sparseColorProperty_[v1]),
                     base::PlannerDataVertex(sparseStateProperty_[v1], sparseColorProperty_[v2]));
    }
    
    // Make sure to add edge-less nodes as well
    foreach(const Node n, boost::vertices(s_))
    {
        if( boost::out_degree( n, s_ ) == 0 )
        {
            data.addVertex( base::PlannerDataVertex(sparseStateProperty_[n], sparseColorProperty_[n] ) );
        }
    }
}


