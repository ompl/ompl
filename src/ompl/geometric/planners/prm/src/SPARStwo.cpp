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

#include "ompl/geometric/planners/prm/SPARStwo.h"
#include "ompl/geometric/planners/prm/ConnectionStrategy.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include "ompl/tools/config/SelfConfig.h"
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
            motion generated as part of the expansion step of SPARStwo. */
        static const unsigned int MAX_RANDOM_BOUNCE_STEPS   = 5;

        /** \brief The number of nearest neighbors to consider by
            default in the construction of the SPARStwo roadmap */
        static const unsigned int DEFAULT_NEAREST_NEIGHBORS = 10;

        /** \brief The time in seconds for a single roadmap building operation (dt)*/
        static const double ROADMAP_BUILD_TIME = 0.2;
    }
}

ompl::geometric::SPARStwo::SPARStwo(const base::SpaceInformationPtr &si) :
    base::Planner(si, "SPARStwo"),
    t_(3),
    sparseDelta_(15),
    denseDelta_(0.5),
    m_(5000),
    nearSamplePoints_((2*si_->getStateDimension())),
    stateProperty_(boost::get(vertex_state_t(), g_)),
    totalConnectionAttemptsProperty_(boost::get(vertex_total_connection_attempts_t(), g_)),
    successfulConnectionAttemptsProperty_(boost::get(vertex_successful_connection_attempts_t(), g_)),
    weightProperty_(boost::get(boost::edge_weight, g_)),
    edgeIDProperty_(boost::get(boost::edge_index, g_)),
    colorProperty_(boost::get(vertex_color_t(), g_)),
    interfaceDataProperty_(boost::get(vertex_interface_data_t(), g_)),
    disjointSets_(boost::get(boost::vertex_rank, g_),
                  boost::get(boost::vertex_predecessor, g_)),
    maxEdgeID_(0),
    userSetConnectionStrategy_(false),
    addedSolution_(false),
    iterations_(0)
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;

    psimp_.reset(new PathSimplifier(si_));

    qNew_ = NULL;
    holdState_ = NULL;
    simpleSampler_ = si_->allocStateSampler();

    Planner::declareParam<double>("stretch_factor", this, &SPARStwo::setStretchFactor, &SPARStwo::getStretchFactor, "1.1:0.1:3.0");
    Planner::declareParam<double>("sparse_delta", this, &SPARStwo::setSparseDelta, &SPARStwo::getSparseDelta, "1.0:0.5:30.0");
    Planner::declareParam<double>("dense_delta", this, &SPARStwo::setDenseDelta, &SPARStwo::getDenseDelta, "0.02:0.02:1.0");
    Planner::declareParam<unsigned int>("max_failures", this, &SPARStwo::setMaxFailures, &SPARStwo::getMaxFailures, "100:10:3000");
}

ompl::geometric::SPARStwo::~SPARStwo(void)
{
    freeMemory();
    si_->freeState( qNew_ );
    si_->freeState( holdState_ );
}

void ompl::geometric::SPARStwo::setup(void)
{
    Planner::setup();
    if (!nn_)
        nn_.reset(new NearestNeighborsGNAT<Vertex>());
    nn_->setDistanceFunction(boost::bind(&SPARStwo::distanceFunction, this, _1, _2));
    if (!connectionStrategy_)
    {
        connectionStrategy_ = KStarStrategy<Vertex>(boost::bind(&SPARStwo::milestoneCount, this), nn_, si_->getStateDimension());
    }
    if (!connectionFilter_)
        connectionFilter_ = boost::lambda::constant(true);
}

void ompl::geometric::SPARStwo::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
    Planner::setProblemDefinition(pdef);
    clearQuery();
}

void ompl::geometric::SPARStwo::clearQuery(void)
{
    startM_.clear();
    goalM_.clear();
    pis_.restart();
}

void ompl::geometric::SPARStwo::clear(void)
{
    Planner::clear();
    clearQuery();
    iterations_ = 0;
    freeMemory();
    if (nn_)
        nn_->clear();
}

void ompl::geometric::SPARStwo::freeMemory(void)
{
    Planner::clear();
    sampler_.reset();
    simpleSampler_.reset();

    foreach (Vertex v, boost::vertices(g_))
    {
        foreach( interfaceData d, interfaceDataProperty_[v] | boost::adaptors::map_values )
        {
            clearInterfaceData( d, si_ );
        }
        if( stateProperty_[v] != NULL )
        {
            si_->freeState(stateProperty_[v]);
        }
        stateProperty_[v] = NULL;
    }
    g_.clear();
    
    if (nn_)
        nn_->clear();
    maxEdgeID_ = 0;    
}

ompl::base::State* ompl::geometric::SPARStwo::sample( void )
{
    if (!isSetup())
        setup();
    if (!simpleSampler_)
        simpleSampler_ = si_->allocStateSampler();
    if( !sampler_ )
        sampler_ = si_->allocValidStateSampler();
    
    sampler_->sample( qNew_ );

    return qNew_;
}

void ompl::geometric::SPARStwo::checkForSolution (const base::PlannerTerminationCondition &ptc,
                                             base::PathPtr &solution)
{
    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

    // Check for any new goal states
    if (goal->maxSampleCount() > goalM_.size())
    {
        const base::State *st = pis_.nextGoal();
        if (st)
            goalM_.push_back(addGuard(si_->cloneState(st),6));
    }

    // Check for a solution
    addedSolution_ = haveSolution (startM_, goalM_, solution);
}

bool ompl::geometric::SPARStwo::haveSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals, base::PathPtr &solution)
{
    base::Goal *g = pdef_->getGoal().get();
    double sl = -1.0; // cache for solution length
    foreach (Vertex start, starts)
    {
        foreach (Vertex goal, goals)
        {
            if (boost::same_component(start, goal, disjointSets_) &&
                g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
            {
                solution = constructSolution(start, goal);
                return true;
            }
        }
    }

    return false;
}

bool ompl::geometric::SPARStwo::addedNewSolution (void) const
{
    return addedSolution_;
}

bool ompl::geometric::SPARStwo::reachedFailureLimit (void) const
{
    return iterations_ >= m_;
}

ompl::base::PlannerStatus ompl::geometric::SPARStwo::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    
    if( boost::num_vertices( g_ ) < 1 )
    {
        queryVertex_ = boost::add_vertex( g_ );
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
        startM_.push_back(addGuard(si_->cloneState(st),6));

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
            goalM_.push_back(addGuard(si_->cloneState(st),6));

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
    base::PlannerOrTerminationCondition ptcOrFail( ptc, base::PlannerTerminationCondition( boost::bind( &SPARStwo::reachedFailureLimit, this ) ) );
    
    if( qNew_ == NULL )
        qNew_ = si_->allocState();
    if( holdState_ == NULL )
        holdState_ = si_->allocState();

    bool doFourth = false;

    while ( !ptcOrFail() )
    {
        //Increment iterations
        ++iterations_;
        
        //Generate a single sample, and attempt to connect it to nearest neighbors.
        sample();
                
        findGraphNeighbors( qNew_ );
        
        if( !checkAddCoverage() )
            if( !checkAddConnectivity() )
                if( !checkAddInterface() )
                {
                    std::pair< std::vector< Vertex >, std::vector< base::State* > > RS = findCloseRepresentatives();
                    for( unsigned int i=0; i<RS.first.size(); ++i )
                    {
                        updatePairPoints( repV_, qNew_, RS.first[i], RS.second[i] );                
                        updatePairPoints( RS.first[i], RS.second[i], repV_, qNew_ );
                    }
                    checkAddPath( repV_ );
                    for( unsigned int i=0; i<RS.first.size(); ++i )
                    {
                        checkAddPath( RS.first[i] );
                    }
                }
    }

    checkForSolution(ptc, sol);

    OMPL_INFORM("Created %u states", boost::num_vertices(g_) - nrStartStates);

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

ompl::base::PlannerStatus ompl::geometric::SPARStwo::solve(const base::PlannerTerminationCondition &ptc, unsigned int maxFail )
{
    m_ = maxFail;
    return solve( ptc );
}

bool ompl::geometric::SPARStwo::checkAddCoverage( void )
{
    if( visibleNeighborhood_.size() > 0 )
        return false;
    //No free paths means we add for coverage
    addGuard( si_->cloneState( qNew_ ), 0 );
    return true;
}

bool ompl::geometric::SPARStwo::checkAddConnectivity( void )
{
    std::vector< Vertex > links;
    if( visibleNeighborhood_.size() > 1 )
    {
        //For each neighbor
        for( unsigned int i=0; i<visibleNeighborhood_.size(); ++i )
        {
            //For each other neighbor
            for( unsigned int j=i+1; j<visibleNeighborhood_.size(); ++j )
            {
                //If they are in different components
                if( !boost::same_component( visibleNeighborhood_[i], visibleNeighborhood_[j], disjointSets_ ) )
                {
                    links.push_back( visibleNeighborhood_[i] );
                    links.push_back( visibleNeighborhood_[j] );
                }
            }
        }
        
        if( links.size() != 0 )
        {
            //Add the node
            Vertex g = addGuard( si_->cloneState( qNew_ ), 1 );
            
            for( unsigned int i=0; i<links.size(); ++i )
            {
                //If there's no edge
                if( !boost::edge(g, links[i], g_).second )
                {
                    //And the components haven't been united by previous links
                    if( !boost::same_component( links[i], g, disjointSets_ ) )
                    {
                        connect( g, links[i] );
                    }
                }
            }
            return true;
        }
    }
    return false;
}

bool ompl::geometric::SPARStwo::checkAddInterface( void )
{
    //If we have more than 1 or 0 neighbors
    if( visibleNeighborhood_.size() > 1 )
    {
        if( graphNeighborhood_[0] == visibleNeighborhood_[0] && graphNeighborhood_[1] == visibleNeighborhood_[1] )
        {
            //If our two closest neighbors don't share an edge
            if( !boost::edge( visibleNeighborhood_[0], visibleNeighborhood_[1], g_ ).second )
            {
                //If they can be directly connected
                if( si_->checkMotion( stateProperty_[visibleNeighborhood_[0]], stateProperty_[visibleNeighborhood_[1]] ) )
                {
                    //Connect them
                    connect( visibleNeighborhood_[0], visibleNeighborhood_[1] );
                    //And report that we added to the roadmap
                    resetFailures();
                    //Report success
                    return true;
                }
                else
                {
                    //Add the new node to the graph, to bridge the interface
                    Vertex v = addGuard( si_->cloneState( qNew_ ), 2 );
                    connect( v, visibleNeighborhood_[0] );
                    connect( v, visibleNeighborhood_[1] );
                    //Report success
                    return true;
                }
            }
        }
    }
    return false;
}

bool ompl::geometric::SPARStwo::checkAddPath( Vertex v )
{
    bool ret = false;
    
    std::vector< Vertex > rs;
    foreach( Vertex r, boost::adjacent_vertices( v, g_ ) )
    {
        rs.push_back(r);
    }
    for( unsigned i=0; i<rs.size() && !ret; ++i )
    {
        Vertex r = rs[i];
        std::vector< Vertex > rps = computeVPP( v, r );
        foreach( Vertex rp, rps )
        {
            //First, compute the longest path through the graph
            std::vector< Vertex > rpps = computeX( v, r, rp );
            double rm_dist = 0;
            foreach( Vertex rpp, rpps )
            {
                double tmp_dist = (si_->distance( stateProperty_[r], stateProperty_[v] ) + si_->distance( stateProperty_[v], stateProperty_[rpp] ) )/2.0;
                if( tmp_dist > rm_dist )
                {
                    rm_dist = tmp_dist;
                }
            }
            
            interfaceData& d = getData( v, r, rp );
            
            //Then, if the spanner property is violated
            if( rm_dist > t_ * d.d_ )
            {
                ret = true; //Report that we added for the path
                if( si_->checkMotion( stateProperty_[r], stateProperty_[rp] ) )
                {
                    connect( r, rp );
                }
                else
                {
                    PathGeometric* p = new PathGeometric( si_ );
                    if( r < rp )
                    {
                        p->append( si_->cloneState(d.sigmas_.first.get()) );
                        p->append( si_->cloneState(d.points_.first.get()) );
                        p->append( si_->cloneState(stateProperty_[v]) );
                        p->append( si_->cloneState(d.points_.second.get()) );
                        p->append( si_->cloneState(d.sigmas_.second.get()) );
                    }
                    else
                    {
                        p->append( si_->cloneState(d.sigmas_.second.get()) );
                        p->append( si_->cloneState(d.points_.second.get()) );
                        p->append( si_->cloneState(stateProperty_[v]) );
                        p->append( si_->cloneState(d.points_.first.get()) );
                        p->append( si_->cloneState(d.sigmas_.first.get()) );
                    }
                    
                    psimp_->shortcutPath( *p, 50 );
                    psimp_->reduceVertices( *p, 50 );
                    
                    p->checkAndRepair( 100 );
                    
                    Vertex prior = r;
                    Vertex vnew;
                    std::vector<base::State*> states = p->getStates();
                    
                    foreach( base::State* st, states )
                    {
                        vnew = addGuard( si_->cloneState( st ), 3 );
                        
                        connect( prior, vnew );
                        prior = vnew;
                        si_->freeState( st );
                    }
                    connect( prior, rp );
                }
            }
        }
    }
    
    return ret;
}

void ompl::geometric::SPARStwo::resetFailures( void )
{
    iterations_ = 0;
}

void ompl::geometric::SPARStwo::findGraphNeighbors( base::State* st )
{
    visibleNeighborhood_.clear();

    stateProperty_[ queryVertex_ ] = st;
    
    nn_->nearestR( queryVertex_, sparseDelta_, graphNeighborhood_ );
    
    stateProperty_[ queryVertex_ ] = NULL;
    
    //Now that we got the neighbors from the NN, we must remove any we can't see
    for( unsigned int i=0; i<graphNeighborhood_.size(); ++i )
    {
        if( si_->checkMotion( st, stateProperty_[graphNeighborhood_[i]] ) )
        {
            visibleNeighborhood_.push_back( graphNeighborhood_[i] );
        }
    }
}

std::vector< ompl::geometric::SPARStwo::Vertex > ompl::geometric::SPARStwo::findTwoClosest( base::State* st )
{
    stateProperty_[ queryVertex_ ] = st;
    
    std::vector< Vertex > hold;
    nn_->nearestK( queryVertex_, 2, hold );
    
    stateProperty_[queryVertex_] = NULL;

    return hold;
}

void ompl::geometric::SPARStwo::approachGraph( Vertex v )
{
    std::vector< Vertex > hold;
    nn_->nearestR( v, sparseDelta_, hold );
    
    std::vector< Vertex > neigh;
    for( size_t i=0; i<hold.size(); ++i )
    {
        if( si_->checkMotion( stateProperty_[v], stateProperty_[hold[i]] ) )
        {
            neigh.push_back( hold[i] );
        }
    }

    foreach( Vertex vp, neigh )
    {
        connect( v, vp );
    }
}

void ompl::geometric::SPARStwo::findGraphRepresentative( base::State* st )
{
    stateProperty_[ queryVertex_ ] = st;

    nn_->nearestR( queryVertex_, sparseDelta_, graphNeighborhood_ );
    
    stateProperty_[queryVertex_] = NULL;
    
    visibleNeighborhood_.clear();
    for( size_t i=0; i<graphNeighborhood_.size() && visibleNeighborhood_.size() == 0; ++i )
    {
        if( si_->checkMotion( st, stateProperty_[graphNeighborhood_[i]] ) )
        {
            visibleNeighborhood_.push_back( graphNeighborhood_[i] );
        }
    }
}

std::pair< std::vector< ompl::geometric::SPARStwo::Vertex >, std::vector< ompl::base::State* > > ompl::geometric::SPARStwo::findCloseRepresentatives( void )
{
    std::pair< std::vector< Vertex >, std::vector< base::State* > > ret;
    //First, remember who represents qNew_
    repV_ = visibleNeighborhood_[0];
    
    bool abort = false;
    //Then, begin searching the space around him
    for( unsigned int i=0; i<nearSamplePoints_ && !abort; ++i )
    {
        bool done = true;
        do
        {
            done = true;
            sampler_->sampleNear( holdState_, qNew_, denseDelta_ );
            if( !si_->isValid( holdState_) )
            {
                done = false;
            }
            if( si_->distance( qNew_, holdState_ ) > denseDelta_ )
            {
                done = false;
            }
            else if( !si_->checkMotion( qNew_, holdState_ ) )
            {
                done = false;
            }
        } while( !done );
        //Compute who his graph neighbors are
        findGraphRepresentative( holdState_ );
        //Assuming this sample is actually seen by somebody (which he should be in all likelihood) 
        if( visibleNeighborhood_.size() > 0 )
        {
            //If his representative is different than qNew_
            if( repV_ != visibleNeighborhood_[0] )
            {
                //And we haven't already tracked this representative
                if( std::find( ret.first.begin(), ret.first.end(), visibleNeighborhood_[0] ) == ret.first.end() )
                {
                    //Track the representative
                    ret.first.push_back( visibleNeighborhood_[0] );
                    //Also remember who generated him
                    ret.second.push_back( si_->cloneState( holdState_ ) );
                }
            }
        }
        else
        {
            //This guy can't be seen by anybody, so we should take this opportunity to add him
            addGuard( si_->cloneState( holdState_ ), 4 );
            //We should also stop our efforts to add a dense path
            ret.first.clear();
            for( unsigned int i=0; i<ret.second.size(); ++i )
            {
                si_->freeState( ret.second[i] );
            }
            ret.second.clear();
            abort = true;
        }
    }
    return ret;
}

void ompl::geometric::SPARStwo::updatePairPoints( Vertex rep, const safeState& q, Vertex r, const safeState& s )
{
    //First of all, we need to compute all candidate r'
    std::vector< Vertex > rps = computeVPP( rep, r );
    //Then, for each pair Pv(r,r')
    foreach( Vertex rp, rps )
    {
        //Try updating the pair info
        distanceCheck( rep, q, r, s, rp );
    }
}

std::vector< ompl::geometric::SPARStwo::Vertex > ompl::geometric::SPARStwo::computeVPP( Vertex v, Vertex vp )
{
    std::vector< Vertex > ret;
    
    foreach( Vertex cvpp, boost::adjacent_vertices( v, g_ ) )
    {
        if( cvpp != vp )
        {
            if( !boost::edge( cvpp, vp, g_ ).second )
            {
                ret.push_back( cvpp );
            }
        }
    }
    
    return ret;
}

std::vector< ompl::geometric::SPARStwo::Vertex > ompl::geometric::SPARStwo::computeX( Vertex v, Vertex vp, Vertex vpp )
{
    std::vector< Vertex > ret;
    
    foreach( Vertex cx, boost::adjacent_vertices( vpp, g_ ) )
    {
        if( boost::edge( cx, v, g_ ).second && !boost::edge( cx, vp, g_ ).second )
        {
            interfaceData& d = getData( v, vpp, cx );
            if( vpp < cx && d.points_.first.get() != NULL )
            {
                ret.push_back( cx );
            }
            else if( cx < vpp && d.points_.second.get() != NULL )
            {
                ret.push_back( cx );
            }
        }
    }
    ret.push_back( vpp );
    
    return ret;
}

ompl::geometric::SPARStwo::vertexPair ompl::geometric::SPARStwo::index( Vertex vp, Vertex vpp )
{
    if( vp < vpp )
    {
        return vertexPair( vp, vpp );
    }
    else if( vpp < vp )
    {
        return vertexPair( vpp, vp );
    }
    else
    {
        throw Exception( name_, "Trying to get an index where the pairs are the same point!");
    }
}

ompl::geometric::SPARStwo::interfaceData& ompl::geometric::SPARStwo::getData( Vertex v, Vertex vp, Vertex vpp )
{
    vertexPair r = index( vp, vpp );
    interfaceData& d = interfaceDataProperty_[v][r];
    return d;
}

void ompl::geometric::SPARStwo::setData( Vertex v, Vertex vp, Vertex vpp, interfaceData d )
{
    vertexPair r = index( vp, vpp );
    interfaceDataProperty_[v][r] = d;
}

void ompl::geometric::SPARStwo::distanceCheck( Vertex rep, const safeState& q, Vertex r, const safeState& s, Vertex rp )
{
    //Get the info for the current representative-neighbors pair
    interfaceData& d = getData( rep, r, rp );
    
    if( r < rp ) // FIRST points represent r (the guy discovered through sampling)
    {
        if( d.points_.first.get() == NULL ) // If the point we're considering replacing (P_v(r,.)) isn't there
        {
            //Then we know we're doing better, so add it
            d.set_first( q, s, si_ );
        }
        else //Otherwise, he is there,
        {
            if( d.points_.second.get() == NULL ) //But if the other guy doesn't exist, we can't compare.
            {
                //Should probably keep the one that is further away from rep?  Not known what to do in this case.
            }
            else //We know both of these points exist, so we can check some distances
            {
                if( si_->distance( q.get(), d.points_.second.get() ) < si_->distance( d.points_.first.get(), d.points_.second.get() ) )
                {
                    //Distance with the new point is good, so set it.
                    d.set_first( q, s, si_ );
                }
            }
        }
    }
    else // SECOND points represent r (the guy discovered through sampling)
    {
        if( d.points_.second.get() == NULL ) //If the point we're considering replacing (P_V(.,r)) isn't there...
        {
            //Then we must be doing better, so add it
            d.set_second( q, s, si_ );
        }
        else //Otherwise, he is there
        {
            if( d.points_.first.get() == NULL ) //But if the other guy doesn't exist, we can't compare.
            {
                //Should we be doing something cool here?
            }
            else
            {
                if( si_->distance( q.get(), d.points_.first.get() ) < si_->distance( d.points_.second.get(), d.points_.first.get() ) )
                {
                    //Distance with the new point is good, so set it
                    d.set_second( q, s, si_ );
                }
            }
        }
    }
    //Lastly, save what we have discovered
    setData( rep, r, rp, d );
}

void ompl::geometric::SPARStwo::abandonLists( base::State* st )
{
    stateProperty_[ queryVertex_ ] = st;
    
    std::vector< Vertex > hold;
    nn_->nearestR( queryVertex_, sparseDelta_, hold );
    
    stateProperty_[queryVertex_] = NULL;

    //For each of the vertices
    foreach( Vertex v, hold )
    {
        deletePairInfo( v );
    }
}

void ompl::geometric::SPARStwo::deletePairInfo( Vertex v )
{
    foreach( vertexPair r, interfaceDataProperty_[v] | boost::adaptors::map_keys )
    {
        interfaceData& d = interfaceDataProperty_[v][r];
        clearInterfaceData( d, si_ );
    }
}

ompl::geometric::SPARStwo::Vertex ompl::geometric::SPARStwo::addGuard(base::State *state, unsigned int c)
{
    Vertex m = boost::add_vertex(g_);
    stateProperty_[m] = state;
    colorProperty_[m] = c;

    if( !si_->isValid( state ) )
    {
        throw Exception( name_, "Attempting to promote a guard which is an invalid state!");
    }

    abandonLists( state );

    disjointSets_.make_set(m);
    nn_->add(m);
    resetFailures();
    
    return m;
}

void ompl::geometric::SPARStwo::connect( Vertex v, Vertex vp )
{
    if( v > milestoneCount() )
        OMPL_ERROR("\'From\' Vertex out of range : %u\n", v );
    if( vp > milestoneCount() )
        OMPL_ERROR("\'To\' Vertex out of range : %u\n", vp );
    
    const double weight = distanceFunction(v, vp);
    const unsigned int id = maxEdgeID_++;
    const Graph::edge_property_type properties(weight, id);
    boost::add_edge(v, vp, properties, g_);
    uniteComponents( v, vp );
}

void ompl::geometric::SPARStwo::uniteComponents(Vertex m1, Vertex m2)
{
    disjointSets_.union_set(m1, m2);
}

ompl::base::PathPtr ompl::geometric::SPARStwo::constructSolution(const Vertex start, const Vertex goal) const
{
    PathGeometric *p = new PathGeometric(si_);

    boost::vector_property_map<Vertex> prev(boost::num_vertices(g_));

    boost::astar_search(g_, start,
            boost::bind(&SPARStwo::distanceFunction, this, _1, goal),
            boost::predecessor_map(prev));

    if (prev[goal] == goal)
        throw Exception(name_, "Could not find solution path");
    else
        for (Vertex pos = goal; prev[pos] != pos; pos = prev[pos])
            p->append(stateProperty_[pos]);
    p->append(stateProperty_[start]);
    p->reverse();

    return base::PathPtr(p);
}

void ompl::geometric::SPARStwo::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    // Explicitly add start and goal states:
    for (size_t i = 0; i < startM_.size(); ++i)
        data.addStartVertex(base::PlannerDataVertex(stateProperty_[startM_[i]]));

    for (size_t i = 0; i < goalM_.size(); ++i)
        data.addGoalVertex(base::PlannerDataVertex(stateProperty_[goalM_[i]]));

    // If there are even edges here
    if( boost::num_edges( g_ ) > 0 )
    {
        // Adding edges and all other vertices simultaneously
        foreach(const Edge e, boost::edges(g_))
        {
            const Vertex v1 = boost::source(e, g_);
            const Vertex v2 = boost::target(e, g_);
            unsigned long size = boost::num_vertices( g_ );
            if( v1 < size || v2 < size )
            {
                data.addEdge(base::PlannerDataVertex(stateProperty_[v1], colorProperty_[v1]),
                             base::PlannerDataVertex(stateProperty_[v2], colorProperty_[v2]));
                
                // Add the reverse edge, since we're constructing an undirected roadmap
                data.addEdge(base::PlannerDataVertex(stateProperty_[v2], colorProperty_[v2]),
                             base::PlannerDataVertex(stateProperty_[v1], colorProperty_[v1]));
            }
            else
            {
                OMPL_ERROR("Edge Vertex Error: [%lu][%lu] > %lu\n", v1, v2, size);
            }
        }
    }
    else
    {
        OMPL_INFORM("There are no edges in the graph!\n");
    }
    
    // Make sure to add edge-less nodes as well
    foreach(const Vertex n, boost::vertices(g_))
    {
        if( boost::out_degree( n, g_ ) == 0 )
        {
            data.addVertex( base::PlannerDataVertex(stateProperty_[n], colorProperty_[n] ) );
        }
    }
}



