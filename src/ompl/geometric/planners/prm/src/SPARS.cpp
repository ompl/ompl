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

#include "ompl/geometric/planners/prm/SPARS.h"
#include "ompl/geometric/planners/prm/ConnectionStrategy.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include <boost/bind.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH
#define foreach_reverse BOOST_REVERSE_FOREACH

ompl::geometric::SPARS::SPARS(const base::SpaceInformationPtr &si) :
    base::Planner(si, "SPARS"),
    geomPath_(si),
    stateProperty_(boost::get(vertex_state_t(), g_)),
    sparseStateProperty_(boost::get(vertex_state_t(), s_)),
    sparseColorProperty_(boost::get(vertex_color_t(), s_)),
    representativesProperty_(boost::get(vertex_representative_t(), g_)),
    nonInterfaceListsProperty_(boost::get(vertex_list_t(), s_)),
    interfaceListsProperty_(boost::get(vertex_interface_list_t(), s_)),
    weightProperty_(boost::get(boost::edge_weight, g_)),
    sparseDJSets_(boost::get(boost::vertex_rank, s_),
                  boost::get(boost::vertex_predecessor, s_)),
    iterations_(0),
    stretchFactor_(3.),
    maxFailures_(1000),
    denseDeltaFraction_(.001),
    sparseDeltaFraction_(.25),
    lastState_(NULL),
    denseDelta_(0.),
    sparseDelta_(0.)
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = false;
    specs_.optimizingPaths = true;

    psimp_.reset(new PathSimplifier(si_));
    psimp_->freeStates(false);

    Planner::declareParam<double>("stretch_factor", this, &SPARS::setStretchFactor, &SPARS::getStretchFactor, "1.1:0.1:3.0");
    Planner::declareParam<double>("sparse_delta_fraction", this, &SPARS::setSparseDeltaFraction, &SPARS::getSparseDeltaFraction, "0.0:0.01:1.0");
    Planner::declareParam<double>("dense_delta_fraction", this, &SPARS::setDenseDeltaFraction, &SPARS::getDenseDeltaFraction, "0.0:0.0001:0.1");
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
        nn_.reset(new NearestNeighborsGNAT<DenseVertex>());
    nn_->setDistanceFunction(boost::bind(&SPARS::distanceFunction, this, _1, _2));
    if (!snn_)
        snn_.reset(new NearestNeighborsGNAT<SparseVertex>());
    snn_->setDistanceFunction(boost::bind(&SPARS::sparseDistanceFunction, this, _1, _2));
    if (!connectionStrategy_)
        connectionStrategy_ = KStarStrategy<DenseVertex>(boost::bind(&SPARS::milestoneCount, this), nn_, si_->getStateDimension());
    double maxExt = si_->getMaximumExtent();
    sparseDelta_ = sparseDeltaFraction_ * maxExt;
    denseDelta_ = denseDeltaFraction_ * maxExt;
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
    iterations_ = 0;
    lastState_ = NULL;

    Xs_.clear();
    VPPs_.clear();
    graphNeighborhood_.clear();
    visibleNeighborhood_.clear();
    interfaceNeighborhood_.clear();
    interfaceRepresentatives_.clear();
}

void ompl::geometric::SPARS::freeMemory(void)
{
    foreach (DenseVertex v, boost::vertices(g_))
        if( stateProperty_[v] != NULL )
        {
            si_->freeState(stateProperty_[v]);
            stateProperty_[v] = NULL;
        }
    foreach (SparseVertex n, boost::vertices(s_))
        if( sparseStateProperty_[n] != NULL )
        {
            si_->freeState(sparseStateProperty_[n]);
            sparseStateProperty_[n] = NULL;
        }
    s_.clear();
    g_.clear();

    if( lastState_ != NULL )
        si_->freeState( lastState_ );
    path_.clear();
}

ompl::geometric::SPARS::DenseVertex ompl::geometric::SPARS::addSample( void )
{
    return addSample( lastState_ );
}

ompl::geometric::SPARS::DenseVertex ompl::geometric::SPARS::addSample(base::State *workState)
{
    bool found = false;
    DenseVertex ret = boost::graph_traits<DenseGraph>::null_vertex();
    do
        found = sampler_->sample(workState);
    while (!found);
    if (found)
        ret = addMilestone(si_->cloneState(workState));
    return ret;
}

bool ompl::geometric::SPARS::haveSolution(const std::vector<DenseVertex> &starts, const std::vector<DenseVertex> &goals, base::PathPtr &solution)
{
    base::Goal *g = pdef_->getGoal().get();
    foreach (DenseVertex start, starts)
    {
        foreach (DenseVertex goal, goals)
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

bool ompl::geometric::SPARS::reachedFailureLimit (void) const
{
    return iterations_ >= maxFailures_;
}

ompl::base::PlannerStatus ompl::geometric::SPARS::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();

    lastState_ = si_->allocState();

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
        startM_.push_back(addGuard(si_->cloneState(st), START ));
    }
    if (startM_.empty())
    {
        OMPL_ERROR("There are no valid initial states!");
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("Insufficient states in sampleable goal region");
        return base::PlannerStatus::INVALID_GOAL;
    }

    // Add the valid goal states as milestones
    while (const base::State *st = pis_.nextGoal())
    {
        addMilestone(si_->cloneState(st));
        goalM_.push_back(addGuard(si_->cloneState(st), GOAL ));
    }
    if (goalM_.empty())
    {
        OMPL_ERROR("Unable to find any valid goal states");
        return base::PlannerStatus::INVALID_GOAL;
    }

    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();
    if (!simpleSampler_)
        simpleSampler_ = si_->allocStateSampler();

    unsigned int nrStartStates = boost::num_vertices(g_) - 1; // don't count query vertex
    OMPL_INFORM("Starting with %u states", nrStartStates);

    // Reset addedSolution_ member
    base::PathPtr sol;
    sol.reset();

    //Construct planner termination condition which also takes M into account
    base::PlannerOrTerminationCondition ptcOrFail(ptc,base::PlannerTerminationCondition(boost::bind(&SPARS::reachedFailureLimit, this)));

    while ( !ptcOrFail() )
    {
        //Generate a single sample, and attempt to connect it to nearest neighbors.
        DenseVertex q = addSample();

        //Now that we've added to D, try adding to S
        //Start by figuring out who our neighbors are
        getSparseNeighbors( lastState_ );
        getVisibleNeighbors( lastState_ );
        //Check for addition for Coverage
        if( !checkAddCoverage( graphNeighborhood_ ) )
            //If not for Coverage, then Connectivity
            if( !checkAddConnectivity( graphNeighborhood_ ) )
                //Check for the existence of an interface
                if( !checkAddInterface( graphNeighborhood_, visibleNeighborhood_, q ) )
                    //Then check to see if it's on an interface
                    getInterfaceNeighborhood( q );
                    if( interfaceNeighborhood_.size() > 0 )
                    {
                        //Check for addition for spanner prop
                        if( !checkAddPath( q, interfaceNeighborhood_ ) )
                            //All of the tests have failed.  Report failure for the sample
                            ++iterations_;
                    }
                    else
                        //There's no interface here, so drop it
                        ++iterations_;
    }

    haveSolution( startM_, goalM_, sol );

    if (sol)
        pdef_->addSolutionPath (sol, false);

    // Return true if any solution was found.
    return sol ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

ompl::base::PlannerStatus ompl::geometric::SPARS::solve(const base::PlannerTerminationCondition &ptc, unsigned int maxFail)
{
    maxFailures_ = maxFail;
    return solve( ptc );
}

ompl::geometric::SPARS::DenseVertex ompl::geometric::SPARS::addMilestone(base::State *state)
{
    DenseVertex m = boost::add_vertex(g_);
    stateProperty_[m] = state;

    // Which milestones will we attempt to connect to?
    const std::vector<DenseVertex>& neighbors = connectionStrategy_(m);

    foreach (DenseVertex n, neighbors)
        if (si_->checkMotion(stateProperty_[m], stateProperty_[n]))
        {
            const double weight = distanceFunction(m, n);
            const DenseGraph::edge_property_type properties(weight);

            boost::add_edge(m, n, properties, g_);
        }

    nn_->add(m);

    //Need to update representative information here...
    calculateRepresentative(m);

    getInterfaceNeighborRepresentatives(m);
    getInterfaceNeighborhood(m);
    addToRep( m, representativesProperty_[m], interfaceRepresentatives_ );
    foreach( DenseVertex qp, interfaceNeighborhood_ )
    {
        removeFromRep( qp, representativesProperty_[qp] );
        getInterfaceNeighborRepresentatives( qp );
        addToRep( qp, representativesProperty_[qp], interfaceRepresentatives_ );
    }

    return m;
}

ompl::geometric::SPARS::SparseVertex ompl::geometric::SPARS::addGuard(base::State *state, GuardType type)
{
    SparseVertex v = boost::add_vertex( s_ );
    sparseStateProperty_[v] = state;
    sparseColorProperty_[v] = type;

    sparseDJSets_.make_set(v);

    snn_->add(v);
    updateReps( v );

    resetFailures();
    return v;
}

void ompl::geometric::SPARS::uniteSparseComponents(SparseVertex m1, SparseVertex m2)
{
    sparseDJSets_.union_set(m1, m2);
}

void ompl::geometric::SPARS::connectSparsePoints( SparseVertex v, SparseVertex vp )
{
    const double weight = sparseDistanceFunction(v, vp);
    const SpannerGraph::edge_property_type properties(weight);
    boost::add_edge(v, vp, properties, s_);
    uniteSparseComponents( v, vp );
}

void ompl::geometric::SPARS::connectDensePoints( DenseVertex v, DenseVertex vp )
{
    const double weight = distanceFunction(v, vp);
    const DenseGraph::edge_property_type properties( weight );
    boost::add_edge(v, vp, properties, g_);
}

bool ompl::geometric::SPARS::checkAddCoverage( const std::vector<SparseVertex>& neigh )
{
    //For each of these neighbors,
    foreach( SparseVertex n, neigh )
        //If path between is free
        if( si_->checkMotion( lastState_, sparseStateProperty_[n] ) )
            //Abort out and return false
            return false;
    //No free paths means we add for coverage
    addGuard( si_->cloneState(lastState_), COVERAGE );
    return true;
}

bool ompl::geometric::SPARS::checkAddConnectivity( const std::vector<SparseVertex>& neigh )
{
    std::vector< SparseVertex > links;
    //For each neighbor
    for (std::size_t i = 0; i < neigh.size(); ++i )
        //For each other neighbor
        for (std::size_t j = i + 1; j < neigh.size(); ++j )
            //If they are in different components
            if( !boost::same_component( neigh[i], neigh[j], sparseDJSets_ ) )
                //If the paths between are collision free
                if( si_->checkMotion( lastState_, sparseStateProperty_[neigh[i]] ) && si_->checkMotion( lastState_, sparseStateProperty_[neigh[j]] ) )
                {
                    links.push_back( neigh[i] );
                    links.push_back( neigh[j] );
                }

    if( links.size() != 0 )
    {
        //Add the node
        SparseVertex g = addGuard( si_->cloneState(lastState_), CONNECTIVITY );

        for (std::size_t i = 0; i < links.size(); ++i )
            //If there's no edge
            if (!boost::edge(g, links[i], s_).second)
                //And the components haven't been united by previous links
                if (!boost::same_component( links[i], g, sparseDJSets_))
                    connectSparsePoints( g, links[i] );
        return true;
    }
    return false;
}

bool ompl::geometric::SPARS::checkAddInterface( const std::vector<ompl::geometric::SPARS::SparseVertex>& graphNeighborhood_, const std::vector<ompl::geometric::SPARS::SparseVertex>& visibleNeighborhood_, ompl::geometric::SPARS::DenseVertex q )
{
    //If we have more than 1 neighbor
    if( visibleNeighborhood_.size() > 1 )
        //If our closest neighbors are also visible
        if( graphNeighborhood_[0] == visibleNeighborhood_[0] && graphNeighborhood_[1] == visibleNeighborhood_[1] )
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
                    SparseVertex v = addGuard( si_->cloneState( stateProperty_[q] ), INTERFACE );
                    connectSparsePoints( v, visibleNeighborhood_[0] );
                    connectSparsePoints( v, visibleNeighborhood_[1] );
                    //Report success
                    return true;
                }
            }
    return false;
}

bool ompl::geometric::SPARS::checkAddPath(ompl::geometric::SPARS::DenseVertex q, const std::vector<ompl::geometric::SPARS::DenseVertex>& neigh )
{
    bool ret = false;
    //Get q's representative => v
    SparseVertex v = representativesProperty_[q];
    //Extract the representatives of neigh => n_rep
    std::vector< SparseVertex > n_rep;

    foreach( DenseVertex qp, neigh )
        //If we haven't tracked this representative
        if( std::find( n_rep.begin(), n_rep.end(), representativesProperty_[qp] ) == n_rep.end() )
            n_rep.push_back( representativesProperty_[qp] );

    //for each v' in n_rep
    for (std::size_t i = 0; i < n_rep.size() && !ret; ++i )
    {
        SparseVertex vp = n_rep[i];
        //Identify appropriate v" candidates => vpps
        computeVPP( v, vp );

        foreach( SparseVertex vpp, VPPs_ )
        {
            double s_max = 0;
            //Find the X nodes to test
            computeX( v, vp, vpp );
            //For each x in xs
            foreach( SparseVertex x, Xs_ )
            {
                //Compute/Retain MAXimum distance path thorugh S
                double dist = (si_->distance(sparseStateProperty_[x], sparseStateProperty_[v])
                    + si_->distance(sparseStateProperty_[v], sparseStateProperty_[vp])) / 2.0;
                if( dist > s_max )
                    s_max = dist;
            }

            std::deque< base::State* > bestDPath;
            DenseVertex best_qpp = boost::graph_traits<DenseGraph>::null_vertex();
            double d_min = std::numeric_limits<double>::infinity(); //Insanely big number
            //For each vpp in vpps
            for (std::size_t j = 0; j < VPPs_.size() && !ret; ++j)
            {
                SparseVertex vpp = VPPs_[j];
                //For each q", which are stored interface nodes on v for i(vpp,v)
                foreach( DenseVertex qpp, interfaceListsProperty_[v][vpp] )
                {
                    if( representativesProperty_[qpp] != v )
                        throw Exception(name_, "Representative mismatch!");
                    else
                    {
                        //If they happen to be the one and same node
                        if( q == qpp )
                        {
                            bestDPath.push_front( stateProperty_[q] );
                            best_qpp = qpp;
                            d_min = 0;
                        }
                        else
                        {
                            //Compute/Retain MINimum distance path on D through q, q"
                            densePath( q, qpp );
                            if( path_.size() != 0 )
                            {
                                double length = pathLength();
                                if( length < d_min )
                                {
                                    d_min = length;
                                    bestDPath = path_;
                                    best_qpp = qpp;
                                }
                            }
                        }
                    }
                }

                //If the spanner property is violated for these paths
                if( s_max > stretchFactor_* d_min )
                {
                    //Need to augment this path with the appropriate neighbor information
                    DenseVertex na = getInterfaceNeighbor(q, vp);
                    DenseVertex nb = getInterfaceNeighbor(best_qpp, vpp);

                    bestDPath.push_front( stateProperty_[na] );
                    bestDPath.push_back( stateProperty_[nb] );

                    if( representativesProperty_[na] != vp || representativesProperty_[nb] != vpp )
                        throw Exception(name_, "Inappropriate representatives found.");
                    //Add the dense path to the spanner
                    addPathToSpanner( bestDPath, vpp, vp );

                    //Report success
                    ret = true;
                }
            }
        }
    }
    return ret;
}

double ompl::geometric::SPARS::avgValence(void) const
{
    double degree = 0.0;
    foreach (DenseVertex v, boost::vertices(s_))
        degree += (double)boost::out_degree(v, s_);
    degree /= (double)boost::num_vertices(s_);
    return degree;
}

void ompl::geometric::SPARS::resetFailures(void)
{
    iterations_ = 0;
}

void ompl::geometric::SPARS::approachGraph(DenseVertex v)
{
    std::vector<DenseVertex> hold;
    nn_->nearestR( v, 3.0 * denseDelta_, hold );
    // \todo what is 3  here? A magic constant?

    std::vector< DenseVertex > neigh;
    for (std::size_t i = 0; i < hold.size() ; ++i)
        if (si_->checkMotion(stateProperty_[v], stateProperty_[hold[i]]))
            neigh.push_back(hold[i]);

    foreach (DenseVertex vp, neigh)
        connectDensePoints(v, vp);
}

void ompl::geometric::SPARS::approachSpanner( SparseVertex n )
{
    std::vector< SparseVertex > hold;
    snn_->nearestR( n, sparseDelta_, hold );

    std::vector< SparseVertex > neigh;
    for (std::size_t i = 0; i < hold.size(); ++i)
        if( si_->checkMotion( sparseStateProperty_[n], sparseStateProperty_[hold[i]] ) )
            neigh.push_back( hold[i] );

    foreach( SparseVertex np, neigh )
        connectSparsePoints( n, np );
}

void ompl::geometric::SPARS::getSparseNeighbors( base::State* inState )
{
    sparseStateProperty_[sparseQueryVertex_] = inState;

    graphNeighborhood_.clear();
    snn_->nearestR( sparseQueryVertex_, sparseDelta_, graphNeighborhood_ );

    sparseStateProperty_[sparseQueryVertex_] = NULL;
}

void ompl::geometric::SPARS::getVisibleNeighbors( base::State* inState )
{
    sparseStateProperty_[sparseQueryVertex_] = inState;

    std::vector< SparseVertex > hold;
    snn_->nearestR( sparseQueryVertex_, sparseDelta_, hold );

    sparseStateProperty_[sparseQueryVertex_] = NULL;

    visibleNeighborhood_.clear();
    //Now that we got the neighbors from the NN, we must remove any we can't see
    for (std::size_t i = 0; i < hold.size(); ++i)
        if( si_->checkMotion( inState, sparseStateProperty_[hold[i]] ) )
            visibleNeighborhood_.push_back( hold[i] );
}

ompl::geometric::SPARS::DenseVertex ompl::geometric::SPARS::getInterfaceNeighbor(DenseVertex q, SparseVertex rep)
{
    foreach (DenseVertex vp, boost::adjacent_vertices( q, g_ ))
        if (representativesProperty_[vp] == rep )
            if (distanceFunction( q, vp ) <= denseDelta_)
                return vp;
    throw Exception(name_, "Vertex has no interface neighbor with given representative");
}

bool ompl::geometric::SPARS::addPathToSpanner( const std::deque< base::State* >& dense_path, SparseVertex vp, SparseVertex vpp )
{
    // First, check to see that the path has length
    if (dense_path.size() <= 1)
    {
        // The path is 0 length, so simply link the representatives
        connectSparsePoints( vp, vpp );
        resetFailures();
    }
    else
    {
        //We will need to construct a PathGeometric to do this.
        geomPath_.getStates().resize( dense_path.size() );
        std::copy( dense_path.begin(), dense_path.end(), geomPath_.getStates().begin() );

        //Attempt to simplify the path
        psimp_->reduceVertices( geomPath_, geomPath_.getStateCount() * 2);

        // we are sure there are at least 2 points left on geomPath_

        std::vector< SparseVertex > added_nodes;
        added_nodes.reserve(geomPath_.getStateCount());
        for (std::size_t i = 0; i < geomPath_.getStateCount(); ++i )
        {
            //Add each guard
            SparseVertex ng = addGuard( si_->cloneState(geomPath_.getState(i)), QUALITY );
            added_nodes.push_back( ng );
        }
        //Link them up
        for (std::size_t i = 1; i < added_nodes.size() ; ++i )
        {
            connectSparsePoints(added_nodes[i - 1], added_nodes[i]);
        }
        //Don't forget to link them to their representatives
        connectSparsePoints( added_nodes[0], vp );
        connectSparsePoints( added_nodes[added_nodes.size()-1], vpp );
    }
    geomPath_.getStates().clear();
    return true;
}

void ompl::geometric::SPARS::updateReps( SparseVertex v )
{
    //Get all of the dense samples which may be affected by adding this node
    std::vector< DenseVertex > dense_points;

    stateProperty_[ queryVertex_ ] = sparseStateProperty_[ v ];

    nn_->nearestR( queryVertex_, sparseDelta_ + denseDelta_, dense_points );

    stateProperty_[ queryVertex_ ] = NULL;

    //For each of those points
    for (std::size_t i = 0 ; i < dense_points.size() ; ++i)
    {
        //Remove that point from the old representative's list(s)
        removeFromRep( dense_points[i], representativesProperty_[dense_points[i]] );
        //Update that point's representative
        calculateRepresentative( dense_points[i] );
    }

    //For each of the points
    for (std::size_t i = 0 ; i < dense_points.size(); ++i)
    {
        //Get it's representative
        SparseVertex rep = representativesProperty_[dense_points[i]];
        //Extract the representatives of any interface-sharing neighbors
        getInterfaceNeighborRepresentatives( dense_points[i] );
        //For sanity's sake, make sure we clear ourselves out of what this new rep might think of us
        removeFromRep( dense_points[i], rep );
        //Add this vertex to it's representative's list for the other representatives
        addToRep( dense_points[i], rep, interfaceRepresentatives_ );
    }
}

void ompl::geometric::SPARS::calculateRepresentative( DenseVertex q )
{
    //Get the nearest neighbors within sparseDelta_
    getSparseNeighbors( stateProperty_[q] );

    //For each neighbor
    for (std::size_t i = 0; i < graphNeighborhood_.size(); ++i)
        if( si_->checkMotion( stateProperty_[q], sparseStateProperty_[graphNeighborhood_[i]] ) )
        {
            //update the representative
            representativesProperty_[q] = graphNeighborhood_[i];
            //abort
            break;
        }
}

void ompl::geometric::SPARS::addToRep( DenseVertex q, SparseVertex rep, const std::vector<ompl::geometric::SPARS::SparseVertex>& oreps )
{
    //If this node supports no interfaces
    if( oreps.size() == 0 )
    {
        //Add it to the pool of non-interface nodes
        if( std::find( nonInterfaceListsProperty_[rep].begin(), nonInterfaceListsProperty_[rep].end(), q ) == nonInterfaceListsProperty_[rep].end() )
            nonInterfaceListsProperty_[rep].push_back( q );
        else
            throw Exception(name_, "Node already tracked in non-interface list");
    }
    else
    {
        //otherwise, for every neighbor representative
        foreach( SparseVertex v, oreps )
        {
            if( std::find( interfaceListsProperty_[rep][v].begin(), interfaceListsProperty_[rep][v].end(), q ) != interfaceListsProperty_[rep][v].end() )
                throw Exception(name_, "Node already in interface list");
            else
            {
                if( rep != representativesProperty_[q] )
                    throw Exception(name_, "Node has representative different than the list he's being put into.");
                //Add this node to the list for that representative
                interfaceListsProperty_[rep][v].push_back( q );
            }
        }
    }
}

void ompl::geometric::SPARS::removeFromRep( DenseVertex q, SparseVertex rep )
{
    // Remove the node from the non-interface points (if there)
    nonInterfaceListsProperty_[rep].remove( q );
    if( std::find( nonInterfaceListsProperty_[rep].begin(), nonInterfaceListsProperty_[rep].end(), q ) != nonInterfaceListsProperty_[rep].end() )
        throw Exception(name_, "The point could not be removed?");
    // From each of the interfaces
    foreach( SparseVertex vpp, interfaceListsProperty_[rep] | boost::adaptors::map_keys )
    {
        // Remove this node from that list
        interfaceListsProperty_[rep][vpp].remove( q );
        if( std::find( interfaceListsProperty_[rep][vpp].begin(), interfaceListsProperty_[rep][vpp].end(), q ) != interfaceListsProperty_[rep][vpp].end() )
            throw Exception(name_, "Point is some how impossible to remove?");
    }
}

void ompl::geometric::SPARS::computeVPP( SparseVertex v, SparseVertex vp )
{
    VPPs_.clear();

    foreach( SparseVertex cvpp, boost::adjacent_vertices( v, s_ ) )
        if( cvpp != vp )
            if( !boost::edge( cvpp, vp, s_ ).second )
                VPPs_.push_back( cvpp );
}

void ompl::geometric::SPARS::computeX( SparseVertex v, SparseVertex vp, SparseVertex vpp )
{
    Xs_.clear();

    foreach( SparseVertex cx, boost::adjacent_vertices( vpp, s_ ) )
        if( boost::edge( cx, v, s_ ).second && !boost::edge( cx, vp, s_ ).second )
            if( interfaceListsProperty_[vpp][cx].size() > 0 )
                Xs_.push_back( cx );
    Xs_.push_back( vpp );
}

ompl::base::State* ompl::geometric::SPARS::generateMidpoint( const ompl::base::State* a, const ompl::base::State* b ) const
{
    base::State* st = si_->allocState();
    si_->getStateSpace()->interpolate( a, b, 0.5, st );
    return st;
}

void ompl::geometric::SPARS::getInterfaceNeighborRepresentatives( ompl::geometric::SPARS::DenseVertex q )
{
    interfaceRepresentatives_.clear();
    // Get our representative
    SparseVertex rep = representativesProperty_[q];
    // For each neighbor we are connected to
    foreach( DenseVertex n, boost::adjacent_vertices( q, g_ ) )
    {
        // Get his representative
        SparseVertex orep = representativesProperty_[n];
        // If that representative is not our own
        if( orep != rep )
            // If he is within denseDelta_
            if( distanceFunction( q, n ) < denseDelta_ )
                // And we haven't tracked him yet
                if( std::find( interfaceRepresentatives_.begin(), interfaceRepresentatives_.end(), orep )
                    == interfaceRepresentatives_.end() )
                    // Append his rep to the list
                    interfaceRepresentatives_.push_back( orep );
    }
}

void ompl::geometric::SPARS::getInterfaceNeighborhood( ompl::geometric::SPARS::DenseVertex q )
{
    interfaceNeighborhood_.clear();
    // Get our representative
    SparseVertex rep = representativesProperty_[q];
    // For each neighbor we are connected to
    foreach( DenseVertex n, boost::adjacent_vertices( q, g_ ) )
        // If neighbor representative is not our own
        if( representativesProperty_[n] != rep )
            // If he is within denseDelta_
            if( distanceFunction( q, n ) < denseDelta_ )
                // Append him to the list
                interfaceNeighborhood_.push_back( n );
}

ompl::base::PathPtr ompl::geometric::SPARS::constructSolution(const SparseVertex start, const SparseVertex goal) const
{
    PathGeometric *p = new PathGeometric(si_);

    boost::vector_property_map<SparseVertex> prev(boost::num_vertices(s_));

    boost::astar_search(s_, start,
            boost::bind(&SPARS::sparseDistanceFunction, this, _1, goal),
            boost::predecessor_map(prev));

    if (prev[goal] == goal)
        throw Exception(name_, "Could not find solution path");
    else
        for (SparseVertex pos = goal; prev[pos] != pos; pos = prev[pos])
            p->append(sparseStateProperty_[pos]);
    p->append(sparseStateProperty_[start]);
    p->reverse();

    return base::PathPtr(p);
}

void ompl::geometric::SPARS::densePath( const DenseVertex start, const DenseVertex goal ) const
{
    path_.clear();

    boost::vector_property_map<DenseVertex> prev(boost::num_vertices(g_));

    boost::astar_search(g_, start,
                        boost::bind(&SPARS::distanceFunction, this, _1, goal),
                        boost::predecessor_map(prev));

    if (prev[goal] == goal)
        OMPL_WARN("No dense path was found?");
    else
    {
        for (DenseVertex pos = goal; prev[pos] != pos; pos = prev[pos])
            path_.push_front( stateProperty_[pos] );
        path_.push_front( stateProperty_[start] );
    }
}

double ompl::geometric::SPARS::pathLength() const
{
    double l = 0.0;
    for (std::size_t i = 1; i < path_.size(); ++i)
        l += si_->distance(path_[i-1], path_[i]);
    return l;
}

void ompl::geometric::SPARS::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    // Explicitly add start and goal states:
    for (std::size_t i = 0; i < startM_.size(); ++i)
        data.addStartVertex(base::PlannerDataVertex(sparseStateProperty_[startM_[i]]));

    for (std::size_t i = 0; i < goalM_.size(); ++i)
        data.addGoalVertex(base::PlannerDataVertex(sparseStateProperty_[goalM_[i]]));

    // Adding edges and all other vertices simultaneously
    foreach (const SparseEdge e, boost::edges(s_))
    {
        const SparseVertex v1 = boost::source(e, s_);
        const SparseVertex v2 = boost::target(e, s_);
        data.addEdge(base::PlannerDataVertex(sparseStateProperty_[v1], sparseColorProperty_[v1]),
                     base::PlannerDataVertex(sparseStateProperty_[v2], sparseColorProperty_[v2]));

        // Add the reverse edge, since we're constructing an undirected roadmap
        data.addEdge(base::PlannerDataVertex(sparseStateProperty_[v2], sparseColorProperty_[v1]),
                     base::PlannerDataVertex(sparseStateProperty_[v1], sparseColorProperty_[v2]));
    }

    // Make sure to add edge-less nodes as well
    foreach (const SparseVertex n, boost::vertices(s_))
        if (boost::out_degree( n, s_ ) == 0)
            data.addVertex( base::PlannerDataVertex(sparseStateProperty_[n], sparseColorProperty_[n]));
}
