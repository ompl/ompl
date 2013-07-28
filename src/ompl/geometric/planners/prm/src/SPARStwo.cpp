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

#include "ompl/geometric/planners/prm/SPARStwo.h"
#include "ompl/geometric/planners/prm/ConnectionStrategy.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <boost/lambda/bind.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>

#include "GoalVisitor.hpp"

#define foreach BOOST_FOREACH
#define foreach_reverse BOOST_REVERSE_FOREACH

ompl::geometric::SPARStwo::SPARStwo(const base::SpaceInformationPtr &si) :
    base::Planner(si, "SPARStwo"),
    stretchFactor_(3.),
    sparseDeltaFraction_(.25),
    denseDeltaFraction_(.001),
    maxFailures_(5000),
    nearSamplePoints_((2*si_->getStateDimension())),
    stateProperty_(boost::get(vertex_state_t(), g_)),
    weightProperty_(boost::get(boost::edge_weight, g_)),
    colorProperty_(boost::get(vertex_color_t(), g_)),
    interfaceDataProperty_(boost::get(vertex_interface_data_t(), g_)),
    disjointSets_(boost::get(boost::vertex_rank, g_),
                  boost::get(boost::vertex_predecessor, g_)),
    addedSolution_(false),
    iterations_(0),
    sparseDelta_(0.),
    denseDelta_(0.)
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = false;
    specs_.optimizingPaths = true;

    psimp_.reset(new PathSimplifier(si_));

    Planner::declareParam<double>("stretch_factor", this, &SPARStwo::setStretchFactor, &SPARStwo::getStretchFactor, "1.1:0.1:3.0");
    Planner::declareParam<double>("sparse_delta_fraction", this, &SPARStwo::setSparseDeltaFraction, &SPARStwo::getSparseDeltaFraction, "0.0:0.01:1.0");
    Planner::declareParam<double>("dense_delta_fraction", this, &SPARStwo::setDenseDeltaFraction, &SPARStwo::getDenseDeltaFraction, "0.0:0.0001:0.1");
    Planner::declareParam<unsigned int>("max_failures", this, &SPARStwo::setMaxFailures, &SPARStwo::getMaxFailures, "100:10:3000");
}

ompl::geometric::SPARStwo::~SPARStwo(void)
{
    freeMemory();
}

void ompl::geometric::SPARStwo::setup(void)
{
    Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(si_->getStateSpace()));
    nn_->setDistanceFunction(boost::bind(&SPARStwo::distanceFunction, this, _1, _2));
    double maxExt = si_->getMaximumExtent();
    sparseDelta_ = sparseDeltaFraction_ * maxExt;
    denseDelta_ = denseDeltaFraction_ * maxExt;
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
        foreach (InterfaceData &d, interfaceDataProperty_[v] | boost::adaptors::map_values)
            d.clear(si_);
        if( stateProperty_[v] != NULL )
            si_->freeState(stateProperty_[v]);
        stateProperty_[v] = NULL;
    }
    g_.clear();

    if (nn_)
        nn_->clear();
}

bool ompl::geometric::SPARStwo::haveSolution(const std::vector<Vertex> &starts, const std::vector<Vertex> &goals, base::PathPtr &solution)
{
    base::Goal *g = pdef_->getGoal().get();
    foreach (Vertex start, starts)
        foreach (Vertex goal, goals)
        {
	    // we lock because the connected components algorithm is incremental and may change disjointSets_
            graphMutex_.lock();
            bool same_component = sameComponent(start, goal);
            graphMutex_.unlock();

            if (same_component && g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
            {
                solution = constructSolution(start, goal);
                return true;
            }
	}
    return false;
}

bool ompl::geometric::SPARStwo::sameComponent(Vertex m1, Vertex m2)
{
    return boost::same_component(m1, m2, disjointSets_);
}

bool ompl::geometric::SPARStwo::reachedFailureLimit(void) const
{
    return iterations_ >= maxFailures_;
}

bool ompl::geometric::SPARStwo::reachedTerminationCriterion(void) const
{
    return iterations_ >= maxFailures_ || addedSolution_;
}

void ompl::geometric::SPARStwo::constructRoadmap(const base::PlannerTerminationCondition &ptc, bool stopOnMaxFail)
{
    if (stopOnMaxFail)
    {
        base::PlannerOrTerminationCondition ptcOrFail(ptc, base::PlannerTerminationCondition(boost::bind(&SPARStwo::reachedFailureLimit, this)));
        constructRoadmap(ptcOrFail);
    }
    else
        constructRoadmap(ptc);
}

void ompl::geometric::SPARStwo::constructRoadmap(const base::PlannerTerminationCondition &ptc)
{
    checkQueryStateInitialization();

    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();
    if (!simpleSampler_)
        simpleSampler_ = si_->allocStateSampler();

    base::State *qNew = si_->allocState();
    base::State *workState = si_->allocState();

    /* The whole neighborhood set which has been most recently computed */
    std::vector<Vertex> graphNeighborhood;
    /* The visible neighborhood set which has been most recently computed */
    std::vector<Vertex> visibleNeighborhood;

    while (ptc == false)
    {
        //Increment iterations
        ++iterations_;

        //Generate a single sample, and attempt to connect it to nearest neighbors.
        sampler_->sample(qNew);

        findGraphNeighbors(qNew, graphNeighborhood, visibleNeighborhood);

        if (!checkAddCoverage(qNew, visibleNeighborhood))
            if (!checkAddConnectivity(qNew, visibleNeighborhood))
                if (!checkAddInterface(qNew, graphNeighborhood, visibleNeighborhood))
                {
                    if (visibleNeighborhood.size() > 0)
                    {
                        std::map<Vertex, base::State*> closeRepresentatives;
                        findCloseRepresentatives(workState, qNew, visibleNeighborhood[0], closeRepresentatives, ptc);
                        for (std::map<Vertex, base::State*>::iterator it = closeRepresentatives.begin(); it != closeRepresentatives.end(); ++it)
                        {
                            updatePairPoints(visibleNeighborhood[0], qNew, it->first, it->second);
                            updatePairPoints(it->first, it->second, visibleNeighborhood[0], qNew);
                        }
                        checkAddPath(visibleNeighborhood[0]);
                        for (std::map<Vertex, base::State*>::iterator it = closeRepresentatives.begin(); it != closeRepresentatives.end(); ++it)
			{
			    checkAddPath(it->first);
			    si_->freeState(it->second);
			}
		    }
                }
    }
    si_->freeState(workState);
}

void ompl::geometric::SPARStwo::checkQueryStateInitialization(void)
{
    if (boost::num_vertices( g_ ) < 1)
    {
        queryVertex_ = boost::add_vertex( g_ );
        stateProperty_[queryVertex_] = NULL;
    }
}

ompl::base::PlannerStatus ompl::geometric::SPARStwo::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    checkQueryStateInitialization();

    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

    if (!goal)
    {
        OMPL_ERROR("Goal undefined or unknown type of goal");
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    // Add the valid start states as milestones
    while (const base::State *st = pis_.nextStart())
        startM_.push_back(addGuard(si_->cloneState(st), START));
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
    while (const base::State *st = (goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal()))
        goalM_.push_back(addGuard(si_->cloneState(st), GOAL));
    if (goalM_.empty())
    {
        OMPL_ERROR("Unable to find any valid goal states");
        return base::PlannerStatus::INVALID_GOAL;
    }

    unsigned int nrStartStates = boost::num_vertices(g_) - 1;  // don't count query vertex
    OMPL_INFORM("Starting with %u states", nrStartStates);

    // Reset addedSolution_ member
    addedSolution_ = false;
    base::PathPtr sol;
    base::PlannerOrTerminationCondition ptcOrFail(ptc, base::PlannerTerminationCondition(boost::bind(&SPARStwo::reachedFailureLimit, this)));
    boost::thread slnThread(boost::bind(&SPARStwo::checkForSolution, this, ptcOrFail, boost::ref(sol)));

    //Construct planner termination condition which also takes M into account
    base::PlannerOrTerminationCondition ptcOrStop(ptc, base::PlannerTerminationCondition(boost::bind(&SPARStwo::reachedTerminationCriterion, this)));
    constructRoadmap(ptcOrStop);

    // Ensure slnThread is ceased before exiting solve
    slnThread.join();

    OMPL_INFORM("Created %u states", boost::num_vertices(g_) - nrStartStates);

    if (sol)
        pdef_->addSolutionPath(sol, false);

    // Return true if any solution was found.
    return sol ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::SPARStwo::checkForSolution(const base::PlannerTerminationCondition &ptc, base::PathPtr &solution)
{
    base::GoalSampleableRegion *goal = static_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());
    while (!ptc && !addedSolution_)
    {
        // Check for any new goal states
        if (goal->maxSampleCount() > goalM_.size())
        {
            const base::State *st = pis_.nextGoal();
            if (st)
		goalM_.push_back(addGuard(si_->cloneState(st), GOAL));
        }

        // Check for a solution
        addedSolution_ = haveSolution(startM_, goalM_, solution);
        // Sleep for 1ms
        if (!addedSolution_)
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    }
}

bool ompl::geometric::SPARStwo::checkAddCoverage(const base::State *qNew, std::vector<Vertex> &visibleNeighborhood)
{
    if (visibleNeighborhood.size() > 0)
        return false;
    //No free paths means we add for coverage
    addGuard(si_->cloneState(qNew), COVERAGE);
    return true;
}

bool ompl::geometric::SPARStwo::checkAddConnectivity(const base::State *qNew, std::vector<Vertex> &visibleNeighborhood)
{
    std::vector<Vertex> links;
    if (visibleNeighborhood.size() > 1)
    {
        //For each neighbor
        for (std::size_t i = 0; i < visibleNeighborhood.size(); ++i)
            //For each other neighbor
            for (std::size_t j = i + 1; j < visibleNeighborhood.size(); ++j)
                //If they are in different components
                if (!sameComponent(visibleNeighborhood[i], visibleNeighborhood[j]))
                {
                    links.push_back(visibleNeighborhood[i]);
                    links.push_back(visibleNeighborhood[j]);
                }

        if (links.size() > 0)
        {
            //Add the node
            Vertex g = addGuard(si_->cloneState(qNew), CONNECTIVITY);

            for (std::size_t i = 0; i < links.size() ; ++i)
                //If there's no edge
                if (!boost::edge(g, links[i], g_).second)
                    //And the components haven't been united by previous links
                    if (!sameComponent(links[i], g))
                        connect(g, links[i]);
            return true;
        }
    }
    return false;
}

bool ompl::geometric::SPARStwo::checkAddInterface(const base::State *qNew, std::vector<Vertex> &graphNeighborhood, std::vector<Vertex> &visibleNeighborhood)
{
    //If we have more than 1 or 0 neighbors
    if (visibleNeighborhood.size() > 1)
        if (graphNeighborhood[0] == visibleNeighborhood[0] && graphNeighborhood[1] == visibleNeighborhood[1])
            //If our two closest neighbors don't share an edge
            if (!boost::edge(visibleNeighborhood[0], visibleNeighborhood[1], g_).second)
            {
                //If they can be directly connected
                if (si_->checkMotion(stateProperty_[visibleNeighborhood[0]], stateProperty_[visibleNeighborhood[1]]))
                {
                    //Connect them
                    connect(visibleNeighborhood[0], visibleNeighborhood[1]);
                    //And report that we added to the roadmap
                    resetFailures();
                    //Report success
                    return true;
                }
                else
                {
                    //Add the new node to the graph, to bridge the interface
                    Vertex v = addGuard(si_->cloneState(qNew), INTERFACE);
                    connect(v, visibleNeighborhood[0]);
                    connect(v, visibleNeighborhood[1]);
                    //Report success
                    return true;
                }
            }
    return false;
}

bool ompl::geometric::SPARStwo::checkAddPath( Vertex v )
{
    bool ret = false;

    std::vector< Vertex > rs;
    foreach( Vertex r, boost::adjacent_vertices( v, g_ ) )
        rs.push_back(r);

    /* Candidate x vertices as described in the method, filled by function computeX(). */
    std::vector<Vertex> Xs;

    /* Candidate v" vertices as described in the method, filled by function computeVPP(). */
    std::vector<Vertex> VPPs;

    for( size_t i=0; i<rs.size() && !ret; ++i )
    {
        Vertex r = rs[i];
        computeVPP(v, r, VPPs);
        foreach (Vertex rp, VPPs)
        {
            //First, compute the longest path through the graph
            computeX(v, r, rp, Xs);
            double rm_dist = 0.0;
            foreach( Vertex rpp, Xs)
            {
                double tmp_dist = (si_->distance( stateProperty_[r], stateProperty_[v] )
                    + si_->distance( stateProperty_[v], stateProperty_[rpp] ) )/2.0;
                if( tmp_dist > rm_dist )
                    rm_dist = tmp_dist;
            }

            InterfaceData& d = getData( v, r, rp );

            //Then, if the spanner property is violated
            if( rm_dist > stretchFactor_ * d.d_ )
            {
                ret = true; //Report that we added for the path
                if( si_->checkMotion( stateProperty_[r], stateProperty_[rp] ) )
                    connect( r, rp );
                else
                {
                    PathGeometric* p = new PathGeometric( si_ );
                    if( r < rp )
                    {
                        p->append( d.sigmaA_ );
                        p->append( d.pointA_ );
                        p->append( stateProperty_[v] );
                        p->append( d.pointB_ );
                        p->append( d.sigmaB_ );
                    }
                    else
                    {
                        p->append( d.sigmaB_ );
                        p->append( d.pointB_ );
                        p->append( stateProperty_[v] );
                        p->append( d.pointA_ );
                        p->append( d.sigmaA_ );
                    }

                    psimp_->reduceVertices(*p, 10);
                    psimp_->shortcutPath(*p, 50);

                    if (p->checkAndRepair( 100 ).second)
                    {
                        Vertex prior = r;
                        Vertex vnew;
                        std::vector<base::State*>& states = p->getStates();

                        foreach( base::State* st, states )
                        {
                            // no need to clone st, since we will destroy p; we just copy the pointer
                            vnew = addGuard( st , QUALITY );

                            connect( prior, vnew );
                            prior = vnew;
                        }
                        // clear the states, so memory is not freed twice
                        states.clear();
                        connect( prior, rp );
                    }

                    delete p;
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

void ompl::geometric::SPARStwo::findGraphNeighbors(base::State* st, std::vector<Vertex> &graphNeighborhood, std::vector<Vertex> &visibleNeighborhood)
{
    visibleNeighborhood.clear();
    stateProperty_[ queryVertex_ ] = st;
    nn_->nearestR( queryVertex_, sparseDelta_, graphNeighborhood);
    stateProperty_[ queryVertex_ ] = NULL;

    //Now that we got the neighbors from the NN, we must remove any we can't see
    for (std::size_t i = 0; i < graphNeighborhood.size() ; ++i )
        if (si_->checkMotion(st, stateProperty_[graphNeighborhood[i]]))
            visibleNeighborhood.push_back(graphNeighborhood[i]);
}

void ompl::geometric::SPARStwo::approachGraph( Vertex v )
{
    std::vector< Vertex > hold;
    nn_->nearestR( v, sparseDelta_, hold );

    std::vector< Vertex > neigh;
    for( size_t i=0; i<hold.size(); ++i )
        if( si_->checkMotion( stateProperty_[v], stateProperty_[hold[i]] ) )
            neigh.push_back( hold[i] );

    foreach( Vertex vp, neigh )
        connect( v, vp );
}

ompl::geometric::SPARStwo::Vertex ompl::geometric::SPARStwo::findGraphRepresentative(base::State* st)
{
    std::vector<Vertex> nbh;
    stateProperty_[ queryVertex_ ] = st;
    nn_->nearestR( queryVertex_, sparseDelta_, nbh);
    stateProperty_[queryVertex_] = NULL;

    Vertex result = boost::graph_traits<Graph>::null_vertex();

    for (std::size_t i = 0 ; i< nbh.size() ; ++i)
        if (si_->checkMotion(st, stateProperty_[nbh[i]]))
        {
            result = nbh[i];
            break;
        }
    return result;
}

void ompl::geometric::SPARStwo::findCloseRepresentatives(base::State *workArea, const base::State *qNew, const Vertex qRep,
                                                         std::map<Vertex, base::State*> &closeRepresentatives,
                                                         const base::PlannerTerminationCondition &ptc)
{
    for (std::map<Vertex, base::State*>::iterator it = closeRepresentatives.begin(); it != closeRepresentatives.end(); ++it)
        si_->freeState(it->second);
    closeRepresentatives.clear();

    // Then, begin searching the space around him
    for (unsigned int i = 0 ; i < nearSamplePoints_ ; ++i)
    {
        do
        {
            sampler_->sampleNear(workArea, qNew, denseDelta_);
        } while ((!si_->isValid(workArea) || si_->distance(qNew, workArea) > denseDelta_ || !si_->checkMotion(qNew, workArea)) && ptc == false);

        // if we were not successful at sampling a desirable state, we are out of time
        if (ptc == false)
            break;

        // Compute who his graph neighbors are
        Vertex representative = findGraphRepresentative(workArea);

        // Assuming this sample is actually seen by somebody (which he should be in all likelihood)
        if (representative != boost::graph_traits<Graph>::null_vertex())
        {
            //If his representative is different than qNew
            if (qRep != representative)
                //And we haven't already tracked this representative
                if (closeRepresentatives.find(representative) == closeRepresentatives.end())
                    //Track the representative
                    closeRepresentatives[representative] = si_->cloneState(workArea);
        }
        else
        {
            //This guy can't be seen by anybody, so we should take this opportunity to add him
            addGuard(si_->cloneState(workArea), COVERAGE);

            //We should also stop our efforts to add a dense path
            for (std::map<Vertex, base::State*>::iterator it = closeRepresentatives.begin(); it != closeRepresentatives.end(); ++it)
                si_->freeState(it->second);
            closeRepresentatives.clear();
            break;
        }
    }
}

void ompl::geometric::SPARStwo::updatePairPoints(Vertex rep, const base::State *q, Vertex r, const base::State *s)
{
    //First of all, we need to compute all candidate r'
    std::vector<Vertex> VPPs;
    computeVPP(rep, r, VPPs);

    //Then, for each pair Pv(r,r')
    foreach (Vertex rp, VPPs)
        //Try updating the pair info
        distanceCheck(rep, q, r, s, rp);
}

void ompl::geometric::SPARStwo::computeVPP(Vertex v, Vertex vp, std::vector<Vertex> &VPPs)
{
    VPPs.clear();
    foreach( Vertex cvpp, boost::adjacent_vertices( v, g_ ) )
        if( cvpp != vp )
            if( !boost::edge( cvpp, vp, g_ ).second )
                VPPs.push_back( cvpp );
}

void ompl::geometric::SPARStwo::computeX(Vertex v, Vertex vp, Vertex vpp, std::vector<Vertex> &Xs)
{
    Xs.clear();

    foreach (Vertex cx, boost::adjacent_vertices(vpp, g_))
        if (boost::edge(cx, v, g_).second && !boost::edge(cx, vp, g_).second)
        {
            InterfaceData& d = getData( v, vpp, cx );
            if ((vpp < cx && d.pointA_) || (cx < vpp && d.pointB_))
                Xs.push_back( cx );
        }
    Xs.push_back(vpp);
}

ompl::geometric::SPARStwo::VertexPair ompl::geometric::SPARStwo::index( Vertex vp, Vertex vpp )
{
    if( vp < vpp )
        return VertexPair( vp, vpp );
    else if( vpp < vp )
        return VertexPair( vpp, vp );
    else
        throw Exception( name_, "Trying to get an index where the pairs are the same point!");
}

ompl::geometric::SPARStwo::InterfaceData& ompl::geometric::SPARStwo::getData( Vertex v, Vertex vp, Vertex vpp )
{
    return interfaceDataProperty_[v][index( vp, vpp )];
}

void ompl::geometric::SPARStwo::setData( Vertex v, Vertex vp, Vertex vpp, const InterfaceData& d )
{
    interfaceDataProperty_[v][index( vp, vpp )] = d;
}

void ompl::geometric::SPARStwo::distanceCheck(Vertex rep, const base::State *q, Vertex r, const base::State *s, Vertex rp)
{
    //Get the info for the current representative-neighbors pair
    InterfaceData& d = getData( rep, r, rp );

    if( r < rp ) // FIRST points represent r (the guy discovered through sampling)
    {
        if( d.pointA_ == NULL ) // If the point we're considering replacing (P_v(r,.)) isn't there
            //Then we know we're doing better, so add it
            d.setFirst( q, s, si_ );
        else //Otherwise, he is there,
        {
            if( d.pointB_ == NULL ) //But if the other guy doesn't exist, we can't compare.
            {
                //Should probably keep the one that is further away from rep?  Not known what to do in this case.
                // TODO: is this not part of the algorithm?
            }
            else //We know both of these points exist, so we can check some distances
                if (si_->distance(q, d.pointB_) < si_->distance(d.pointA_, d.pointB_))
                    //Distance with the new point is good, so set it.
                    d.setFirst( q, s, si_ );
        }
    }
    else // SECOND points represent r (the guy discovered through sampling)
    {
        if (d.pointB_ == NULL ) //If the point we're considering replacing (P_V(.,r)) isn't there...
            //Then we must be doing better, so add it
            d.setSecond(q, s, si_);
        else //Otherwise, he is there
        {
            if (d.pointA_ == NULL ) //But if the other guy doesn't exist, we can't compare.
            {
                //Should we be doing something cool here?
            }
            else
                if (si_->distance(q, d.pointA_) < si_->distance(d.pointB_, d.pointA_))
                    //Distance with the new point is good, so set it
                    d.setSecond(q, s, si_);
        }
    }
    // Lastly, save what we have discovered
    setData(rep, r, rp, d);
}

void ompl::geometric::SPARStwo::abandonLists(base::State* st)
{
    stateProperty_[ queryVertex_ ] = st;

    std::vector< Vertex > hold;
    nn_->nearestR( queryVertex_, sparseDelta_, hold );

    stateProperty_[queryVertex_] = NULL;

    //For each of the vertices
    foreach( Vertex v, hold )
    {
        foreach (VertexPair r, interfaceDataProperty_[v] | boost::adaptors::map_keys)
            interfaceDataProperty_[v][r].clear(si_);
    }
}

ompl::geometric::SPARStwo::Vertex ompl::geometric::SPARStwo::addGuard( base::State *state, GuardType type)
{
    boost::mutex::scoped_lock _(graphMutex_);

    Vertex m = boost::add_vertex(g_);
    stateProperty_[m] = state;
    colorProperty_[m] = type;

    if( !si_->isValid( state ) )
        throw Exception( name_, "Attempting to promote a guard which is an invalid state!");

    abandonLists( state );

    disjointSets_.make_set(m);
    nn_->add(m);
    resetFailures();

    return m;
}

void ompl::geometric::SPARStwo::connect(Vertex v, Vertex vp)
{
    if( v > milestoneCount() )
        OMPL_ERROR("\'From\' Vertex out of range : %u\n", v );
    if( vp > milestoneCount() )
        OMPL_ERROR("\'To\' Vertex out of range : %u\n", vp );

    const double weight = distanceFunction(v, vp);
    const Graph::edge_property_type properties(weight);
    boost::add_edge(v, vp, properties, g_);
    uniteComponents( v, vp );
}

void ompl::geometric::SPARStwo::uniteComponents(Vertex m1, Vertex m2)
{
    disjointSets_.union_set(m1, m2);
}

ompl::base::PathPtr ompl::geometric::SPARStwo::constructSolution(const Vertex start, const Vertex goal) const
{
    boost::mutex::scoped_lock _(graphMutex_);

    boost::vector_property_map<Vertex> prev(boost::num_vertices(g_));

    try
    {
        boost::astar_search(g_, start,
                            boost::bind(&SPARStwo::distanceFunction, this, _1, goal),
                            boost::predecessor_map(prev).
                            visitor(AStarGoalVisitor<Vertex>(goal)));
    }
    catch (AStarFoundGoal&)
    {
    }

    if (prev[goal] == goal)
        throw Exception(name_, "Could not find solution path");
    else
    {
	PathGeometric *p = new PathGeometric(si_);
        for (Vertex pos = goal; prev[pos] != pos; pos = prev[pos])
            p->append(stateProperty_[pos]);
	p->append(stateProperty_[start]);
	p->reverse();

	return base::PathPtr(p);
    }
}

void ompl::geometric::SPARStwo::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    // Explicitly add start and goal states:
    for (size_t i = 0; i < startM_.size(); ++i)
        data.addStartVertex(base::PlannerDataVertex(stateProperty_[startM_[i]], (int)START));

    for (size_t i = 0; i < goalM_.size(); ++i)
        data.addGoalVertex(base::PlannerDataVertex(stateProperty_[goalM_[i]], (int)GOAL));

    // If there are even edges here
    if( boost::num_edges( g_ ) > 0 )
    {
        // Adding edges and all other vertices simultaneously
        foreach(const Edge e, boost::edges(g_))
        {
            const Vertex v1 = boost::source(e, g_);
            const Vertex v2 = boost::target(e, g_);
            unsigned long size = boost::num_vertices( g_ );
            if (v1 < size || v2 < size)
            {
                data.addEdge(base::PlannerDataVertex(stateProperty_[v1], (int)colorProperty_[v1]),
                             base::PlannerDataVertex(stateProperty_[v2], (int)colorProperty_[v2]));

                // Add the reverse edge, since we're constructing an undirected roadmap
                data.addEdge(base::PlannerDataVertex(stateProperty_[v2], (int)colorProperty_[v2]),
                             base::PlannerDataVertex(stateProperty_[v1], (int)colorProperty_[v1]));
            }
            else
                OMPL_ERROR("Edge Vertex Error: [%lu][%lu] > %lu\n", v1, v2, size);
        }
    }
    else
        OMPL_INFORM("There are no edges in the graph!\n");

    // Make sure to add edge-less nodes as well
    foreach (const Vertex n, boost::vertices(g_))
        if( boost::out_degree( n, g_ ) == 0 )
            data.addVertex(base::PlannerDataVertex(stateProperty_[n], (int)colorProperty_[n]));
}
