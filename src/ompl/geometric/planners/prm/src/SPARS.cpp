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

/* Author: Andrew Dobson */

#include "ompl/geometric/planners/prm/SPARS.h"
#include "ompl/geometric/planners/prm/ConnectionStrategy.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/tools/config/MagicConstants.h"
#include <boost/bind.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <boost/foreach.hpp>

#include "GoalVisitor.hpp"

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
    consecutiveFailures_(0),
    iterations_(0),
    stretchFactor_(3.),
    maxFailures_(1000),
    addedSolution_(false),
    denseDeltaFraction_(.001),
    sparseDeltaFraction_(.25),
    denseDelta_(0.),
    sparseDelta_(0.)
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = false;
    specs_.optimizingPaths = false;

    psimp_.reset(new PathSimplifier(si_));
    psimp_->freeStates(false);

    Planner::declareParam<double>("stretch_factor", this, &SPARS::setStretchFactor, &SPARS::getStretchFactor, "1.1:0.1:3.0");
    Planner::declareParam<double>("sparse_delta_fraction", this, &SPARS::setSparseDeltaFraction, &SPARS::getSparseDeltaFraction, "0.0:0.01:1.0");
    Planner::declareParam<double>("dense_delta_fraction", this, &SPARS::setDenseDeltaFraction, &SPARS::getDenseDeltaFraction, "0.0:0.0001:0.1");
    Planner::declareParam<unsigned int>("max_failures", this, &SPARS::setMaxFailures, &SPARS::getMaxFailures, "100:10:3000");
}

ompl::geometric::SPARS::~SPARS()
{
    freeMemory();
}

void ompl::geometric::SPARS::setup()
{
    Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<DenseVertex>(si_->getStateSpace()));
    nn_->setDistanceFunction(boost::bind(&SPARS::distanceFunction, this, _1, _2));
    if (!snn_)
        snn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<SparseVertex>(si_->getStateSpace()));
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

void ompl::geometric::SPARS::resetFailures()
{
    consecutiveFailures_ = 0;
}

void ompl::geometric::SPARS::clearQuery()
{
    startM_.clear();
    goalM_.clear();
    pis_.restart();
}

void ompl::geometric::SPARS::clear()
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
    resetFailures();
    iterations_ = 0;
}

void ompl::geometric::SPARS::freeMemory()
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
}

ompl::geometric::SPARS::DenseVertex ompl::geometric::SPARS::addSample(base::State *workState, const base::PlannerTerminationCondition &ptc)
{
    DenseVertex result = boost::graph_traits<DenseGraph>::null_vertex();

    // search for a valid state
    bool found = false;
    while (!found && ptc == false)
    {
        unsigned int attempts = 0;
        do
        {
            found = sampler_->sample(workState);
            attempts++;
        } while (attempts < magic::FIND_VALID_STATE_ATTEMPTS_WITHOUT_TERMINATION_CHECK && !found);
    }

    if (found)
        result = addMilestone(si_->cloneState(workState));
    return result;
}

void ompl::geometric::SPARS::checkForSolution(const base::PlannerTerminationCondition &ptc, base::PathPtr &solution)
{
    base::GoalSampleableRegion *goal = static_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());
    while (!ptc && !addedSolution_)
    {
        // Check for any new goal states
        if (goal->maxSampleCount() > goalM_.size())
        {
            const base::State *st = pis_.nextGoal();
            if (st)
            {
                addMilestone(si_->cloneState(st));
                goalM_.push_back(addGuard(si_->cloneState(st), GOAL));
            }
        }

        // Check for a solution
        addedSolution_ = haveSolution(startM_, goalM_, solution);
        // Sleep for 1ms
        if (!addedSolution_)
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    }
}

bool ompl::geometric::SPARS::haveSolution(const std::vector<DenseVertex> &starts, const std::vector<DenseVertex> &goals, base::PathPtr &solution)
{
    base::Goal *g = pdef_->getGoal().get();
    foreach (DenseVertex start, starts)
    {
        foreach (DenseVertex goal, goals)
        {
            // we lock because the connected components algorithm is incremental and may change disjointSets_
            graphMutex_.lock();
            bool same_component = sameComponent(start, goal);
            graphMutex_.unlock();

            if (same_component && g->isStartGoalPairValid(sparseStateProperty_[goal], sparseStateProperty_[start]))
            {
                solution = constructSolution(start, goal);
                return true;
            }
        }
    }

    return false;
}

bool ompl::geometric::SPARS::reachedTerminationCriterion() const
{
    return consecutiveFailures_ >= maxFailures_ || addedSolution_;
}

bool ompl::geometric::SPARS::reachedFailureLimit() const
{
    return consecutiveFailures_ >= maxFailures_;
}

void ompl::geometric::SPARS::checkQueryStateInitialization()
{
    boost::mutex::scoped_lock _(graphMutex_);
    if (boost::num_vertices(g_) < 1)
    {
        sparseQueryVertex_ = boost::add_vertex(s_);
        queryVertex_ = boost::add_vertex(g_);
        sparseStateProperty_[sparseQueryVertex_] = NULL;
        stateProperty_[queryVertex_] = NULL;
    }
}

bool ompl::geometric::SPARS::sameComponent(SparseVertex m1, SparseVertex m2)
{
    return boost::same_component(m1, m2, sparseDJSets_);
}

ompl::base::PlannerStatus ompl::geometric::SPARS::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    checkQueryStateInitialization();

    base::GoalSampleableRegion *goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

    if (!goal)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
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
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (goalM_.empty() && !goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    // Add the valid goal states as milestones
    while (const base::State *st = (goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal()))
    {
        addMilestone(si_->cloneState(st));
        goalM_.push_back(addGuard(si_->cloneState(st), GOAL ));
    }
    if (goalM_.empty())
    {
        OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    unsigned int nrStartStatesDense = boost::num_vertices(g_) - 1; // don't count query vertex
    unsigned int nrStartStatesSparse = boost::num_vertices(s_) - 1; // don't count query vertex
    OMPL_INFORM("%s: Starting planning with %u dense states, %u sparse states", getName().c_str(), nrStartStatesDense, nrStartStatesSparse);

    // Reset addedSolution_ member
    addedSolution_ = false;
    resetFailures();
    base::PathPtr sol;
    base::PlannerTerminationCondition ptcOrFail =
        base::plannerOrTerminationCondition(ptc, base::PlannerTerminationCondition(boost::bind(&SPARS::reachedFailureLimit, this)));
    boost::thread slnThread(boost::bind(&SPARS::checkForSolution, this, ptcOrFail, boost::ref(sol)));

    // Construct planner termination condition which also takes maxFailures_ and addedSolution_ into account
    base::PlannerTerminationCondition ptcOrStop =
        base::plannerOrTerminationCondition(ptc, base::PlannerTerminationCondition(boost::bind(&SPARS::reachedTerminationCriterion, this)));
    constructRoadmap(ptcOrStop);

    // Ensure slnThread is ceased before exiting solve
    slnThread.join();

    if (sol)
        pdef_->addSolutionPath(sol, false, -1.0, getName());

    OMPL_INFORM("%s: Created %u dense states, %u sparse states", getName().c_str(),
                (unsigned int)(boost::num_vertices(g_) - nrStartStatesDense),
                (unsigned int)(boost::num_vertices(s_) - nrStartStatesSparse));

    // Return true if any solution was found.
    return sol ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::SPARS::constructRoadmap(const base::PlannerTerminationCondition &ptc, bool stopOnMaxFail)
{
    if (stopOnMaxFail)
    {
        resetFailures();
        base::PlannerTerminationCondition ptcOrFail =
	    base::plannerOrTerminationCondition(ptc, base::PlannerTerminationCondition(boost::bind(&SPARS::reachedFailureLimit, this)));
        constructRoadmap(ptcOrFail);
    }
    else
        constructRoadmap(ptc);
}

void ompl::geometric::SPARS::constructRoadmap(const base::PlannerTerminationCondition &ptc)
{
    checkQueryStateInitialization();

    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();
    if (!simpleSampler_)
        simpleSampler_ = si_->allocStateSampler();

    base::State *workState = si_->allocState();

    /* The whole neighborhood set which has been most recently computed */
    std::vector<SparseVertex> graphNeighborhood;

    /* The visible neighborhood set which has been most recently computed */
    std::vector<SparseVertex> visibleNeighborhood;

    /* Storage for the interface neighborhood, populated by getInterfaceNeighborhood() */
    std::vector<DenseVertex> interfaceNeighborhood;

    while (ptc == false)
    {
        iterations_++;

        // Generate a single sample, and attempt to connect it to nearest neighbors.
        DenseVertex q = addSample(workState, ptc);
        if (q == boost::graph_traits<DenseGraph>::null_vertex())
            continue;

        //Now that we've added to D, try adding to S
        //Start by figuring out who our neighbors are
        getSparseNeighbors(workState, graphNeighborhood);
        filterVisibleNeighbors(workState, graphNeighborhood, visibleNeighborhood);
        //Check for addition for Coverage
        if( !checkAddCoverage(workState, graphNeighborhood))
            //If not for Coverage, then Connectivity
            if( !checkAddConnectivity(workState, graphNeighborhood))
                //Check for the existence of an interface
                if( !checkAddInterface(graphNeighborhood, visibleNeighborhood, q))
                {
                    // Then check to see if it's on an interface
                    getInterfaceNeighborhood(q, interfaceNeighborhood);
                    if (interfaceNeighborhood.size() > 0)
                    {
                        //Check for addition for spanner prop
                        if (!checkAddPath(q, interfaceNeighborhood))
                            //All of the tests have failed.  Report failure for the sample
                            ++consecutiveFailures_;
                    }
                    else
                        //There's no interface here, so drop it
                        ++consecutiveFailures_;
                }
    }

    si_->freeState(workState);
}

ompl::geometric::SPARS::DenseVertex ompl::geometric::SPARS::addMilestone(base::State *state)
{
    boost::mutex::scoped_lock _(graphMutex_);

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

    std::vector<DenseVertex> interfaceNeighborhood;
    std::set<SparseVertex> interfaceRepresentatives;

    getInterfaceNeighborRepresentatives(m, interfaceRepresentatives);
    getInterfaceNeighborhood(m, interfaceNeighborhood);
    addToRepresentatives(m, representativesProperty_[m], interfaceRepresentatives);
    foreach (DenseVertex qp, interfaceNeighborhood)
    {
        removeFromRepresentatives( qp, representativesProperty_[qp] );
        getInterfaceNeighborRepresentatives( qp, interfaceRepresentatives );
        addToRepresentatives( qp, representativesProperty_[qp], interfaceRepresentatives );
    }

    return m;
}

ompl::geometric::SPARS::SparseVertex ompl::geometric::SPARS::addGuard(base::State *state, GuardType type)
{
    boost::mutex::scoped_lock _(graphMutex_);

    SparseVertex v = boost::add_vertex(s_);
    sparseStateProperty_[v] = state;
    sparseColorProperty_[v] = type;

    sparseDJSets_.make_set(v);

    snn_->add(v);
    updateRepresentatives(v);

    resetFailures();
    return v;
}

void ompl::geometric::SPARS::connectSparsePoints(SparseVertex v, SparseVertex vp)
{
    const double weight = sparseDistanceFunction(v, vp);
    const SpannerGraph::edge_property_type properties(weight);
    boost::mutex::scoped_lock _(graphMutex_);
    boost::add_edge(v, vp, properties, s_);
    sparseDJSets_.union_set(v, vp);
}

void ompl::geometric::SPARS::connectDensePoints( DenseVertex v, DenseVertex vp )
{
    const double weight = distanceFunction(v, vp);
    const DenseGraph::edge_property_type properties(weight);
    boost::mutex::scoped_lock _(graphMutex_);
    boost::add_edge(v, vp, properties, g_);
}

bool ompl::geometric::SPARS::checkAddCoverage(const base::State *lastState, const std::vector<SparseVertex>& neigh )
{
    //For each of these neighbors,
    foreach (SparseVertex n, neigh)
        //If path between is free
        if (si_->checkMotion( lastState, sparseStateProperty_[n]))
            //Abort out and return false
            return false;
    //No free paths means we add for coverage
    addGuard(si_->cloneState(lastState), COVERAGE);
    return true;
}

bool ompl::geometric::SPARS::checkAddConnectivity( const base::State *lastState, const std::vector<SparseVertex>& neigh )
{
    std::vector< SparseVertex > links;
    //For each neighbor
    for (std::size_t i = 0; i < neigh.size(); ++i )
        //For each other neighbor
        for (std::size_t j = i + 1; j < neigh.size(); ++j )
            //If they are in different components
            if (!sameComponent(neigh[i], neigh[j]))
                //If the paths between are collision free
                if( si_->checkMotion( lastState, sparseStateProperty_[neigh[i]] ) && si_->checkMotion( lastState, sparseStateProperty_[neigh[j]] ) )
                {
                    links.push_back( neigh[i] );
                    links.push_back( neigh[j] );
                }

    if( links.size() != 0 )
    {
        //Add the node
        SparseVertex g = addGuard( si_->cloneState(lastState), CONNECTIVITY );

        for (std::size_t i = 0; i < links.size(); ++i )
            //If there's no edge
            if (!boost::edge(g, links[i], s_).second)
                //And the components haven't been united by previous links
                if (!sameComponent(links[i], g))
                    connectSparsePoints( g, links[i] );
        return true;
    }
    return false;
}

bool ompl::geometric::SPARS::checkAddInterface(const std::vector<SparseVertex>& graphNeighborhood, const std::vector<SparseVertex>& visibleNeighborhood, DenseVertex q )
{
    //If we have more than 1 neighbor
    if( visibleNeighborhood.size() > 1 )
        //If our closest neighbors are also visible
        if( graphNeighborhood[0] == visibleNeighborhood[0] && graphNeighborhood[1] == visibleNeighborhood[1] )
            //If our two closest neighbors don't share an edge
            if (!boost::edge(visibleNeighborhood[0], visibleNeighborhood[1], s_).second)
            {
                //If they can be directly connected
                if( si_->checkMotion( sparseStateProperty_[visibleNeighborhood[0]], sparseStateProperty_[visibleNeighborhood[1]] ) )
                {
                    //Connect them
                    connectSparsePoints( visibleNeighborhood[0], visibleNeighborhood[1] );
                    //And report that we added to the roadmap
                    resetFailures();
                    //Report success
                    return true;
                }
                else
                {
                    //Add the new node to the graph, to bridge the interface
                    SparseVertex v = addGuard( si_->cloneState( stateProperty_[q] ), INTERFACE );
                    connectSparsePoints( v, visibleNeighborhood[0] );
                    connectSparsePoints( v, visibleNeighborhood[1] );
                    //Report success
                    return true;
                }
            }
    return false;
}

bool ompl::geometric::SPARS::checkAddPath(DenseVertex q, const std::vector<DenseVertex>& neigh)
{
    bool result = false;

    //Get q's representative => v
    SparseVertex v = representativesProperty_[q];

    //Extract the representatives of neigh => n_rep
    std::set<SparseVertex> n_rep;
    foreach( DenseVertex qp, neigh )
        n_rep.insert(representativesProperty_[qp]);

    std::vector<SparseVertex> Xs;
    //for each v' in n_rep
    for (std::set<SparseVertex>::iterator it = n_rep.begin() ; it != n_rep.end() && !result ; ++it)
    {
        SparseVertex vp = *it;
        //Identify appropriate v" candidates => vpps
        std::vector<SparseVertex> VPPs;
        computeVPP(v, vp, VPPs);

        foreach( SparseVertex vpp, VPPs )
        {
            double s_max = 0;
            //Find the X nodes to test
            computeX(v, vp, vpp, Xs);

            //For each x in xs
            foreach( SparseVertex x, Xs )
            {
                //Compute/Retain MAXimum distance path thorugh S
                double dist = (si_->distance(sparseStateProperty_[x], sparseStateProperty_[v])
                    + si_->distance(sparseStateProperty_[v], sparseStateProperty_[vp])) / 2.0;
                if( dist > s_max )
                    s_max = dist;
            }

            DensePath bestDPath;
            DenseVertex best_qpp = boost::graph_traits<DenseGraph>::null_vertex();
            double d_min = std::numeric_limits<double>::infinity(); //Insanely big number
            //For each vpp in vpps
            for (std::size_t j = 0; j < VPPs.size() && !result; ++j)
            {
                SparseVertex vpp = VPPs[j];
                //For each q", which are stored interface nodes on v for i(vpp,v)
                foreach( DenseVertex qpp, interfaceListsProperty_[v].interfaceHash[vpp] )
                {
                    // check that representatives are consistent
                    assert(representativesProperty_[qpp] == v);

                    //If they happen to be the one and same node
                    if (q == qpp)
                    {
                        bestDPath.push_front( stateProperty_[q] );
                        best_qpp = qpp;
                        d_min = 0;
                    }
                    else
                    {
                        //Compute/Retain MINimum distance path on D through q, q"
                        DensePath dPath;
                        computeDensePath(q, qpp, dPath);
                        if (dPath.size() > 0)
                        {
                            // compute path length
                            double length = 0.0;
                            DensePath::const_iterator jt = dPath.begin();
                            for (DensePath::const_iterator it = jt + 1 ; it != dPath.end() ; ++it)
                            {
                                length += si_->distance(*jt, *it);
                                jt = it;
                            }

                            if (length < d_min)
                            {
                                d_min = length;
                                bestDPath.swap(dPath);
                                best_qpp = qpp;
                            }
                        }
                    }
                }

                //If the spanner property is violated for these paths
                if (s_max > stretchFactor_* d_min)
                {
                    //Need to augment this path with the appropriate neighbor information
                    DenseVertex na = getInterfaceNeighbor(q, vp);
                    DenseVertex nb = getInterfaceNeighbor(best_qpp, vpp);

                    bestDPath.push_front( stateProperty_[na] );
                    bestDPath.push_back( stateProperty_[nb] );

                    // check consistency of representatives
                    assert(representativesProperty_[na] == vp && representativesProperty_[nb] == vpp);

                    //Add the dense path to the spanner
                    addPathToSpanner( bestDPath, vpp, vp );

                    //Report success
                    result = true;
                }
            }
        }
    }
    return result;
}

double ompl::geometric::SPARS::averageValence() const
{
    double degree = 0.0;
    foreach (DenseVertex v, boost::vertices(s_))
        degree += (double)boost::out_degree(v, s_);
    degree /= (double)boost::num_vertices(s_);
    return degree;
}

void ompl::geometric::SPARS::printDebug(std::ostream &out) const
{
    out << "SPARS Debug Output: " << std::endl;
    out << "  Settings: " << std::endl;
    out << "    Max Failures: " << getMaxFailures() << std::endl;
    out << "    Dense Delta Fraction: " << getDenseDeltaFraction() << std::endl;
    out << "    Sparse Delta Fraction: " << getSparseDeltaFraction() << std::endl;
    out << "    Stretch Factor: " << getStretchFactor() << std::endl;
    out << "  Status: " << std::endl;
    out << "    Milestone Count: " << milestoneCount() << std::endl;
    out << "    Guard Count: " << guardCount() << std::endl;
    out << "    Iterations: " << getIterations() << std::endl;
    out << "    Average Valence: " << averageValence() << std::endl;
    out << "    Consecutive Failures: " << consecutiveFailures_ << std::endl;
}

void ompl::geometric::SPARS::getSparseNeighbors(base::State *inState, std::vector<SparseVertex> &graphNeighborhood)
{
    sparseStateProperty_[sparseQueryVertex_] = inState;

    graphNeighborhood.clear();
    snn_->nearestR(sparseQueryVertex_, sparseDelta_, graphNeighborhood);

    sparseStateProperty_[sparseQueryVertex_] = NULL;
}

void ompl::geometric::SPARS::filterVisibleNeighbors(base::State *inState, const std::vector<SparseVertex> &graphNeighborhood,
                                                    std::vector<SparseVertex> &visibleNeighborhood) const
{
    visibleNeighborhood.clear();
    //Now that we got the neighbors from the NN, we must remove any we can't see
    for (std::size_t i = 0; i < graphNeighborhood.size(); ++i)
        if (si_->checkMotion(inState, sparseStateProperty_[graphNeighborhood[i]]))
            visibleNeighborhood.push_back(graphNeighborhood[i]);
}

ompl::geometric::SPARS::DenseVertex ompl::geometric::SPARS::getInterfaceNeighbor(DenseVertex q, SparseVertex rep)
{
    foreach (DenseVertex vp, boost::adjacent_vertices( q, g_ ))
        if (representativesProperty_[vp] == rep )
            if (distanceFunction( q, vp ) <= denseDelta_)
                return vp;
    throw Exception(name_, "Vertex has no interface neighbor with given representative");
}

bool ompl::geometric::SPARS::addPathToSpanner( const DensePath &dense_path, SparseVertex vp, SparseVertex vpp )
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

void ompl::geometric::SPARS::updateRepresentatives(SparseVertex v)
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
        removeFromRepresentatives( dense_points[i], representativesProperty_[dense_points[i]] );
        //Update that point's representative
        calculateRepresentative( dense_points[i] );
    }

    std::set<SparseVertex> interfaceRepresentatives;
    //For each of the points
    for (std::size_t i = 0 ; i < dense_points.size(); ++i)
    {
        //Get it's representative
        SparseVertex rep = representativesProperty_[dense_points[i]];
        //Extract the representatives of any interface-sharing neighbors
        getInterfaceNeighborRepresentatives( dense_points[i], interfaceRepresentatives );
        //For sanity's sake, make sure we clear ourselves out of what this new rep might think of us
        removeFromRepresentatives( dense_points[i], rep );
        //Add this vertex to it's representative's list for the other representatives
        addToRepresentatives( dense_points[i], rep, interfaceRepresentatives );
    }
}

void ompl::geometric::SPARS::calculateRepresentative(DenseVertex q)
{
    //Get the nearest neighbors within sparseDelta_
    std::vector<SparseVertex> graphNeighborhood;
    getSparseNeighbors(stateProperty_[q], graphNeighborhood);

    //For each neighbor
    for (std::size_t i = 0; i < graphNeighborhood.size(); ++i)
        if (si_->checkMotion(stateProperty_[q], sparseStateProperty_[graphNeighborhood[i]]))
        {
            //update the representative
            representativesProperty_[q] = graphNeighborhood[i];
            //abort
            break;
        }
}

void ompl::geometric::SPARS::addToRepresentatives(DenseVertex q, SparseVertex rep, const std::set<SparseVertex> &oreps)
{
    //If this node supports no interfaces
    if (oreps.size() == 0)
    {
        //Add it to the pool of non-interface nodes
        bool new_insert = nonInterfaceListsProperty_[rep].insert(q).second;

        // we expect this was not previously tracked
        if (!new_insert)
            assert(false);
    }
    else
    {
        //otherwise, for every neighbor representative
        foreach( SparseVertex v, oreps )
        {
            assert(rep == representativesProperty_[q]);
            bool new_insert = interfaceListsProperty_[rep].interfaceHash[v].insert(q).second;
            if (!new_insert)
                assert(false);
        }
    }
}

void ompl::geometric::SPARS::removeFromRepresentatives(DenseVertex q, SparseVertex rep)
{
    // Remove the node from the non-interface points (if there)
    nonInterfaceListsProperty_[rep].erase(q);

    // From each of the interfaces
    foreach (SparseVertex vpp, interfaceListsProperty_[rep].interfaceHash | boost::adaptors::map_keys)
    {
        // Remove this node from that list
        interfaceListsProperty_[rep].interfaceHash[vpp].erase( q );
    }
}

void ompl::geometric::SPARS::computeVPP(SparseVertex v, SparseVertex vp, std::vector<SparseVertex> &VPPs)
{
    foreach( SparseVertex cvpp, boost::adjacent_vertices( v, s_ ) )
        if( cvpp != vp )
            if( !boost::edge( cvpp, vp, s_ ).second )
                VPPs.push_back( cvpp );
}

void ompl::geometric::SPARS::computeX(SparseVertex v, SparseVertex vp, SparseVertex vpp, std::vector<SparseVertex> &Xs)
{
    Xs.clear();
    foreach( SparseVertex cx, boost::adjacent_vertices( vpp, s_ ) )
        if( boost::edge( cx, v, s_ ).second && !boost::edge( cx, vp, s_ ).second )
            if (interfaceListsProperty_[vpp].interfaceHash[cx].size() > 0)
                Xs.push_back( cx );
    Xs.push_back( vpp );
}

void ompl::geometric::SPARS::getInterfaceNeighborRepresentatives(DenseVertex q, std::set<SparseVertex> &interfaceRepresentatives)
{
    interfaceRepresentatives.clear();

    // Get our representative
    SparseVertex rep = representativesProperty_[q];
    // For each neighbor we are connected to
    foreach( DenseVertex n, boost::adjacent_vertices( q, g_ ) )
    {
        // Get his representative
        SparseVertex orep = representativesProperty_[n];
        // If that representative is not our own
        if (orep != rep)
            // If he is within denseDelta_
            if (distanceFunction( q, n ) < denseDelta_)
                // Include his rep in the set
                interfaceRepresentatives.insert(orep);
    }
}

void ompl::geometric::SPARS::getInterfaceNeighborhood(DenseVertex q, std::vector<DenseVertex> &interfaceNeighborhood)
{
    interfaceNeighborhood.clear();

    // Get our representative
    SparseVertex rep = representativesProperty_[q];

    // For each neighbor we are connected to
    foreach( DenseVertex n, boost::adjacent_vertices( q, g_ ) )
        // If neighbor representative is not our own
        if( representativesProperty_[n] != rep )
            // If he is within denseDelta_
            if( distanceFunction( q, n ) < denseDelta_ )
                // Append him to the list
                interfaceNeighborhood.push_back( n );
}

ompl::base::PathPtr ompl::geometric::SPARS::constructSolution(const SparseVertex start, const SparseVertex goal) const
{
    boost::mutex::scoped_lock _(graphMutex_);

    boost::vector_property_map<SparseVertex> prev(boost::num_vertices(s_));

    try
    {
        boost::astar_search(s_, start,
                            boost::bind(&SPARS::sparseDistanceFunction, this, _1, goal),
                            boost::predecessor_map(prev).
                            visitor(AStarGoalVisitor<SparseVertex>(goal)));
    }
    catch (AStarFoundGoal&)
    {
    }

    if (prev[goal] == goal)
        throw Exception(name_, "Could not find solution path");
    else
    {
        PathGeometric *p = new PathGeometric(si_);

        for (SparseVertex pos = goal; prev[pos] != pos; pos = prev[pos])
            p->append(sparseStateProperty_[pos]);
        p->append(sparseStateProperty_[start]);
        p->reverse();

        return base::PathPtr(p);
    }
}

void ompl::geometric::SPARS::computeDensePath(const DenseVertex start, const DenseVertex goal, DensePath &path) const
{
    path.clear();

    boost::vector_property_map<DenseVertex> prev(boost::num_vertices(g_));

    try
    {
        boost::astar_search(g_, start,
                            boost::bind(&SPARS::distanceFunction, this, _1, goal),
                            boost::predecessor_map(prev).
                            visitor(AStarGoalVisitor<DenseVertex>(goal)));
    }
    catch (AStarFoundGoal&)
    {
    }

    if (prev[goal] == goal)
        OMPL_WARN("%s: No dense path was found?", getName().c_str());
    else
    {
        for (DenseVertex pos = goal; prev[pos] != pos; pos = prev[pos])
            path.push_front( stateProperty_[pos] );
        path.push_front( stateProperty_[start] );
    }
}

void ompl::geometric::SPARS::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    // Explicitly add start and goal states:
    for (std::size_t i = 0; i < startM_.size(); ++i)
        data.addStartVertex(base::PlannerDataVertex(sparseStateProperty_[startM_[i]], (int)START));

    for (std::size_t i = 0; i < goalM_.size(); ++i)
        data.addGoalVertex(base::PlannerDataVertex(sparseStateProperty_[goalM_[i]], (int)GOAL));

    // Adding edges and all other vertices simultaneously
    foreach (const SparseEdge e, boost::edges(s_))
    {
        const SparseVertex v1 = boost::source(e, s_);
        const SparseVertex v2 = boost::target(e, s_);
        data.addEdge(base::PlannerDataVertex(sparseStateProperty_[v1], (int)sparseColorProperty_[v1]),
                     base::PlannerDataVertex(sparseStateProperty_[v2], (int)sparseColorProperty_[v2]));

        // Add the reverse edge, since we're constructing an undirected roadmap
        data.addEdge(base::PlannerDataVertex(sparseStateProperty_[v2], (int)sparseColorProperty_[v1]),
                     base::PlannerDataVertex(sparseStateProperty_[v1], (int)sparseColorProperty_[v2]));
    }

    // Make sure to add edge-less nodes as well
    foreach (const SparseVertex n, boost::vertices(s_))
        if (boost::out_degree( n, s_ ) == 0)
            data.addVertex( base::PlannerDataVertex(sparseStateProperty_[n], (int)sparseColorProperty_[n]));
    data.properties["iterations INTEGER"] = boost::lexical_cast<std::string>(iterations_);
}
