/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Rutgers the State University of New Jersey, New Brunswick
 *  Copyright (c) 2014, University of Colorado, Boulder
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

/* Author: Andrew Dobson, Dave Coleman */

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/tools/thunder/SPARSdb.h>
#include <ompl/util/Console.h>
#include <boost/foreach.hpp>
#include <boost/graph/astar_search.hpp>
#include <boost/graph/incremental_components.hpp>
#include <boost/property_map/vector_property_map.hpp>
#include <random>

// Allow hooks for visualizing planner
//#define OMPL_THUNDER_DEBUG

#define foreach BOOST_FOREACH
#define foreach_reverse BOOST_REVERSE_FOREACH

// edgeWeightMap methods ////////////////////////////////////////////////////////////////////////////

BOOST_CONCEPT_ASSERT(
    (boost::ReadablePropertyMapConcept<ompl::geometric::SPARSdb::edgeWeightMap, ompl::geometric::SPARSdb::Edge>));

ompl::geometric::SPARSdb::edgeWeightMap::edgeWeightMap(const Graph &graph, const EdgeCollisionStateMap &collisionStates)
  : g_(graph), collisionStates_(collisionStates)
{
}

double ompl::geometric::SPARSdb::edgeWeightMap::get(Edge e) const
{
    // Get the status of collision checking for this edge
    if (collisionStates_[e] == IN_COLLISION)
        return std::numeric_limits<double>::infinity();

    return boost::get(boost::edge_weight, g_, e);
}

namespace boost
{
    double get(const ompl::geometric::SPARSdb::edgeWeightMap &m, const ompl::geometric::SPARSdb::Edge &e)
    {
        return m.get(e);
    }
}

// CustomVisitor methods ////////////////////////////////////////////////////////////////////////////

BOOST_CONCEPT_ASSERT(
    (boost::AStarVisitorConcept<ompl::geometric::SPARSdb::CustomVisitor, ompl::geometric::SPARSdb::Graph>));

ompl::geometric::SPARSdb::CustomVisitor::CustomVisitor(Vertex goal) : goal(goal)
{
}

void ompl::geometric::SPARSdb::CustomVisitor::examine_vertex(Vertex u, const Graph &) const
{
    if (u == goal)
        throw foundGoalException();
}

// SPARSdb methods ////////////////////////////////////////////////////////////////////////////////////////

ompl::geometric::SPARSdb::SPARSdb(const base::SpaceInformationPtr &si)
  : base::Planner(si, "SPARSdb")
  // Numeric variables
  , nearSamplePoints_((2 * si_->getStateDimension()))
  // Property accessors of edges
  , edgeWeightProperty_(boost::get(boost::edge_weight, g_))
  , edgeCollisionStateProperty_(boost::get(edge_collision_state_t(), g_))
  // Property accessors of vertices
  , stateProperty_(boost::get(vertex_state_t(), g_))
  , colorProperty_(boost::get(vertex_color_t(), g_))
  , interfaceDataProperty_(boost::get(vertex_interface_data_t(), g_))
  // Disjoint set accessors
  , disjointSets_(boost::get(boost::vertex_rank, g_), boost::get(boost::vertex_predecessor, g_))
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.approximateSolutions = false;
    specs_.optimizingPaths = true;

    psimp_ = std::make_shared<PathSimplifier>(si_);

    Planner::declareParam<double>("stretch_factor", this, &SPARSdb::setStretchFactor, &SPARSdb::getStretchFactor,
                                  "1.1:0.1:3.0");
    Planner::declareParam<double>("sparse_delta_fraction", this, &SPARSdb::setSparseDeltaFraction,
                                  &SPARSdb::getSparseDeltaFraction, "0.0:0.01:1.0");
    Planner::declareParam<double>("dense_delta_fraction", this, &SPARSdb::setDenseDeltaFraction,
                                  &SPARSdb::getDenseDeltaFraction, "0.0:0.0001:0.1");
    Planner::declareParam<unsigned int>("max_failures", this, &SPARSdb::setMaxFailures, &SPARSdb::getMaxFailures,
                                        "100:10:3000");
}

ompl::geometric::SPARSdb::~SPARSdb()
{
    freeMemory();
}

void ompl::geometric::SPARSdb::setup()
{
    Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Vertex>(this));
    nn_->setDistanceFunction([this](const Vertex a, const Vertex b)
                             {
                                 return distanceFunction(a, b);
                             });
    double maxExt = si_->getMaximumExtent();
    sparseDelta_ = sparseDeltaFraction_ * maxExt;
    denseDelta_ = denseDeltaFraction_ * maxExt;

    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();
}

void ompl::geometric::SPARSdb::setProblemDefinition(const base::ProblemDefinitionPtr &pdef)
{
    Planner::setProblemDefinition(pdef);
    clearQuery();
}

void ompl::geometric::SPARSdb::clearQuery()
{
    startM_.clear();
    goalM_.clear();
    pis_.restart();
}

void ompl::geometric::SPARSdb::clear()
{
    Planner::clear();
    clearQuery();
    resetFailures();
    iterations_ = 0;
    freeMemory();
    if (nn_)
        nn_->clear();
}

void ompl::geometric::SPARSdb::freeMemory()
{
    Planner::clear();
    sampler_.reset();

    foreach (Vertex v, boost::vertices(g_))
    {
        foreach (InterfaceData &d, interfaceDataProperty_[v].interfaceHash | boost::adaptors::map_values)
            d.clear(si_);
        if (stateProperty_[v] != nullptr)
            si_->freeState(stateProperty_[v]);
        stateProperty_[v] = nullptr;
    }
    g_.clear();

    if (nn_)
        nn_->clear();
}

bool ompl::geometric::SPARSdb::getSimilarPaths(int /*nearestK*/, const base::State *start, const base::State *goal,
                                               CandidateSolution &candidateSolution,
                                               const base::PlannerTerminationCondition &ptc)
{
    // TODO: nearestK unused

    // Get neighbors near start and goal. Note: potentially they are not *visible* - will test for this later

    // Start
    OMPL_INFORM("Looking for a node near the problem start");
    if (!findGraphNeighbors(start, startVertexCandidateNeighbors_))
    {
        OMPL_INFORM("No graph neighbors found for start within radius %f", sparseDelta_);
        return false;
    }
    if (verbose_)
        OMPL_INFORM("Found %d nodes near start", startVertexCandidateNeighbors_.size());

    // Goal
    OMPL_INFORM("Looking for a node near the problem goal");
    if (!findGraphNeighbors(goal, goalVertexCandidateNeighbors_))
    {
        OMPL_INFORM("No graph neighbors found for goal within radius %f", sparseDelta_);
        return false;
    }
    if (verbose_)
        OMPL_INFORM("Found %d nodes near goal", goalVertexCandidateNeighbors_.size());

    // Get paths between start and goal
    bool result =
        getPaths(startVertexCandidateNeighbors_, goalVertexCandidateNeighbors_, start, goal, candidateSolution, ptc);

    // Error check
    if (!result)
    {
        OMPL_INFORM("getSimilarPaths(): SPARSdb returned FALSE for getPaths");
        return false;
    }
    if (!candidateSolution.path_)
    {
        OMPL_ERROR("getSimilarPaths(): SPARSdb returned solution is nullptr");
        return false;
    }

    // Debug output
    if (false)
    {
        ompl::geometric::PathGeometric geometricSolution =
            static_cast<ompl::geometric::PathGeometric &>(*candidateSolution.path_);

        for (std::size_t i = 0; i < geometricSolution.getStateCount(); ++i)
        {
            OMPL_INFORM("  getSimilarPaths(): Adding state %f to plannerData", i);
            si_->printState(geometricSolution.getState(i), std::cout);
        }
    }

    return result;
}

bool ompl::geometric::SPARSdb::getPaths(const std::vector<Vertex> &candidateStarts,
                                        const std::vector<Vertex> &candidateGoals, const base::State *actualStart,
                                        const base::State *actualGoal, CandidateSolution &candidateSolution,
                                        const base::PlannerTerminationCondition &ptc)
{
    // Try every combination of nearby start and goal pairs
    foreach (Vertex start, candidateStarts)
    {
        // Check if this start is visible from the actual start
        if (!si_->checkMotion(actualStart, stateProperty_[start]))
        {
            if (verbose_)
                OMPL_WARN("FOUND CANDIDATE START THAT IS NOT VISIBLE ");
            continue;  // this is actually not visible
        }

        foreach (Vertex goal, candidateGoals)
        {
            if (verbose_)
                OMPL_INFORM("  foreach_goal: Checking motion from  %d to %d", actualGoal, stateProperty_[goal]);

            // Check if our planner is out of time
            if (ptc == true)
            {
                OMPL_DEBUG("getPaths function interrupted because termination condition is true.");
                return false;
            }

            // Check if this goal is visible from the actual goal
            if (!si_->checkMotion(actualGoal, stateProperty_[goal]))
            {
                if (verbose_)
                    OMPL_INFORM("FOUND CANDIDATE GOAL THAT IS NOT VISIBLE! ");
                continue;  // this is actually not visible
            }

            // Repeatidly search through graph for connection then check for collisions then repeat
            if (lazyCollisionSearch(start, goal, actualStart, actualGoal, candidateSolution, ptc))
            {
                // Found a path
                return true;
            }
            else
            {
                // Did not find a path
                OMPL_INFORM("Did not find a path, looking for other start/goal combinations ");
            }

        }  // foreach
    }      // foreach

    return false;
}

bool ompl::geometric::SPARSdb::lazyCollisionSearch(const Vertex &start, const Vertex &goal,
                                                   const base::State *actualStart, const base::State *actualGoal,
                                                   CandidateSolution &candidateSolution,
                                                   const base::PlannerTerminationCondition &ptc)
{
    base::Goal *g = pdef_->getGoal().get();  // for checking isStartGoalPairValid

    // Vector to store candidate paths in before they are converted to PathPtrs
    std::vector<Vertex> vertexPath;

    // decide if start and goal are connected
    // TODO this does not compute dynamic graphcs
    // i.e. it will say its the same components even when an edge has been disabled
    bool same_component =
        true;  // sameComponent(start, goal); // TODO is this important? I disabled it during dev and never used it

    // Check if the chosen start and goal can be used together to satisfy problem
    if (!same_component)
    {
        if (verbose_)
            OMPL_INFORM("    Goal and start are not part of same component, skipping ");
        return false;
    }

    // TODO: remove this because start and goal are not either start nor goals
    if (!g->isStartGoalPairValid(stateProperty_[goal], stateProperty_[start]))
    {
        if (verbose_)
            OMPL_INFORM("    Start and goal pair are not valid combinations, skipping ");
        return false;
    }

    // Make sure that the start and goal aren't so close together that they find the same vertex
    if (start == goal)
    {
        if (verbose_)
            OMPL_INFORM("    Start equals goal, skipping ");
        return false;
    }

    // Keep looking for paths between chosen start and goal until one is found that is valid,
    // or no further paths can be found between them because of disabled edges
    // this is necessary for lazy collision checking i.e. rerun after marking invalid edges we found
    bool havePartialSolution = false;
    while (true)
    {
        if (verbose_)
            OMPL_INFORM("      while true: look for valid paths between start and goal");

        // Check if our planner is out of time
        if (ptc == true)
        {
            OMPL_DEBUG("lazyCollisionSearch: function interrupted because termination condition is true.");
            return false;
        }

        // Attempt to find a solution from start to goal
        if (!constructSolution(start, goal, vertexPath))
        {
            // We will stop looking through this start-goal combination, but perhaps this partial solution is good
            if (verbose_)
                OMPL_INFORM("        unable to construct solution between start and goal using astar");

            // no solution path found. check if a previous partially correct solution was found
            if (havePartialSolution && false)  // TODO: re-implement partial solution logic
            {
                if (verbose_)
                    OMPL_INFORM("has partial solution ");
                // Save this candidateSolution for later
                convertVertexPathToStatePath(vertexPath, actualStart, actualGoal, candidateSolution);
                return false;
            }

            if (verbose_)
                OMPL_INFORM("        no partial solution found on this astar search, keep looking through start-goal "
                            "combos");

            // no path found what so ever
            // return false;
            return false;
        }
        havePartialSolution = true;  // we have found at least one path at this point. may be invalid

        if (verbose_)
        {
            OMPL_INFORM("        has at least a partial solution, maybe exact solution");
            OMPL_INFORM("        Solution has %d vertices", vertexPath.size());
        }

        // Check if all the points in the potential solution are valid
        if (lazyCollisionCheck(vertexPath, ptc))
        {
            if (verbose_)
            {
                OMPL_INFORM("---------- lazy collision check returned valid ");
            }

            // the path is valid, we are done!
            convertVertexPathToStatePath(vertexPath, actualStart, actualGoal, candidateSolution);
            return true;
        }
        // else, loop with updated graph that has the invalid edges/states disabled
    }  // while

    // we never found a valid path
    return false;
}

bool ompl::geometric::SPARSdb::constructSolution(const Vertex start, const Vertex goal,
                                                 std::vector<Vertex> &vertexPath) const
{
    auto *vertexPredecessors = new Vertex[boost::num_vertices(g_)];
    bool foundGoal = false;

    auto *vertexDistances = new double[boost::num_vertices(g_)];

    try
    {
        boost::astar_search(g_,     // graph
                            start,  // start state
                            [this, goal](const Vertex v)
                            {
                                return distanceFunction(v, goal);
                            },  // the heuristic
                            // ability to disable edges (set cost to inifinity):
                            boost::weight_map(edgeWeightMap(g_, edgeCollisionStateProperty_))
                                .predecessor_map(vertexPredecessors)
                                .distance_map(&vertexDistances[0])
                                .visitor(CustomVisitor(goal)));
    }
    catch (ompl::geometric::SPARSdb::foundGoalException &)
    {
        // the custom exception from CustomVisitor
        if (verbose_ && false)
        {
            OMPL_INFORM("constructSolution: Astar found goal vertex ------------------------");
            OMPL_INFORM("distance to goal: %f", vertexDistances[goal]);
        }

        if (vertexDistances[goal] > 1.7e+308)  // terrible hack for detecting infinity
        // double diff = d[goal] - std::numeric_limits<double>::infinity();
        // if ((diff < std::numeric_limits<double>::epsilon()) && (-diff < std::numeric_limits<double>::epsilon()))
        // check if the distance to goal is inifinity. if so, it is unreachable
        // if (d[goal] >= std::numeric_limits<double>::infinity())
        {
            if (verbose_)
                OMPL_INFORM("Distance to goal is infinity");
            foundGoal = false;
        }
        else
        {
            // Only clear the vertexPath after we know we have a new solution, otherwise it might have a good
            // previous one
            vertexPath.clear();  // remove any old solutions

            // Trace back a shortest path in reverse and only save the states
            Vertex v;
            for (v = goal; v != vertexPredecessors[v]; v = vertexPredecessors[v])
            {
                vertexPath.push_back(v);
            }
            if (v != goal)  // TODO explain this because i don't understand
            {
                vertexPath.push_back(v);
            }

            foundGoal = true;
        }
    }

    delete[] vertexPredecessors;
    delete[] vertexDistances;

    // No solution found from start to goal
    return foundGoal;
}

bool ompl::geometric::SPARSdb::lazyCollisionCheck(std::vector<Vertex> &vertexPath,
                                                  const base::PlannerTerminationCondition &ptc)
{
    OMPL_DEBUG("Starting lazy collision checking");

    bool hasInvalidEdges = false;

    // Initialize
    Vertex fromVertex = vertexPath[0];
    Vertex toVertex;

    // Loop through every pair of states and make sure path is valid.
    for (std::size_t toID = 1; toID < vertexPath.size(); ++toID)
    {
        // Increment location on path
        toVertex = vertexPath[toID];

        // Check if our planner is out of time
        if (ptc == true)
        {
            OMPL_DEBUG("Lazy collision check function interrupted because termination condition is true.");
            return false;
        }

        Edge thisEdge = boost::edge(fromVertex, toVertex, g_).first;

        // Has this edge already been checked before?
        if (edgeCollisionStateProperty_[thisEdge] == NOT_CHECKED)
        {
            // Check path between states
            if (!si_->checkMotion(stateProperty_[fromVertex], stateProperty_[toVertex]))
            {
                // Path between (from, to) states not valid, disable the edge
                OMPL_INFORM("  DISABLING EDGE from vertex %f to vertex %f", fromVertex, toVertex);

                // Disable edge
                edgeCollisionStateProperty_[thisEdge] = IN_COLLISION;
            }
            else
            {
                // Mark edge as free so we no longer need to check for collision
                edgeCollisionStateProperty_[thisEdge] = FREE;
            }
        }

        // Check final result
        if (edgeCollisionStateProperty_[thisEdge] == IN_COLLISION)
        {
            // Remember that this path is no longer valid, but keep checking remainder of path edges
            hasInvalidEdges = true;
        }

        // switch vertex focus
        fromVertex = toVertex;
    }

    OMPL_INFORM("Done lazy collision checking");

    // TODO: somewhere in the code we need to reset all edges collision status back to NOT_CHECKED for future queries

    // Only return true if nothing was found invalid
    return !hasInvalidEdges;
}

bool ompl::geometric::SPARSdb::sameComponent(Vertex m1, Vertex m2)
{
    return boost::same_component(m1, m2, disjointSets_);
}

bool ompl::geometric::SPARSdb::reachedFailureLimit() const
{
    return consecutiveFailures_ >= maxFailures_;
}

void ompl::geometric::SPARSdb::printDebug(std::ostream &out) const
{
    out << "SPARSdb Debug Output: " << std::endl;
    out << "  Settings: " << std::endl;
    out << "    Max Failures: " << getMaxFailures() << std::endl;
    out << "    Dense Delta Fraction: " << getDenseDeltaFraction() << std::endl;
    out << "    Sparse Delta Fraction: " << getSparseDeltaFraction() << std::endl;
    out << "    Sparse Delta: " << sparseDelta_ << std::endl;
    out << "    Stretch Factor: " << getStretchFactor() << std::endl;
    out << "    Maximum Extent: " << si_->getMaximumExtent() << std::endl;
    out << "  Status: " << std::endl;
    out << "    Vertices Count: " << getNumVertices() << std::endl;
    out << "    Edges Count:    " << getNumEdges() << std::endl;
    out << "    Iterations: " << getIterations() << std::endl;
    out << "    Consecutive Failures: " << consecutiveFailures_ << std::endl;
    out << "    Number of guards: " << nn_->size() << std::endl << std::endl;
}

bool ompl::geometric::SPARSdb::getGuardSpacingFactor(const double pathLength, int &numGuards, double &spacingFactor)
{
    static const double factorHigh = 1.9;
    static const double factorLow = 1.1;
    double minPathLength = sparseDelta_ * factorLow;

    // Check if the path length is too short
    if (pathLength < minPathLength)
    {
        OMPL_INFORM("Path length is too short to get a correct sparcing factor: length: %f, min: %f ", pathLength,
                    minPathLength);
        spacingFactor = factorLow;
        return true;  // still attempt
    }

    // Get initial guess using med value
    double numGuardsFraction = pathLength / (sparseDelta_ * factorLow);
    if (verbose_)
    {
        OMPL_INFORM("getGuardSpacingFactor: ");
        OMPL_INFORM("  pathLength: %f", pathLength);
        OMPL_INFORM("  sparseDelta: %f", sparseDelta_);
        OMPL_INFORM("  min pathLength: %f", minPathLength);
        OMPL_INFORM("  numGuardsFraction: %f", numGuardsFraction);
    }

    // Round down to nearest integer
    numGuards = numGuardsFraction;

    static std::size_t MAX_ATTEMPTS = 4;
    for (std::size_t i = 0; i < MAX_ATTEMPTS; ++i)
    {
        if (verbose_)
            OMPL_INFORM("  numGuards: %d", numGuards);

        // Find the factor to achieve this number of guards
        spacingFactor = pathLength / (sparseDelta_ * numGuards);
        if (verbose_)
            OMPL_INFORM("  new spacingFactor: %f", spacingFactor);

        // Check if this factor is too low
        if (spacingFactor < factorLow)
        {
            if (verbose_)
                OMPL_INFORM("  spacing factor is too low ");
            numGuards++;
            continue;
        }
        else if (spacingFactor > factorHigh)
        {
            if (verbose_)
                OMPL_INFORM("  spacing factor is too high ");
            numGuards--;
            continue;
        }
        else
            return true;  // a good value
    }

    OMPL_ERROR("Unable to find correct spacing factor - perhaps this is a bug");
    spacingFactor = factorLow;
    return true;  // still attempt
}

bool ompl::geometric::SPARSdb::addPathToRoadmap(const base::PlannerTerminationCondition &ptc,
                                                ompl::geometric::PathGeometric &solutionPath)
{
    // Check that the query vertex is initialized (used for internal nearest neighbor searches)
    checkQueryStateInitialization();

    // Error check
    if (solutionPath.getStateCount() < 2)
    {
        OMPL_ERROR("Less than 2 states were passed to addPathToRoadmap in the solution path");
        return false;
    }

    // Find spacing factor - 2.0 would be a perfect amount, but we leave room for rounding/interpolation errors and
    // curves in path
    int numGuards;  // unused variable that indicates how many guards we will add
    double spacingFactor;
    if (!getGuardSpacingFactor(solutionPath.length(), numGuards, spacingFactor))
        return false;

    OMPL_DEBUG("Expected number of necessary coverage guards is calculated to be %i from the original path state count "
               "%i",
               numGuards, solutionPath.getStateCount());

    unsigned int n = 0;
    const int n1 = solutionPath.getStateCount() - 1;
    for (int i = 0; i < n1; ++i)
        n += si_->getStateSpace()->validSegmentCount(solutionPath.getState(i), solutionPath.getState(i + 1));

    solutionPath.interpolate(n);

    // Debug
    if (verbose_)
    {
        OMPL_INFORM("-------------------------------------------------------");
        OMPL_INFORM("Attempting to add %d states to roadmap", solutionPath.getStateCount());
        OMPL_INFORM("-------------------------------------------------------");
    }

    // Try to add the start first, but don't force it
    addStateToRoadmap(ptc, solutionPath.getState(0));

#ifdef OMPL_THUNDER_DEBUG
    visualizeStateCallback(solutionPath.getState(solutionPath.getStateCount() - 1), 3, sparseDelta_);
#endif

    // Add solution states to SPARSdb one by one ---------------------------

    // Track which nodes we've already tried to add
    std::vector<std::size_t> addedStateIDs;
    // Track which nodes we will attempt to use as connectivity states
    std::vector<std::size_t> connectivityStateIDs;
    // std::vector<base::State*> connectivityStates;

    double distanceFromLastState = 0;

    std::size_t lastStateID = 0;  // track the id in the solutionPath of the last state

    for (std::size_t i = 1; i < solutionPath.getStateCount();
         ++i)  // skip 0 and last because those are start/goal and are already added
    {
        distanceFromLastState = si_->distance(solutionPath.getState(i), solutionPath.getState(lastStateID));

        if (verbose_ && false)
        {
            OMPL_INFORM("Index %d at distance %f from last state ", i, distanceFromLastState);
        }

        if (distanceFromLastState >= sparseDelta_ * spacingFactor)
        {
            if (verbose_)
            {
                OMPL_INFORM("Adding state %d of %d", i, solutionPath.getStateCount());
            }

// Show the candidate state in Rviz for path insertion of GUARDS
#ifdef OMPL_THUNDER_DEBUG
            visualizeStateCallback(solutionPath.getState(i), 1, sparseDelta_);
#endif

            // Add a single state to the roadmap
            if (!addStateToRoadmap(ptc, solutionPath.getState(i)))
            {
                if (verbose_)
                {
                    OMPL_INFORM("Last state added to roadmap failed ");
                }
            }

            // Now figure out midpoint state between lastState and i
            std::size_t midStateID = (i - lastStateID) / 2 + lastStateID;
            connectivityStateIDs.push_back(midStateID);

            double distA = si_->distance(solutionPath.getState(lastStateID), solutionPath.getState(midStateID));
            double distB = si_->distance(solutionPath.getState(i), solutionPath.getState(midStateID));
            double diff = distA - distB;
            if ((diff < std::numeric_limits<double>::epsilon()) && (-diff < std::numeric_limits<double>::epsilon()))
                if (verbose_)
                    OMPL_WARN("DISTANCES ARE DIFFERENT ");

            // Save this state as the new last state
            lastStateID = i;
            // Remember which nodes we've already added / attempted to add
            addedStateIDs.push_back(midStateID);
            addedStateIDs.push_back(i);
        }
        // Close up if it doesn't do it automatically
        else if (i == solutionPath.getStateCount() - 1)
        {
            if (verbose_)
                OMPL_INFORM("Last state - do special midpoint");

            // Now figure out midpoint state between lastState and i
            std::size_t midStateID = (i - lastStateID) / 2 + lastStateID;
            connectivityStateIDs.push_back(midStateID);
            addedStateIDs.push_back(midStateID);
            if (verbose_)
                OMPL_INFORM("Mid state is %d", midStateID);
        }
    }

    // Attempt to add the goal directly
    addStateToRoadmap(ptc, solutionPath.getState(solutionPath.getStateCount() - 1));

    if (verbose_)
    {
        OMPL_INFORM("-------------------------------------------------------");
        OMPL_INFORM("-------------------------------------------------------");
        OMPL_INFORM("Adding connectivity states ----------------------------");
        OMPL_INFORM("-------------------------------------------------------");
        OMPL_INFORM("-------------------------------------------------------");
    }

    for (std::size_t i = 0; i < connectivityStateIDs.size(); ++i)
    {
        base::State *connectivityState = solutionPath.getState(connectivityStateIDs[i]);

        if (verbose_)
        {
            OMPL_INFORM("Adding connectvity state ", i);
        }

#ifdef OMPL_THUNDER_DEBUG
        // Show the candidate state in Rviz for path insertion of BRIDGES (CONNECTIVITY)
        visualizeStateCallback(connectivityState, 2, sparseDelta_);
        sleep(0.5);
#endif

        // Add a single state to the roadmap
        addStateToRoadmap(ptc, connectivityState);
    }

    // Add remaining states at random
    if (verbose_)
    {
        OMPL_INFORM("-------------------------------------------------------");
        OMPL_INFORM("-------------------------------------------------------");
        OMPL_INFORM("Adding remaining states randomly ----------------------");
        OMPL_INFORM("-------------------------------------------------------");
        OMPL_INFORM("-------------------------------------------------------");
    }

    // Create a vector of shuffled indexes
    std::vector<std::size_t> shuffledIDs;
    std::size_t usedIDTracker = 0;
    for (std::size_t i = 1; i < solutionPath.getStateCount(); ++i)  // skip 0 because start already added
    {
        // Check if we've already used this id
        if (usedIDTracker < addedStateIDs.size() && i == addedStateIDs[usedIDTracker])
        {
            // skip this id
            usedIDTracker++;
            continue;
        }

        shuffledIDs.push_back(i);  // 1 2 3...
    }

    std::shuffle(shuffledIDs.begin(), shuffledIDs.end(), std::mt19937(std::random_device()()));

    // Add each state randomly
    for (unsigned long shuffledID : shuffledIDs)
    {
#ifdef OMPL_THUNDER_DEBUG
        visualizeStateCallback(solutionPath.getState(shuffledIDs[i]), 1, sparseDelta_);
#endif

        // Add a single state to the roadmap
        addStateToRoadmap(ptc, solutionPath.getState(shuffledID));
    }

    bool benchmarkLogging = true;
    if (benchmarkLogging)
    {
        OMPL_DEBUG("ompl::geometric::SPARSdb: Benchmark logging enabled (slower)");

        // Return the result of inserting into database, if applicable
        return checkStartGoalConnection(solutionPath);
    }

    return true;
}

bool ompl::geometric::SPARSdb::checkStartGoalConnection(ompl::geometric::PathGeometric &solutionPath)
{
    // Make sure path has states
    if (solutionPath.getStateCount() < 2)
    {
        OMPL_ERROR("Not enough states (< 2) in the solutionPath");
        return false;
    }

    bool error = false;
    CandidateSolution candidateSolution;
    do
    {
        base::State *actualStart = solutionPath.getState(0);
        base::State *actualGoal = solutionPath.getState(solutionPath.getStateCount() - 1);

        /* The whole neighborhood set which has been most recently computed */
        std::vector<Vertex> graphNeighborhood;
        /* The visible neighborhood set which has been most recently computed */
        std::vector<Vertex> visibleNeighborhood;

        // Get start vertex
        findGraphNeighbors(actualStart, graphNeighborhood, visibleNeighborhood);
        if (!visibleNeighborhood.size())
        {
            OMPL_ERROR("No vertexes found near start");
            error = true;
            break;
        }
        Vertex closeStart = visibleNeighborhood[0];

        // Get goal vertex
        findGraphNeighbors(actualGoal, graphNeighborhood, visibleNeighborhood);
        if (!visibleNeighborhood.size())
        {
            OMPL_ERROR("No vertexes found near goal");
            error = true;
            break;
        }
        Vertex closeGoal = visibleNeighborhood[0];

        // Check if connected
        if (false)
            if (!sameComponent(closeStart, closeGoal))
            {
                OMPL_ERROR("Start and goal are not connected!");
                error = true;
                break;
            }

        // Get new path from start to goal
        std::vector<Vertex> vertexPath;
        if (!constructSolution(closeStart, closeGoal, vertexPath))
        {
            OMPL_ERROR("Unable to find path from start to goal - perhaps because of new obstacles");
            error = true;
            break;
        }

        // Convert to PathGeometric
        bool disableCollisionWarning = true;  // this is just for benchmarking purposes
        if (!convertVertexPathToStatePath(vertexPath, actualStart, actualGoal, candidateSolution,
                                          disableCollisionWarning))
        {
            OMPL_ERROR("Unable to convert to state path");
            error = true;
            break;
        }
    } while (false);

    // Check distance of new path from old path
    double originalLength = solutionPath.length();

    OMPL_DEBUG("Results of attempting to make insertion in SPARSdb ");
    OMPL_DEBUG("-------------------------------------------------------");
    OMPL_DEBUG("Original length:    %f", originalLength);

    if (error)
    {
        OMPL_ERROR("UNABLE TO GET PATH");

        // Record this for plotting
        numPathInsertionFailures_++;
    }
    else
    {
        double newLength = candidateSolution.getGeometricPath().length();
        double percentIncrease = 100 - originalLength / newLength * 100;
        OMPL_DEBUG("New length:        %f", newLength);
        OMPL_DEBUG("Percent increase:  %f %%", percentIncrease);
    }

    return !error;  // return true if it inserted correctly
}

bool ompl::geometric::SPARSdb::addStateToRoadmap(const base::PlannerTerminationCondition &ptc, base::State *newState)
{
    bool stateAdded = false;
    // Check that the query vertex is initialized (used for internal nearest neighbor searches)
    checkQueryStateInitialization();

    // Deep copy
    base::State *qNew = si_->cloneState(newState);
    base::State *workState = si_->allocState();

    /* The whole neighborhood set which has been most recently computed */
    std::vector<Vertex> graphNeighborhood;
    /* The visible neighborhood set which has been most recently computed */
    std::vector<Vertex> visibleNeighborhood;

    ++iterations_;

    findGraphNeighbors(qNew, graphNeighborhood, visibleNeighborhood);

    if (verbose_)
    {
        OMPL_INFORM(" graph neighborhood: %d | visible neighborhood: %d", graphNeighborhood.size(),
                    visibleNeighborhood.size());

        foreach (Vertex v, visibleNeighborhood)
        {
            OMPL_INFORM("Visible neighbor is vertex %f with distance %f ", v, si_->distance(qNew, stateProperty_[v]));
        }
    }

    if (verbose_)
        OMPL_INFORM(" - checkAddCoverage() Are other nodes around it visible?");
    // Coverage criterion
    if (!checkAddCoverage(qNew,
                          visibleNeighborhood))  // Always add a node if no other nodes around it are visible (GUARD)
    {
        if (verbose_)
            OMPL_INFORM(" -- checkAddConnectivity() Does this node connect neighboring nodes that are not connected? ");
        // Connectivity criterion
        if (!checkAddConnectivity(qNew, visibleNeighborhood))
        {
            if (verbose_)
                OMPL_INFORM(" --- checkAddInterface() Does this node's neighbor's need it to better connect them? ");
            if (!checkAddInterface(qNew, graphNeighborhood, visibleNeighborhood))
            {
                if (verbose_)
                    OMPL_INFORM(" ---- Ensure SPARS asymptotic optimality");
                if (visibleNeighborhood.size() > 0)
                {
                    std::map<Vertex, base::State *> closeRepresentatives;
                    if (verbose_)
                        OMPL_INFORM(" ----- findCloseRepresentatives()");

                    findCloseRepresentatives(workState, qNew, visibleNeighborhood[0], closeRepresentatives, ptc);
                    if (verbose_)
                        OMPL_INFORM("------ Found %d close representatives", closeRepresentatives.size());

                    for (auto &closeRepresentative : closeRepresentatives)
                    {
                        if (verbose_)
                            OMPL_INFORM(" ------ Looping through close representatives");
                        updatePairPoints(visibleNeighborhood[0], qNew, closeRepresentative.first,
                                         closeRepresentative.second);
                        updatePairPoints(closeRepresentative.first, closeRepresentative.second, visibleNeighborhood[0],
                                         qNew);
                    }
                    if (verbose_)
                        OMPL_INFORM(" ------ checkAddPath()");
                    if (checkAddPath(visibleNeighborhood[0]))
                    {
                        if (verbose_)
                        {
                            OMPL_INFORM("nearest visible neighbor added ");
                        }
                    }

                    for (auto &closeRepresentative : closeRepresentatives)
                    {
                        if (verbose_)
                            OMPL_INFORM(" ------- Looping through close representatives to add path");
                        checkAddPath(closeRepresentative.first);
                        si_->freeState(closeRepresentative.second);
                    }
                    if (verbose_)
                        OMPL_INFORM("------ Done with inner most loop ");
                }
            }
            else  //  added for interface
            {
                stateAdded = true;
            }
        }
        else  // added for connectivity
        {
            stateAdded = true;
        }
    }
    else  // added for coverage
    {
        stateAdded = true;
    }

    if (!stateAdded)
        ++consecutiveFailures_;

    si_->freeState(workState);
    si_->freeState(qNew);

    return stateAdded;
}

void ompl::geometric::SPARSdb::checkQueryStateInitialization()
{
    if (boost::num_vertices(g_) < 1)
    {
        queryVertex_ = boost::add_vertex(g_);
        stateProperty_[queryVertex_] = nullptr;
    }
}

ompl::base::PlannerStatus ompl::geometric::SPARSdb::solve(const base::PlannerTerminationCondition &)
{
    // Disabled
    return base::PlannerStatus::TIMEOUT;
}

bool ompl::geometric::SPARSdb::checkAddCoverage(const base::State *qNew, std::vector<Vertex> &visibleNeighborhood)
{
    if (visibleNeighborhood.size() > 0)
        return false;
    // No free paths means we add for coverage
    if (verbose_)
        OMPL_INFORM(" --- Adding node for COVERAGE ");
    Vertex v = addGuard(si_->cloneState(qNew), COVERAGE);
    if (verbose_)
        OMPL_INFORM("       Added vertex %f", v);

    return true;
}

bool ompl::geometric::SPARSdb::checkAddConnectivity(const base::State *qNew, std::vector<Vertex> &visibleNeighborhood)
{
    // Identify visibile nodes around our new state that are unconnected (in different connected components)
    // and connect them

    std::vector<Vertex> statesInDiffConnectedComponents;  // links
    if (visibleNeighborhood.size() >
        1)  // if less than 2 there is no way to find a pair of nodes in different connected components
    {
        // For each neighbor
        for (std::size_t i = 0; i < visibleNeighborhood.size(); ++i)
        {
            // For each other neighbor
            for (std::size_t j = i + 1; j < visibleNeighborhood.size(); ++j)
            {
                // If they are in different components
                if (!sameComponent(visibleNeighborhood[i], visibleNeighborhood[j]))
                {
                    statesInDiffConnectedComponents.push_back(visibleNeighborhood[i]);
                    statesInDiffConnectedComponents.push_back(visibleNeighborhood[j]);
                }
            }
        }

        // Were any diconnected states found?
        if (statesInDiffConnectedComponents.size() > 0)
        {
            if (verbose_)
                OMPL_INFORM(" --- Adding node for CONNECTIVITY ");
            // Add the node
            Vertex newVertex = addGuard(si_->cloneState(qNew), CONNECTIVITY);

            for (unsigned long statesInDiffConnectedComponent : statesInDiffConnectedComponents)
            {
                // If there's no edge between the two new states
                // DTC: this should actually never happen - we just created the new vertex so
                // why would it be connected to anything?
                if (!boost::edge(newVertex, statesInDiffConnectedComponent, g_).second)
                {
                    // The components haven't been united by previous links
                    if (!sameComponent(statesInDiffConnectedComponent, newVertex))
                        connectGuards(newVertex, statesInDiffConnectedComponent);
                }
            }

            return true;
        }
    }
    return false;
}

bool ompl::geometric::SPARSdb::checkAddInterface(const base::State *qNew, std::vector<Vertex> &graphNeighborhood,
                                                 std::vector<Vertex> &visibleNeighborhood)
{
    // If we have at least 2 neighbors
    if (visibleNeighborhood.size() > 1)
    {
        // If the two closest nodes are also visible
        if (graphNeighborhood[0] == visibleNeighborhood[0] && graphNeighborhood[1] == visibleNeighborhood[1])
        {
            // If our two closest neighbors don't share an edge
            if (!boost::edge(visibleNeighborhood[0], visibleNeighborhood[1], g_).second)
            {
                // If they can be directly connected
                if (si_->checkMotion(stateProperty_[visibleNeighborhood[0]], stateProperty_[visibleNeighborhood[1]]))
                {
                    // Connect them
                    if (verbose_)
                        OMPL_INFORM(" ---   INTERFACE: directly connected nodes ");
                    connectGuards(visibleNeighborhood[0], visibleNeighborhood[1]);
                    // And report that we added to the roadmap
                    resetFailures();
                    // Report success
                    return true;
                }
                else
                {
                    // Add the new node to the graph, to bridge the interface
                    if (verbose_)
                        OMPL_INFORM(" --- Adding node for INTERFACE  ");
                    Vertex v = addGuard(si_->cloneState(qNew), INTERFACE);
                    connectGuards(v, visibleNeighborhood[0]);
                    connectGuards(v, visibleNeighborhood[1]);
                    if (verbose_)
                        OMPL_INFORM(" ---   INTERFACE: connected two neighbors through new interface node ");
                    // Report success
                    return true;
                }
            }
        }
    }
    return false;
}

bool ompl::geometric::SPARSdb::checkAddPath(Vertex v)
{
    bool spannerPropertyWasViolated = false;

    std::vector<Vertex> rs;
    foreach (Vertex r, boost::adjacent_vertices(v, g_))
        rs.push_back(r);

    /* Candidate x vertices as described in the method, filled by function computeX(). */
    std::vector<Vertex> Xs;

    /* Candidate v" vertices as described in the method, filled by function computeVPP(). */
    std::vector<Vertex> VPPs;

    for (std::size_t i = 0; i < rs.size() && !spannerPropertyWasViolated; ++i)
    {
        Vertex r = rs[i];
        computeVPP(v, r, VPPs);
        foreach (Vertex rp, VPPs)
        {
            // First, compute the longest path through the graph
            computeX(v, r, rp, Xs);
            double rm_dist = 0.0;
            foreach (Vertex rpp, Xs)
            {
                double tmp_dist = (si_->distance(stateProperty_[r], stateProperty_[v]) +
                                   si_->distance(stateProperty_[v], stateProperty_[rpp])) /
                                  2.0;
                if (tmp_dist > rm_dist)
                    rm_dist = tmp_dist;
            }

            InterfaceData &d = getData(v, r, rp);

            // Then, if the spanner property is violated
            if (rm_dist > stretchFactor_ * d.d_)
            {
                spannerPropertyWasViolated = true;  // Report that we added for the path
                if (si_->checkMotion(stateProperty_[r], stateProperty_[rp]))
                    connectGuards(r, rp);
                else
                {
                    auto p(std::make_shared<PathGeometric>(si_));
                    if (r < rp)
                    {
                        p->append(d.sigmaA_);
                        p->append(d.pointA_);
                        p->append(stateProperty_[v]);
                        p->append(d.pointB_);
                        p->append(d.sigmaB_);
                    }
                    else
                    {
                        p->append(d.sigmaB_);
                        p->append(d.pointB_);
                        p->append(stateProperty_[v]);
                        p->append(d.pointA_);
                        p->append(d.sigmaA_);
                    }

                    psimp_->reduceVertices(*p, 10);
                    psimp_->shortcutPath(*p, 50);

                    if (p->checkAndRepair(100).second)
                    {
                        Vertex prior = r;
                        Vertex vnew;
                        std::vector<base::State *> &states = p->getStates();

                        foreach (base::State *st, states)
                        {
                            // no need to clone st, since we will destroy p; we just copy the pointer
                            if (verbose_)
                                OMPL_INFORM(" --- Adding node for QUALITY");
                            vnew = addGuard(st, QUALITY);

                            connectGuards(prior, vnew);
                            prior = vnew;
                        }
                        // clear the states, so memory is not freed twice
                        states.clear();
                        connectGuards(prior, rp);
                    }
                }
            }
        }
    }

    if (!spannerPropertyWasViolated)
    {
        if (verbose_)
        {
            OMPL_INFORM(" ------- Spanner property was NOT violated, SKIPPING");
        }
    }

    return spannerPropertyWasViolated;
}

void ompl::geometric::SPARSdb::resetFailures()
{
    consecutiveFailures_ = 0;
}

void ompl::geometric::SPARSdb::findGraphNeighbors(base::State *st, std::vector<Vertex> &graphNeighborhood,
                                                  std::vector<Vertex> &visibleNeighborhood)
{
    visibleNeighborhood.clear();
    stateProperty_[queryVertex_] = st;
    nn_->nearestR(queryVertex_, sparseDelta_, graphNeighborhood);
    if (verbose_ && false)
        OMPL_INFORM("Finding nearest nodes in NN tree within radius %f", sparseDelta_);
    stateProperty_[queryVertex_] = nullptr;

    // Now that we got the neighbors from the NN, we must remove any we can't see
    for (unsigned long i : graphNeighborhood)
        if (si_->checkMotion(st, stateProperty_[i]))
            visibleNeighborhood.push_back(i);
}

bool ompl::geometric::SPARSdb::findGraphNeighbors(const base::State *state, std::vector<Vertex> &graphNeighborhood)
{
    base::State *stateCopy = si_->cloneState(state);

    // Don't check for visibility
    graphNeighborhood.clear();
    stateProperty_[queryVertex_] = stateCopy;

    // Double the range of sparseDelta_ up to 3 times until at least 1 neighbor is found
    std::size_t expandNeighborhoodSearchAttempts = 3;
    double neighborSearchRadius;
    static const double EXPAND_NEIGHBORHOOD_RATE =
        0.25;  // speed to which we look outside the original sparse delta neighborhood
    for (std::size_t i = 0; i < expandNeighborhoodSearchAttempts; ++i)
    {
        neighborSearchRadius = sparseDelta_ + i * EXPAND_NEIGHBORHOOD_RATE * sparseDelta_;
        if (verbose_)
        {
            OMPL_INFORM("-------------------------------------------------------");
            OMPL_INFORM("Attempt %d to find neighborhood at radius %f", i + 1, neighborSearchRadius);
            OMPL_INFORM("-------------------------------------------------------");
        }

        nn_->nearestR(queryVertex_, neighborSearchRadius, graphNeighborhood);

        // Check if at least one neighbor found
        if (graphNeighborhood.size() > 0)
            break;
    }
    stateProperty_[queryVertex_] = nullptr;

    // Check if no neighbors found
    if (!graphNeighborhood.size())
    {
        return false;
    }
    return true;
}

void ompl::geometric::SPARSdb::approachGraph(Vertex v)
{
    std::vector<Vertex> hold;
    nn_->nearestR(v, sparseDelta_, hold);

    std::vector<Vertex> neigh;
    for (unsigned long i : hold)
        if (si_->checkMotion(stateProperty_[v], stateProperty_[i]))
            neigh.push_back(i);

    foreach (Vertex vp, neigh)
        connectGuards(v, vp);
}

ompl::geometric::SPARSdb::Vertex ompl::geometric::SPARSdb::findGraphRepresentative(base::State *st)
{
    std::vector<Vertex> nbh;
    stateProperty_[queryVertex_] = st;
    nn_->nearestR(queryVertex_, sparseDelta_, nbh);
    stateProperty_[queryVertex_] = nullptr;

    if (verbose_)
        OMPL_INFORM(" ------- findGraphRepresentative found %d nearest neighbors of distance %f", nbh.size(),
                    sparseDelta_);

    Vertex result = boost::graph_traits<Graph>::null_vertex();

    for (std::size_t i = 0; i < nbh.size(); ++i)
    {
        if (verbose_)
            OMPL_INFORM(" -------- Checking motion of graph rep candidate %d", i);
        if (si_->checkMotion(st, stateProperty_[nbh[i]]))
        {
            if (verbose_)
                OMPL_INFORM(" --------- VALID ");
            result = nbh[i];
            break;
        }
    }
    return result;
}

void ompl::geometric::SPARSdb::findCloseRepresentatives(base::State *workState, const base::State *qNew,
                                                        const Vertex qRep,
                                                        std::map<Vertex, base::State *> &closeRepresentatives,
                                                        const base::PlannerTerminationCondition &ptc)
{
    // Properly clear the vector by also deleting previously sampled unused states
    for (auto &closeRepresentative : closeRepresentatives)
        si_->freeState(closeRepresentative.second);
    closeRepresentatives.clear();

    // denseDelta_ = 0.25 * sparseDelta_;
    nearSamplePoints_ /= 10;  // HACK - this makes it look for the same number of samples as dimensions

    if (verbose_)
        OMPL_INFORM(" ----- nearSamplePoints: %f, denseDelta: %f", nearSamplePoints_, denseDelta_);

    // Then, begin searching the space around new potential state qNew
    for (unsigned int i = 0; i < nearSamplePoints_ && ptc == false; ++i)
    {
        do
        {
            sampler_->sampleNear(workState, qNew, denseDelta_);

#ifdef OMPL_THUNDER_DEBUG
            visualizeStateCallback(workState, 3, sparseDelta_);
            sleep(0.1);
#endif

            if (verbose_)
            {
                OMPL_INFORM(" ------ findCloseRepresentatives sampled state ");

                if (!si_->isValid(workState))
                {
                    OMPL_INFORM(" ------ isValid ");
                }
                if (si_->distance(qNew, workState) > denseDelta_)
                {
                    OMPL_INFORM(" ------ Distance too far ");
                }
                if (!si_->checkMotion(qNew, workState))
                {
                    OMPL_INFORM(" ------ Motion invalid ");
                }
            }

        } while ((!si_->isValid(workState) || si_->distance(qNew, workState) > denseDelta_ ||
                  !si_->checkMotion(qNew, workState)) &&
                 ptc == false);

        // if we were not successful at sampling a desirable state, we are out of time
        if (ptc == true)
        {
            if (verbose_)
                OMPL_INFORM(" ------ We are out of time ");
            break;
        }

        if (verbose_)
            OMPL_INFORM(" ------ Find graph representative ");

        // Compute who his graph neighbors are
        Vertex representative = findGraphRepresentative(workState);

        // Assuming this sample is actually seen by somebody (which he should be in all likelihood)
        if (representative != boost::graph_traits<Graph>::null_vertex())
        {
            if (verbose_)
                OMPL_INFORM(" ------ Representative is not null ");

            // If his representative is different than qNew
            if (qRep != representative)
            {
                if (verbose_)
                    OMPL_INFORM(" ------ qRep != representative ");

                // And we haven't already tracked this representative
                if (closeRepresentatives.find(representative) == closeRepresentatives.end())
                {
                    if (verbose_)
                        OMPL_INFORM(" ------ Track the representative");
                    // Track the representativen
                    closeRepresentatives[representative] = si_->cloneState(workState);
                }
            }
            else
            {
                if (verbose_)
                    OMPL_INFORM(" ------ qRep == representative, no good ");
            }
        }
        else
        {
            if (verbose_)
                OMPL_INFORM(" ------ Rep is null ");

            // This guy can't be seen by anybody, so we should take this opportunity to add him
            if (verbose_)
                OMPL_INFORM(" --- Adding node for COVERAGE");
            addGuard(si_->cloneState(workState), COVERAGE);

            if (verbose_)
            {
                OMPL_INFORM(" ------ STOP EFFORS TO ADD A DENSE PATH");
            }

            // We should also stop our efforts to add a dense path
            for (auto &closeRepresentative : closeRepresentatives)
                si_->freeState(closeRepresentative.second);
            closeRepresentatives.clear();
            break;
        }
    }  // for loop
}

void ompl::geometric::SPARSdb::updatePairPoints(Vertex rep, const base::State *q, Vertex r, const base::State *s)
{
    // First of all, we need to compute all candidate r'
    std::vector<Vertex> VPPs;
    computeVPP(rep, r, VPPs);

    // Then, for each pair Pv(r,r')
    foreach (Vertex rp, VPPs)
        // Try updating the pair info
        distanceCheck(rep, q, r, s, rp);
}

void ompl::geometric::SPARSdb::computeVPP(Vertex v, Vertex vp, std::vector<Vertex> &VPPs)
{
    VPPs.clear();
    foreach (Vertex cvpp, boost::adjacent_vertices(v, g_))
        if (cvpp != vp)
            if (!boost::edge(cvpp, vp, g_).second)
                VPPs.push_back(cvpp);
}

void ompl::geometric::SPARSdb::computeX(Vertex v, Vertex vp, Vertex vpp, std::vector<Vertex> &Xs)
{
    Xs.clear();

    foreach (Vertex cx, boost::adjacent_vertices(vpp, g_))
        if (boost::edge(cx, v, g_).second && !boost::edge(cx, vp, g_).second)
        {
            InterfaceData &d = getData(v, vpp, cx);
            if ((vpp < cx && d.pointA_) || (cx < vpp && d.pointB_))
                Xs.push_back(cx);
        }
    Xs.push_back(vpp);
}

ompl::geometric::SPARSdb::VertexPair ompl::geometric::SPARSdb::index(Vertex vp, Vertex vpp)
{
    if (vp < vpp)
        return VertexPair(vp, vpp);
    else if (vpp < vp)
        return VertexPair(vpp, vp);
    else
        throw Exception(name_, "Trying to get an index where the pairs are the same point!");
}

ompl::geometric::SPARSdb::InterfaceData &ompl::geometric::SPARSdb::getData(Vertex v, Vertex vp, Vertex vpp)
{
    return interfaceDataProperty_[v].interfaceHash[index(vp, vpp)];
}

void ompl::geometric::SPARSdb::distanceCheck(Vertex rep, const base::State *q, Vertex r, const base::State *s,
                                             Vertex rp)
{
    // Get the info for the current representative-neighbors pair
    InterfaceData &d = getData(rep, r, rp);

    if (r < rp)  // FIRST points represent r (the guy discovered through sampling)
    {
        if (d.pointA_ == nullptr)  // If the point we're considering replacing (P_v(r,.)) isn't there
            // Then we know we're doing better, so add it
            d.setFirst(q, s, si_);
        else  // Otherwise, he is there,
        {
            if (d.pointB_ == nullptr)  // But if the other guy doesn't exist, we can't compare.
            {
                // Should probably keep the one that is further away from rep?  Not known what to do in this case.
                // TODO: is this not part of the algorithm?
            }
            else  // We know both of these points exist, so we can check some distances
                if (si_->distance(q, d.pointB_) < si_->distance(d.pointA_, d.pointB_))
                // Distance with the new point is good, so set it.
                d.setFirst(q, s, si_);
        }
    }
    else  // SECOND points represent r (the guy discovered through sampling)
    {
        if (d.pointB_ == nullptr)  // If the point we're considering replacing (P_V(.,r)) isn't there...
            // Then we must be doing better, so add it
            d.setSecond(q, s, si_);
        else  // Otherwise, he is there
        {
            if (d.pointA_ == nullptr)  // But if the other guy doesn't exist, we can't compare.
            {
                // Should we be doing something cool here?
            }
            else if (si_->distance(q, d.pointA_) < si_->distance(d.pointB_, d.pointA_))
                // Distance with the new point is good, so set it
                d.setSecond(q, s, si_);
        }
    }

    // Lastly, save what we have discovered
    interfaceDataProperty_[rep].interfaceHash[index(r, rp)] = d;
}

void ompl::geometric::SPARSdb::abandonLists(base::State *st)
{
    stateProperty_[queryVertex_] = st;

    std::vector<Vertex> hold;
    nn_->nearestR(queryVertex_, sparseDelta_, hold);

    stateProperty_[queryVertex_] = nullptr;

    // For each of the vertices
    foreach (Vertex v, hold)
    {
        foreach (VertexPair r, interfaceDataProperty_[v].interfaceHash | boost::adaptors::map_keys)
            interfaceDataProperty_[v].interfaceHash[r].clear(si_);
    }
}

ompl::geometric::SPARSdb::Vertex ompl::geometric::SPARSdb::addGuard(base::State *state, GuardType type)
{
    Vertex m = boost::add_vertex(g_);
    stateProperty_[m] = state;
    colorProperty_[m] = type;

    // assert(si_->isValid(state));
    abandonLists(state);

    disjointSets_.make_set(m);
    nn_->add(m);
    resetFailures();

    if (verbose_)
    {
        OMPL_INFORM(" ---- addGuard() of type %f", type);
    }
#ifdef OMPL_THUNDER_DEBUG
    visualizeStateCallback(state, 4, sparseDelta_);  // Candidate node has already (just) been added
    sleep(0.1);
#endif

    return m;
}

void ompl::geometric::SPARSdb::connectGuards(Vertex v, Vertex vp)
{
    // OMPL_INFORM("connectGuards called ---------------------------------------------------------------- ");
    assert(v <= getNumVertices());
    assert(vp <= getNumVertices());

    if (verbose_)
    {
        OMPL_INFORM(" ------- connectGuards/addEdge: Connecting vertex %f to vertex %f", v, vp);
    }

    // Create the new edge
    Edge e = (boost::add_edge(v, vp, g_)).first;

    // Add associated properties to the edge
    edgeWeightProperty_[e] = distanceFunction(v, vp);  // TODO: use this value with astar
    edgeCollisionStateProperty_[e] = NOT_CHECKED;

    // Add the edge to the incrementeal connected components datastructure
    disjointSets_.union_set(v, vp);

// Debug in Rviz
#ifdef OMPL_THUNDER_DEBUG
    visualizeEdgeCallback(stateProperty_[v], stateProperty_[vp]);
    sleep(0.8);
#endif
}

bool ompl::geometric::SPARSdb::convertVertexPathToStatePath(std::vector<Vertex> &vertexPath,
                                                            const base::State *actualStart,
                                                            const base::State *actualGoal,
                                                            CandidateSolution &candidateSolution,
                                                            bool disableCollisionWarning)
{
    if (!vertexPath.size())
        return false;

    auto pathGeometric(std::make_shared<ompl::geometric::PathGeometric>(si_));
    candidateSolution.isApproximate_ = false;  // assume path is valid

    // Add original start if it is different than the first state
    if (actualStart != stateProperty_[vertexPath.back()])
    {
        pathGeometric->append(actualStart);

        // Add the edge status
        // the edge from actualStart to start is always valid otherwise we would not have used that start
        candidateSolution.edgeCollisionStatus_.push_back(FREE);
    }

    // Reverse the vertexPath and convert to state path
    for (std::size_t i = vertexPath.size(); i > 0; --i)
    {
        pathGeometric->append(stateProperty_[vertexPath[i - 1]]);

        // Add the edge status
        if (i > 1)  // skip the last vertex (its reversed)
        {
            Edge thisEdge = boost::edge(vertexPath[i - 1], vertexPath[i - 2], g_).first;

            // Check if any edges in path are not free (then it an approximate path)
            if (edgeCollisionStateProperty_[thisEdge] == IN_COLLISION)
            {
                candidateSolution.isApproximate_ = true;
                candidateSolution.edgeCollisionStatus_.push_back(IN_COLLISION);
            }
            else if (edgeCollisionStateProperty_[thisEdge] == NOT_CHECKED)
            {
                if (!disableCollisionWarning)
                    OMPL_ERROR("A chosen path has an edge that has not been checked for collision. This should not "
                               "happen");
                candidateSolution.edgeCollisionStatus_.push_back(NOT_CHECKED);
            }
            else
            {
                candidateSolution.edgeCollisionStatus_.push_back(FREE);
            }
        }
    }

    // Add original goal if it is different than the last state
    if (actualGoal != stateProperty_[vertexPath.front()])
    {
        pathGeometric->append(actualGoal);

        // Add the edge status
        // the edge from actualGoal to goal is always valid otherwise we would not have used that goal
        candidateSolution.edgeCollisionStatus_.push_back(FREE);
    }

    candidateSolution.path_ = pathGeometric;

    return true;
}

void ompl::geometric::SPARSdb::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    // Explicitly add start and goal states:
    for (unsigned long i : startM_)
        data.addStartVertex(base::PlannerDataVertex(stateProperty_[i], (int)START));

    for (unsigned long i : goalM_)
        data.addGoalVertex(base::PlannerDataVertex(stateProperty_[i], (int)GOAL));

    // I'm curious:
    if (goalM_.size() > 0)
    {
        throw Exception(name_, "SPARS2 has goal states?");
    }
    if (startM_.size() > 0)
    {
        throw Exception(name_, "SPARS2 has start states?");
    }

    // If there are even edges here
    if (boost::num_edges(g_) > 0)
    {
        // Adding edges and all other vertices simultaneously
        foreach (const Edge e, boost::edges(g_))
        {
            const Vertex v1 = boost::source(e, g_);
            const Vertex v2 = boost::target(e, g_);

            // TODO save weights!
            data.addEdge(base::PlannerDataVertex(stateProperty_[v1], (int)colorProperty_[v1]),
                         base::PlannerDataVertex(stateProperty_[v2], (int)colorProperty_[v2]));

            // OMPL_INFORM("Adding edge from vertex of type %d to vertex of type %d", colorProperty_[v1],
            // colorProperty_[v2]);
        }
    }
    // else
    //    OMPL_INFORM("%s: There are no edges in the graph!", getName().c_str());

    // Make sure to add edge-less nodes as well
    foreach (const Vertex n, boost::vertices(g_))
        if (boost::out_degree(n, g_) == 0)
            data.addVertex(base::PlannerDataVertex(stateProperty_[n], (int)colorProperty_[n]));

    data.properties["iterations INTEGER"] = std::to_string(iterations_);
}

void ompl::geometric::SPARSdb::setPlannerData(const base::PlannerData &data)
{
    // Check that the query vertex is initialized (used for internal nearest neighbor searches)
    checkQueryStateInitialization();

    // Add all vertices
    if (verbose_)
    {
        OMPL_INFORM("SPARS::setPlannerData: numVertices=%d", data.numVertices());
    }
    OMPL_INFORM("Loading PlannerData into SPARSdb");

    std::vector<Vertex> idToVertex;

    // Temp disable verbose mode for loading database
    bool wasVerbose = verbose_;
    verbose_ = false;

    OMPL_INFORM("Loading vertices:");
    // Add the nodes to the graph
    for (std::size_t vertexID = 0; vertexID < data.numVertices(); ++vertexID)
    {
        // Get the state from loaded planner data
        const base::State *oldState = data.getVertex(vertexID).getState();
        base::State *state = si_->cloneState(oldState);

        // Get the tag, which in this application represents the vertex type
        auto type = static_cast<GuardType>(data.getVertex(vertexID).getTag());

        // ADD GUARD
        idToVertex.push_back(addGuard(state, type));
    }

    OMPL_INFORM("Loading edges:");
    // Add the corresponding edges to the graph
    std::vector<unsigned int> edgeList;
    for (std::size_t fromVertex = 0; fromVertex < data.numVertices(); ++fromVertex)
    {
        edgeList.clear();

        // Get the edges
        data.getEdges(fromVertex, edgeList);  // returns num of edges

        Vertex m = idToVertex[fromVertex];

        // Process edges
        for (unsigned int toVertex : edgeList)
        {
            Vertex n = idToVertex[toVertex];

            // Add the edge to the graph
            const base::Cost weight(0);
            if (verbose_ && false)
            {
                OMPL_INFORM("    Adding edge from vertex id %d to id %d into edgeList", fromVertex, toVertex);
                OMPL_INFORM("      Vertex %d to %d", m, n);
            }
            connectGuards(m, n);
        }
    }  // for

    // Re-enable verbose mode, if necessary
    verbose_ = wasVerbose;
}

void ompl::geometric::SPARSdb::clearEdgeCollisionStates()
{
    foreach (const Edge e, boost::edges(g_))
        edgeCollisionStateProperty_[e] = NOT_CHECKED;  // each edge has an unknown state
}
