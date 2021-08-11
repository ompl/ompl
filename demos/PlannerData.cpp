/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Rice University
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

/* Author: Ryan Luna, Luis G. Torres */

#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/base/PlannerDataGraph.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/goals/GoalState.h>

#include <boost/graph/astar_search.hpp>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state)
{
    // cast the abstract state type to the type we expect
    const auto *se3state = state->as<ob::SE3StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

    // extract the second component of the state and cast it to what we expect
    const auto *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

    // check validity of state defined by pos & rot


    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return (const void*)rot != (const void*)pos;
}

void planWithSimpleSetup()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE3StateSpace>());

    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-10);
    bounds.setHigh(10);

    space->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });

    // create a random start state
    ob::ScopedState<> start(space);
    start.random();

    // create a random goal state
    ob::ScopedState<> goal(space);
    goal.random();

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // this call is optional, but we put it in to get more output information
    ss.setup();
    ss.print();

    // attempt to find an exact solution within five seconds
    if (ss.solve(5.0) == ob::PlannerStatus::EXACT_SOLUTION)
    {
        og::PathGeometric slnPath = ss.getSolutionPath();

        std::cout << std::endl;
        std::cout << "Found solution with " << slnPath.getStateCount() << " states and length " << slnPath.length() << std::endl;
        // print the path to screen
        //slnPath.print(std::cout);

        std::cout << "Writing PlannerData to file './myPlannerData'" << std::endl;
        ob::PlannerData data(ss.getSpaceInformation());
        ss.getPlannerData(data);

        ob::PlannerDataStorage dataStorage;
        dataStorage.store(data, "myPlannerData");
    }
    else
        std::cout << "No solution found" << std::endl;
}

// Used for A* search.  Computes the heuristic distance from vertex v1 to the goal
ob::Cost distanceHeuristic(ob::PlannerData::Graph::Vertex v1,
                           const ob::GoalState* goal,
                           const ob::OptimizationObjective* obj,
                           const boost::property_map<ob::PlannerData::Graph::Type,
                           vertex_type_t>::type& plannerDataVertices)
{
    return ob::Cost(obj->costToGo(plannerDataVertices[v1]->getState(), goal));
}

void readPlannerData()
{
    std::cout << std::endl;
    std::cout << "Reading PlannerData from './myPlannerData'" << std::endl;

    // Recreating the space information from the stored planner data instance
    auto space(std::make_shared<ob::SE3StateSpace>());
    auto si(std::make_shared<ob::SpaceInformation>(space));

    ob::PlannerDataStorage dataStorage;
    ob::PlannerData data(si);

    // Loading an instance of PlannerData from disk.
    dataStorage.load("myPlannerData", data);

    // Re-extract a shortest path from the loaded planner data
    if (data.numStartVertices() > 0 && data.numGoalVertices() > 0)
    {
        // Create an optimization objective for optimizing path length in A*
        ob::PathLengthOptimizationObjective opt(si);

        // Computing the weights of all edges based on the state space distance
        // This is not done by default for efficiency
        data.computeEdgeWeights(opt);

        // Getting a handle to the raw Boost.Graph data
        ob::PlannerData::Graph::Type& graph = data.toBoostGraph();

        // Now we can apply any Boost.Graph algorithm.  How about A*!

        // create a predecessor map to store A* results in
        boost::vector_property_map<ob::PlannerData::Graph::Vertex> prev(data.numVertices());

        // Retrieve a property map with the PlannerDataVertex object pointers for quick lookup
        boost::property_map<ob::PlannerData::Graph::Type, vertex_type_t>::type vertices = get(vertex_type_t(), graph);

        // Run A* search over our planner data
        ob::GoalState goal(si);
        goal.setState(data.getGoalVertex(0).getState());
        ob::PlannerData::Graph::Vertex start = boost::vertex(data.getStartIndex(0), graph);
        boost::astar_visitor<boost::null_visitor> dummy_visitor;
        boost::astar_search(graph, start,
            [&goal, &opt, &vertices](ob::PlannerData::Graph::Vertex v1) { return distanceHeuristic(v1, &goal, &opt, vertices); },
            boost::predecessor_map(prev).
            distance_compare([&opt](ob::Cost c1, ob::Cost c2) { return opt.isCostBetterThan(c1, c2); }).
            distance_combine([&opt](ob::Cost c1, ob::Cost c2) { return opt.combineCosts(c1, c2); }).
            distance_inf(opt.infiniteCost()).
            distance_zero(opt.identityCost()).
            visitor(dummy_visitor));

        // Extracting the path
        og::PathGeometric path(si);
        for (ob::PlannerData::Graph::Vertex pos = boost::vertex(data.getGoalIndex(0), graph);
             prev[pos] != pos;
             pos = prev[pos])
        {
            path.append(vertices[pos]->getState());
        }
        path.append(vertices[start]->getState());
        path.reverse();

        // print the path to screen
        //path.print(std::cout);
        std::cout << "Found stored solution with " << path.getStateCount() << " states and length " << path.length() << std::endl;
    }
}

int main(int /*argc*/, char ** /*argv*/)
{
    // Plan and save all of the planner data to disk
    planWithSimpleSetup();

    // Read in the saved planner data and extract the solution path
    readPlannerData();

    return 0;
}
