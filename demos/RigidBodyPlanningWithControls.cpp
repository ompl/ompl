/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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

/* Author: Ioan Sucan */

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/syclop/SyclopRRT.h>
#include <ompl/control/planners/syclop/SyclopEST.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/syclop/GridDecomposition.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>

namespace ob = ompl::base;
namespace oc = ompl::control;

// a decomposition is only needed for SyclopRRT and SyclopEST
class MyDecomposition : public oc::GridDecomposition
{
public:
    MyDecomposition(const int length, const ob::RealVectorBounds& bounds)
        : GridDecomposition(length, 2, bounds)
    {
    }
    virtual void project(const ob::State* s, std::vector<double>& coord) const
    {
        coord.resize(2);
        coord[0] = s->as<ob::SE2StateSpace::StateType>()->getX();
        coord[1] = s->as<ob::SE2StateSpace::StateType>()->getY();
    }

    virtual void sampleFullState(const ob::StateSamplerPtr& sampler, const std::vector<double>& coord, ob::State* s) const
    {
        sampler->sampleUniform(s);
        s->as<ob::SE2StateSpace::StateType>()->setXY(coord[0], coord[1]);
    }
};

bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    //    ob::ScopedState<ob::SE2StateSpace>
    // cast the abstract state type to the type we expect
    const ob::SE2StateSpace::StateType *se2state = state->as<ob::SE2StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const ob::RealVectorStateSpace::StateType *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

    // extract the second component of the state and cast it to what we expect
    const ob::SO2StateSpace::StateType *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

    // check validity of state defined by pos & rot


    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return si->satisfiesBounds(state) && (const void*)rot != (const void*)pos;
}

void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    const ob::SE2StateSpace::StateType *se2state = start->as<ob::SE2StateSpace::StateType>();
    const double* pos = se2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    const double rot = se2state->as<ob::SO2StateSpace::StateType>(1)->value;
    const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

    result->as<ob::SE2StateSpace::StateType>()->setXY(
        pos[0] + ctrl[0] * duration * cos(rot),
        pos[1] + ctrl[0] * duration * sin(rot));
    result->as<ob::SE2StateSpace::StateType>()->setYaw(
        rot    + ctrl[1] * duration);
}

void plan(void)
{

    // construct the state space we are planning in
    ob::StateSpacePtr space(new ob::SE2StateSpace());

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);

    space->as<ob::SE2StateSpace>()->setBounds(bounds);

    // create a control space
    oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 2));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-0.3);
    cbounds.setHigh(0.3);

    cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

    // construct an instance of  space information from this control space
    oc::SpaceInformationPtr si(new oc::SpaceInformation(space, cspace));

    // set state validity checking for this space
    si->setStateValidityChecker(boost::bind(&isStateValid, si.get(),  _1));

    // set the state propagation routine
    si->setStatePropagator(boost::bind(&propagate, _1, _2, _3, _4));

    // create a start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(-0.5);
    start->setY(0.0);
    start->setYaw(0.0);

    // create a goal state
    ob::ScopedState<ob::SE2StateSpace> goal(start);
    goal->setX(0.5);

    // create a problem instance
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal, 0.1);

    // create a planner for the defined space
    //ob::PlannerPtr planner(new oc::RRT(si));
    //ob::PlannerPtr planner(new oc::EST(si));
    //ob::PlannerPtr planner(new oc::KPIECE1(si));
    oc::DecompositionPtr decomp(new MyDecomposition(32, bounds));
    ob::PlannerPtr planner(new oc::SyclopEST(si, decomp));
    //ob::PlannerPtr planner(new oc::SyclopRRT(si, decomp));

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();


    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->solve(10.0);

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        path->print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}


void planWithSimpleSetup(void)
{
    // construct the state space we are planning in
    ob::StateSpacePtr space(new ob::SE2StateSpace());

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);

    space->as<ob::SE2StateSpace>()->setBounds(bounds);

    // create a control space
    oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 2));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-0.3);
    cbounds.setHigh(0.3);

    cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

    // define a simple setup class
    oc::SimpleSetup ss(cspace);

    // set the state propagation routine
    ss.setStatePropagator(boost::bind(&propagate, _1, _2, _3, _4));

    // set state validity checking for this space
    ss.setStateValidityChecker(boost::bind(&isStateValid, ss.getSpaceInformation().get(), _1));

    // create a start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(-0.5);
    start->setY(0.0);
    start->setYaw(0.0);

    // create a  goal state; use the hard way to set the elements
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    (*goal)[0]->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0.0;
    (*goal)[0]->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0.5;
    (*goal)[1]->as<ob::SO2StateSpace::StateType>()->value = 0.0;


    // set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.05);

    // ss.setPlanner(ob::PlannerPtr(new oc::PDST(ss.getSpaceInformation())));
    // ss.getSpaceInformation()->setMinMaxControlDuration(1,100);
    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve(10.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen

        ss.getSolutionPath().printAsMatrix(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int, char **)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    // plan();
    //
    // std::cout << std::endl << std::endl;
    //
    planWithSimpleSetup();

    return 0;
}
