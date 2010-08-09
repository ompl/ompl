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
#include <ompl/base/GoalState.h>
#include <ompl/base/manifolds/SE2StateManifold.h>
#include <ompl/control/manifolds/RealVectorControlManifold.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/SimpleSetup.h>
#include <iostream>

namespace ob = ompl::base;
namespace oc = ompl::control;

bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    //    ob::ScopedState<ob::SE2StateManifold>
    /// cast the abstract state type to the type we expect
    const ob::SE2StateManifold::StateType *se2state = state->as<ob::SE2StateManifold::StateType>();

    /// extract the first component of the state and cast it to what we expect
    const ob::RealVectorStateManifold::StateType *pos = se2state->as<ob::RealVectorStateManifold::StateType>(0);

    /// extract the second component of the state and cast it to what we expect
    const ob::SO2StateManifold::StateType *rot = se2state->as<ob::SO2StateManifold::StateType>(1);
    
    /// check validity of state defined by pos & rot
    
    
    // return a value that is always true but uses the two variables we define, so we avoid compiler warnings
    return si->satisfiesBounds(state) && (void*)rot != (void*)pos;
}

oc::PropagationResult propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    const ob::SE2StateManifold::StateType *se2state = start->as<ob::SE2StateManifold::StateType>();
    const ob::RealVectorStateManifold::StateType *pos = se2state->as<ob::RealVectorStateManifold::StateType>(0);
    const ob::SO2StateManifold::StateType *rot = se2state->as<ob::SO2StateManifold::StateType>(1);
    const oc::RealVectorControlManifold::ControlType *rctrl = control->as<oc::RealVectorControlManifold::ControlType>();

    result->as<ob::SE2StateManifold::StateType>()->as<ob::RealVectorStateManifold::StateType>(0)->values[0] = 
	(*pos)[0] + (*rctrl)[0] * duration * cos(rot->value);
    
    result->as<ob::SE2StateManifold::StateType>()->as<ob::RealVectorStateManifold::StateType>(0)->values[1] = 
	(*pos)[1] + (*rctrl)[0] * duration * sin(rot->value);
    
    result->as<ob::SE2StateManifold::StateType>()->as<ob::SO2StateManifold::StateType>(1)->value = 
	rot->value + (*rctrl)[1];
    
    return oc::PROPAGATION_START_UNKNOWN;
}

void plan(void)
{    

    /// construct the manifold we are planning in
    ob::StateManifoldPtr manifold(new ob::SE2StateManifold());

    /// set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);
    
    manifold->as<ob::SE2StateManifold>()->setBounds(bounds);
    
    // create a control manifold
    oc::ControlManifoldPtr cmanifold(new oc::RealVectorControlManifold(manifold, 2));
    
    // set the bounds for the control manifold
    oc::RealVectorBounds cbounds(2);
    cbounds.setLow(-0.3);
    cbounds.setHigh(0.3);
    
    cmanifold->as<oc::RealVectorControlManifold>()->setBounds(cbounds);

    // set the state propagation routine 
    cmanifold->setPropagationFunction(boost::bind(&propagate, _1, _2, _3, _4));
    
    /// construct an instance of  space information from this manifold
    oc::SpaceInformationPtr si(new oc::SpaceInformation(manifold, cmanifold));

    /// set state validity checking for this space
    si->setStateValidityChecker(boost::bind(&isStateValid, si.get(),  _1));
    
    /// create a start state
    ob::ScopedState<ob::SE2StateManifold> start(manifold);
    start->setX(-0.5);
    start->setY(0.0);
    start->setYaw(0.0);
    
    /// create a goal state
    ob::ScopedState<ob::SE2StateManifold> goal(start);
    goal->setX(0.5);
    
    /// create a problem instance
    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));

    /// set the start and goal states
    pdef->setStartAndGoalStates(start, goal, 0.1);
    
    /// create a planner for the defined space
    ob::PlannerPtr planner(new oc::KPIECE1(si));

    /// set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    /// perform setup steps for the planner
    planner->setup();


    /// print the settings for this space
    si->printSettings(std::cout);

    /// print the problem settings
    pdef->print(std::cout);    
    
    /// attempt to solve the problem within one second of planning time
    bool solved = planner->solve(10.0);

    if (solved)
    {
	/// get the goal representation from the problem definition (not the same as the goal state)
	/// and inquire about the found path
	ob::PathPtr path = pdef->getGoal()->getSolutionPath();
	std::cout << "Found solution:" << std::endl;

	/// print the path to screen
	path->print(std::cout);
    }
    else
	std::cout << "No solution found" << std::endl;
}


void planWithSimpleSetup(void)
{
    /// construct the manifold we are planning in
    ob::StateManifoldPtr manifold(new ob::SE2StateManifold());

    /// set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);
    
    manifold->as<ob::SE2StateManifold>()->setBounds(bounds);

    // create a control manifold
    oc::ControlManifoldPtr cmanifold(new oc::RealVectorControlManifold(manifold, 2));
    
    // set the bounds for the control manifold
    oc::RealVectorBounds cbounds(2);
    cbounds.setLow(-0.3);
    cbounds.setHigh(0.3);
    
    cmanifold->as<oc::RealVectorControlManifold>()->setBounds(cbounds);

    // set the state propagation routine 
    cmanifold->setPropagationFunction(boost::bind(&propagate, _1, _2, _3, _4));
    
    // define a simple setup class
    oc::SimpleSetup ss(cmanifold);

    /// set state validity checking for this space
    ss.setStateValidityChecker(boost::bind(&isStateValid, ss.getSpaceInformation().get(), _1));
    
    /// create a random start state
    ob::ScopedState<ob::SE2StateManifold> start(manifold);
    (*start)[0]->as<ob::RealVectorStateManifold::StateType>()->values[0] = -0.5;
    (*start)[0]->as<ob::RealVectorStateManifold::StateType>()->values[1] = 0.0;
    (*start)[1]->as<ob::SO2StateManifold::StateType>()->value = 0.0;

    /// create a random goal state
    ob::ScopedState<ob::SE2StateManifold> goal(manifold);
    (*goal)[0]->as<ob::RealVectorStateManifold::StateType>()->values[0] = 0.0;
    (*goal)[0]->as<ob::RealVectorStateManifold::StateType>()->values[1] = 0.5;
    (*goal)[1]->as<ob::SO2StateManifold::StateType>()->value = 0.0;

    
    /// set the start and goal states; this call allows SimpleSetup to infer the planning manifold, if needed
    ss.setStartAndGoalStates(start, goal, 0.05);
        
    /// attempt to solve the problem within one second of planning time
    bool solved = ss.solve(10.0);

    if (solved)
    {
	std::cout << "Found solution:" << std::endl;
	/// print the path to screen

	ss.getSolutionPath().asGeometric()->print(std::cout);
    }
    else
	std::cout << "No solution found" << std::endl;
}

int main(int, char **)
{
    plan();
    
    std::cout << std::endl << std::endl;
    
    //    planWithSimpleSetup();
    
    return 0;
}
