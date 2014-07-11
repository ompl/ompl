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

/* Author: Javier V. Gomez*/

#include <ompl/base/State.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/fmt/FMT.h>
#include <ompl/control/DirectedControlSampler.h>
#include <ompl/control/SimpleDirectedControlSampler.h>

#include <boost/math/constants/constants.hpp>

//#include "kinodynamic/double_integrator.hpp"

namespace ob = ompl::base;
namespace oc = ompl::control;
//namespace og = ompl::geometric;
//namespace ot = ompl::time;

bool isStateValid(const oc::SpaceInformationPtr &si, const ob::State *state)
{
    // return a value that is always true
    return si->satisfiesBounds(state); 
}

class SteeredStatePropagator : public oc::StatePropagator
{
public:
    SteeredStatePropagator (const oc::SpaceInformationPtr &si) : oc::StatePropagator(si)
    {
        sdcs_ = new oc::SimpleDirectedControlSampler(si_, 100);
    }

    virtual void propagate (const ob::State *state, const oc::Control *control, const double duration, ob::State *result) const
    {
        const double* values = state->as<ob::RealVectorStateSpace::StateType>()->values;
        const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

        result->as<ob::RealVectorStateSpace::StateType>()->values[0] = values[0]+values[2]*duration+ctrl[0]/2.0*duration*duration; // x
        result->as<ob::RealVectorStateSpace::StateType>()->values[1] = values[1]+values[3]*duration+ctrl[1]/2.0*duration*duration; // y
        result->as<ob::RealVectorStateSpace::StateType>()->values[2] = values[2]+ctrl[0]*duration; // vx
        result->as<ob::RealVectorStateSpace::StateType>()->values[3] = values[3]+ctrl[1]*duration; // vy

    }

    virtual bool steer (const ob::State *from, const ob::State *to, oc::Control *result, double &duration) const 
    {
        ob::State* state = si_->cloneState(to);
        int steps = sdcs_->sampleTo(result, from, state);
        duration = steps*si_->getPropagationStepSize();
        return true;
    }

    virtual bool canSteer() const
    {
        return true;
    }
    
private:
    oc::SimpleDirectedControlSampler* sdcs_;
    
};

int main(int argc, char** argv)
{
    // construct the state space we are planning in
    ob::StateSpacePtr space(new ob::RealVectorStateSpace(4));

    // x,y,vx,vy
    ob::RealVectorBounds bounds(4);
    bounds.setLow(0,-10);
    bounds.setHigh(0,10);
    bounds.setLow(1,-10);
    bounds.setHigh(1,10);
    bounds.setLow(2,-1);
    bounds.setHigh(2,1);
    bounds.setLow(3,-1);
    bounds.setHigh(3,1);

    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    // create a control space
    oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 2));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-1);
    cbounds.setHigh(1);

    cspace->as<oc::RealVectorControlSpace>()->setBounds(cbounds);

    // define a simple setup class
    oc::SimpleSetup ss(cspace);

    // set the state propagation routine
    const oc::SpaceInformationPtr &si = ss.getSpaceInformation();
    oc::StatePropagatorPtr sp (new SteeredStatePropagator(si));
    ss.setStatePropagator(sp);
    
    /*oc::FMT fmt(si);
    fmt.setNumSamples(1000);
    ss.setPlanner(ob::PlannerPtr(&fmt));*/
    ss.setPlanner(ob::PlannerPtr(new oc::FMT(si)));
    ss.getPlanner()->as<oc::FMT>()->setNumSamples(1000);

    // set state validity checking for this space
    ss.setStateValidityChecker(boost::bind(&isStateValid, si, _1));

    // testing the self-made propagator
    ob::ScopedState<ob::RealVectorStateSpace> start(space);
    start->values[0] = 0.0;
    start->values[1] = 0.0;
    start->values[2] = 0.0;
    start->values[3] = 0.0;

    ob::ScopedState<ob::RealVectorStateSpace> goal(space);
    goal->values[0] = 8.0;
    goal->values[1] = 8.0;
    goal->values[2] = 0.0;
    goal->values[3] = 0.0;

    oc::DirectedControlSamplerPtr cs = si->allocDirectedControlSampler();
    si->setMinMaxControlDuration(1,100);
    //si->setup();

    //cs->sampleTo(c,start.get(),goal.get());
    
    ss.setStartAndGoalStates(start, goal, 0.2);
    ob::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen

        ss.getSolutionPath().printAsMatrix(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;

    ss.getPlanner()->as<oc::FMT>()->saveTree();

    return 0;   
}
