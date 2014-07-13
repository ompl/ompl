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
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/spaces/DiscreteControlSpace.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/fmt/FMT.h>
#include <ompl/control/DirectedControlSampler.h>
#include <ompl/control/SimpleDirectedControlSampler.h>

#include <boost/math/constants/constants.hpp>

namespace ob = ompl::base;
namespace oc = ompl::control;
//namespace og = ompl::geometric;
//namespace ot = ompl::time;

bool isStateValid(const oc::SpaceInformationPtr &si, const ob::State *state)
{
    // return a value that is always true
    return si->satisfiesBounds(state); 
}

class ReedsSheppStatePropagator : public oc::StatePropagator
{
public:
    ReedsSheppStatePropagator (const oc::SpaceInformationPtr &si) : oc::StatePropagator(si)
    {
        sdcs_ = new oc::SimpleDirectedControlSampler(si_, 100);
    }

    virtual void propagate (const ob::State *state, const oc::Control *control, const double duration, ob::State *result) const
    {
        //std::cout << "Propagating" << std::endl;
        /*const ob::SE2StateSpace::StateType *se2state = start->as<ob::SE2StateSpace::StateType>();
        const double* pos = se2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
        const double rot = se2state->as<ob::SO2StateSpace::StateType>(1)->value;
        const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

        result->as<ob::SE2StateSpace::StateType>()->setXY(
            pos[0] + ctrl[0] * duration * cos(rot),
            pos[1] + ctrl[0] * duration * sin(rot));
        result->as<ob::SE2StateSpace::StateType>()->setYaw(
            rot    + ctrl[1] * duration);*/
    }

    virtual bool steer (const ob::State *from, const ob::State *to, oc::Control *result, double &duration) const 
    {
        ob::ReedsSheppStateSpace::ReedsSheppPath rsp = rs_.reedsShepp(from, to);
        for (int i = 0; i<5; ++i)
        {
           std::cout << rsp.length_[i] << "   " << rsp.type_[i] << std::endl;
        }
        
        return true;
    }
    
    virtual double distance(const ob::State *from, const ob::State *to) const
    {
        return rs_.distance(from,to);
    }

    virtual bool canSteer() const
    {
        return true;
    }

    virtual bool hasDistance() const
    {
        return true;
    }
    
private:
    oc::SimpleDirectedControlSampler* sdcs_;
    ob::ReedsSheppStateSpace rs_;
};

int main(int argc, char** argv)
{
    const int r = 1;
    // construct the state space we are planning in
    ob::StateSpacePtr space(new ob::SE2StateSpace());

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-1);
    bounds.setHigh(1);
    space->as<ob::SE2StateSpace>()->setBounds(bounds);

    // create a control space
    oc::CompoundControlSpace *ccspace = new oc::CompoundControlSpace(space);
    ccspace->addSubspace(oc::ControlSpacePtr(new oc::DiscreteControlSpace(space, -1, 1)));
    ccspace->addSubspace(oc::ControlSpacePtr(new oc::DiscreteControlSpace(space, -r, r)));
    oc::ControlSpacePtr cspace(ccspace);
    
    // define a simple setup class
    oc::SimpleSetup ss(cspace);

    // set the state propagation routine
    const oc::SpaceInformationPtr &si = ss.getSpaceInformation();
    oc::StatePropagatorPtr sp (new ReedsSheppStatePropagator(si));
    ss.setStatePropagator(sp);

    ss.setPlanner(ob::PlannerPtr(new oc::FMT(si)));
    ss.getPlanner()->as<oc::FMT>()->setNumSamples(1000);

    // set state validity checking for this space
    ss.setStateValidityChecker(boost::bind(&isStateValid, si, _1));

    // Setting start and goal states.
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(0.0);
    start->setY(0.0);
    start->setYaw(0.0);

    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setX(0.0);
    goal->setY(0.0);
    goal->setYaw(3.14);
    
    ss.setStartAndGoalStates(start, goal, 0.2);

    // testing the self-made propagator
    oc::DirectedControlSamplerPtr cs = si->allocDirectedControlSampler();
    si->setMinMaxControlDuration(1,100);
    si->setup();
    
    std::cout << si->distance(start.get(),goal.get()) << std::endl;
    cs->sampleTo(si->allocControl(),start.get(),goal.get());
    
    /*
    ob::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen

        ss.getSolutionPath().printAsMatrix(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;

    ss.getPlanner()->as<oc::FMT>()->saveTree();*/

    return 0;   
}
