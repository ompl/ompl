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

class SteeredStatePropagator : public oc::StatePropagator
{
public:
    SteeredStatePropagator (const oc::SpaceInformationPtr &si) : oc::StatePropagator(si)
    {
        sdcs_ = new oc::SimpleDirectedControlSampler(si_, 50);
    }

    virtual void propagate (const ob::State *state, const oc::Control *control, const double duration, ob::State *result) const
    {
        const ob::SE2StateSpace::StateType *se2state = state->as<ob::SE2StateSpace::StateType>();
        const double* pos = se2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
        const double rot = se2state->as<ob::SO2StateSpace::StateType>(1)->value;
        const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

        result->as<ob::SE2StateSpace::StateType>()->setXY(
            pos[0] + ctrl[0] * duration * cos(rot),
            pos[1] + ctrl[0] * duration * sin(rot));
        result->as<ob::SE2StateSpace::StateType>()->setYaw(
            rot    + ctrl[1] * duration);

        // Ensure that the car's resulting orientation lies between 0 and 2*pi.
        ob::SO2StateSpace SO2;
        ompl::base::SE2StateSpace::StateType& s = *result->as<ompl::base::SE2StateSpace::StateType>();
        SO2.enforceBounds(s[1]);

        // Printing matrix-like for fast Matlab visualization.
        /*std::cout << result->as<ob::SE2StateSpace::StateType>()->getX() << "\t"
                  << result->as<ob::SE2StateSpace::StateType>()->getY() << "\t"
                  << result->as<ob::SE2StateSpace::StateType>()->getYaw() << std::endl;*/
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


/*void SimpleCarODE(const oc::ODESolver::StateType& q, const oc::Control* c, oc::ODESolver::StateType& qdot)
{
    // Retrieve control values.  Velocity is the first entry, steering angle is second.
    const double *u = c->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double velocity = u[0];
    const double steeringAngle = u[1];
    // Retrieve the current orientation of the car.  The memory for ompl::base::SE2StateSpace is mapped as:
    // 0: x
    // 1: y
    // 2: theta
    const double theta = q[2];
    // Ensure qdot is the same size as q.  Zero out all values.
    qdot.resize(q.size(), 0);
    qdot[0] = velocity * cos(theta);            // x-dot
    qdot[1] = velocity * sin(theta);            // y-dot
    qdot[2] = velocity * tan(steeringAngle);    // theta-dot
}*/

/*void postPropagate(const ob::State* state, const oc::Control* control, const double duration, ob::State* result)
{
    ob::SO2StateSpace SO2;
    // Ensure that the car's resulting orientation lies between 0 and 2*pi.
    ob::SE2StateSpace::StateType& s = *result->as<ob::SE2StateSpace::StateType>();
    SO2.enforceBounds(s[1]);
}*/


int main(int argc, char** argv)
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
    const oc::SpaceInformationPtr &si = ss.getSpaceInformation();
    oc::StatePropagatorPtr sp (new SteeredStatePropagator(si));
    ss.setStatePropagator(sp);
    ss.setPlanner(ob::PlannerPtr(new oc::RRT(si)));

    // set state validity checking for this space
    ss.setStateValidityChecker(boost::bind(&isStateValid, si, _1));

    // testing the self-made propagator
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(0.0);
    start->setY(0.0);
    start->setYaw(0.0);
    
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setX(0.7);
    goal->setY(0.7);
    goal->setYaw(-boost::math::constants::pi<double>()/2);

    oc::DirectedControlSamplerPtr cs = si->allocDirectedControlSampler();
    //si->setMinMaxControlDuration(1,50);
    //si->setup();

    //cs->sampleTo(c,start.get(),goal.get());
    

	ss.setStartAndGoalStates(start, goal, 0.05);
    ob::PlannerStatus solved = ss.solve(10.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen

        ss.getSolutionPath().printAsMatrix(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;

    return 0;
    
}
