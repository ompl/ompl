/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Rice University
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

/* Author: Javier V. Gomez */

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include <ompl/control/StatePropagator.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/StateSpaceFromPropagator.h>

namespace ob = ompl::base;
//namespace og = ompl::geometric;
//namespace ot = ompl::time;
namespace oc = ompl::control;

bool isStateValid(const oc::SpaceInformationPtr &si, const ob::State *state)
{
    // return a value that is always true
    return si->satisfiesBounds(state); 
}

// TODO: for any reason I am not able to generalize to any radius. Try it again.
class ReedsSheppStatePropagator : public oc::StatePropagator
{
public:
    ReedsSheppStatePropagator (const oc::SpaceInformationPtr &si) : oc::StatePropagator(si)
    {
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
            rot    + ctrl[0]*ctrl[1] * duration);
    }

    // TODO how can this be a const function?
    virtual bool steer (const ob::State *from, const ob::State *to, std::vector<oc::TimedControl> &controls, double &duration) const
    {
        ob::ReedsSheppStateSpace::ReedsSheppPath rsp = rs_.reedsShepp(from, to);
        oc::Control *c;
        int i = 0;
        
        while (rsp.type_[i] != 0 && i<5)
        {
             c = si_->allocControl();
            if (rsp.length_[i] > 0) // Forward
                c->as<oc::RealVectorControlSpace::ControlType>()->values[0] = 1;
            else // Backwards
                c->as<oc::RealVectorControlSpace::ControlType>()->values[0] = -1;

            if (rsp.type_[i] == 1) // Left
                c->as<oc::RealVectorControlSpace::ControlType>()->values[1] = 1;
            else if (rsp.type_[i] == 3) // Right
                c->as<oc::RealVectorControlSpace::ControlType>()->values[1] = -1;
            else // Straight
                c->as<oc::RealVectorControlSpace::ControlType>()->values[1] = 0;

            double cduration = std::abs(rsp.length_[i]);
            duration += cduration;
            controls.push_back(std::make_pair(c,cduration));

            ++i;
        }

        if (!controls.empty())
            return true;
        return false;
    }

    virtual bool canSteer() const
    {
        return true;
    }

private:
    ob::ReedsSheppStateSpace rs_;
};




int main(int argc, char** argv)
{
    // construct the state space we are planning in
    ob::StateSpacePtr space(new ob::SE2StateSpace());

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-5);
    bounds.setHigh(5);
    space->as<ob::SE2StateSpace>()->setBounds(bounds);

    // create a control space - RealVector space is not the most correct space, but easier to use.
    oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 2));
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-1);
    cbounds.setHigh(1);
    cspace->as<oc::RealVectorControlSpace>()->setBounds(bounds);
    
    // define a simple setup class
    oc::SimpleSetup ss(cspace);

    // set the state propagation routine
    // The state propagator works with a control::SpaceInformation...
    const oc::SpaceInformationPtr &siC = ss.getSpaceInformation();
    oc::StatePropagatorPtr sp (new ReedsSheppStatePropagator(siC));
    
    ob::StateSpaceFromPropagator<ob::SE2StateSpace> mySE2(sp);
    ob::StateSpacePtr mySpace (new ob::StateSpaceFromPropagator<ob::SE2StateSpace>(sp));
    // ... but here we are creating a base::SpaceInformation with the new state space (no control used).
    ob::SpaceInformationPtr si2 (new ob::SpaceInformation(mySpace));

    ob::ScopedState<> start(mySpace), goal(mySpace);
    start[0] = start[1] = start[2] = 0.; // From (0,0,0)
    goal[0] = goal[1] = 0.; goal[2] = boost::math::constants::pi<double>(); // To (0,0,pi)
    
    ob::State *istate = si2->allocState();
    
    for(double t = 0.01; t<=1; t+=0.01)
    {
        mySpace->interpolate(start.get(), goal.get(),t, istate);
        
        std::cout << istate->as<ob::StateSpaceFromPropagator<ob::SE2StateSpace>::StateType >()->getX() << "\t" 
                  << istate->as<ob::StateSpaceFromPropagator<ob::SE2StateSpace>::StateType >()->getY() << "\t" 
                  << istate->as<ob::StateSpaceFromPropagator<ob::SE2StateSpace>::StateType >()->getYaw() <<std::endl;
    }
    
    /*mySpace->interpolate(start.get(), goal.get(),1/3., istate);
    si2->printState(istate);
    
    mySpace->interpolate(start.get(), goal.get(),2/3., istate);
    si2->printState(istate);
    
    mySpace->interpolate(start.get(), goal.get(),1, istate);
    si2->printState(istate);*/
    
    
    return 0;
}
