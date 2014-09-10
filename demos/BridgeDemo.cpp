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
#include <ompl/control/StatePropagator.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/FromPropagatorStateSpace.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
//#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <fstream>
#include <limits>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

ob::ReedsSheppStateSpace::ReedsSheppPath grsp;

bool isStateValid(const ob::State *state)
{
    return true;
}

// TODO: for any reason I am not able to generalize to any radius. Try it again.
class ReedsSheppStatePropagator : public oc::StatePropagator
{
public:
    ReedsSheppStatePropagator (const oc::SpaceInformationPtr &si) : oc::StatePropagator(si)
    {
        rho_ = 1.0;
    }

    virtual void propagate (const ob::State *state, const oc::Control *control, const double duration, ob::State *result) const
    {
        // Code extracted from ReedsSheppStateSpace::interpolate()
        ob::ReedsSheppStateSpace::StateType* s = rs_.allocState()->as<ob::ReedsSheppStateSpace::StateType>();
        
        double phi,v;
        int steering = (int)control->as<oc::RealVectorControlSpace::ControlType>()->values[1];
        int forward = (int)control->as<oc::RealVectorControlSpace::ControlType>()->values[0];
        
        s->setXY(state->as<ob::ReedsSheppStateSpace::StateType>()->getX(), state->as<ob::ReedsSheppStateSpace::StateType>()->getY());
        s->setYaw(state->as<ob::ReedsSheppStateSpace::StateType>()->getYaw());
        phi = s->getYaw();

        if (forward > 0)
             v = duration;
        else
             v = -duration;

        switch(steering)
        {
            case 1:
                s->setXY(s->getX() + sin(phi+v) - sin(phi), s->getY() - cos(phi+v) + cos(phi));
                s->setYaw(phi+v);
                break;
            case -1:
                s->setXY(s->getX() - sin(phi-v) + sin(phi), s->getY() + cos(phi-v) - cos(phi));
                s->setYaw(phi-v);
                break;
            case 0:
                s->setXY(s->getX() + v * cos(phi), s->getY() + v * sin(phi));
                break;
        }
        
        result->as<ob::ReedsSheppStateSpace::StateType>()->setX(s->getX() * rho_);
        result->as<ob::ReedsSheppStateSpace::StateType>()->setY(s->getY() * rho_);
        rs_.getSubspace(1)->enforceBounds(s->as<ob::SO2StateSpace::StateType>(1));
        result->as<ob::ReedsSheppStateSpace::StateType>()->setYaw(s->getYaw());
        rs_.freeState(s);
    }

    // TODO how can this be a const function?
    virtual bool steer (const ob::State *from, const ob::State *to, std::vector<oc::TimedControl> &controls, double &duration) const
    {
        ob::ReedsSheppStateSpace::ReedsSheppPath rsp = rs_.reedsShepp(from, to);
        grsp = rsp;
        oc::Control *c;
        int i = 0;
        
        while (rsp.type_[i] != 0 && i < 5)
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
    double rho_;
};

int main(int argc, char** argv)
{
    // construct the state space we are planning in
    ob::StateSpacePtr space(new ob::SE2StateSpace());

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    space->as<ob::SE2StateSpace>()->setBounds(bounds);

    // create a control space - RealVector space is not the most correct space, but easier to use.
    oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 2));
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-1);
    cbounds.setHigh(1);
    cspace->as<oc::RealVectorControlSpace>()->setBounds(bounds);

    // define the space information of the real, controlled system
    oc::SpaceInformationPtr siC (new  oc::SpaceInformation(space,cspace));

    // set the state propagation routine
    oc::StatePropagatorPtr sp (new ReedsSheppStatePropagator(siC));

    ////////////////////////////
    // and now, with the control space information and state propagator
    // we create the geometric version of the problem.
    ////////////////////////////
    ob::StateSpacePtr fromPropSpace (new ob::FromPropagatorStateSpace<ob::SE2StateSpace>(sp));
    fromPropSpace->as<ob::FromPropagatorStateSpace<ob::SE2StateSpace> >()->setBounds(bounds);

    // creating the new simple setup for geometric planning
    og::SimpleSetup ss (fromPropSpace);
    ss.setStateValidityChecker(boost::bind(&isStateValid, _1));

    ob::ScopedState<> start(fromPropSpace), goal(fromPropSpace);
    start[0] = 0.;
    start[1] = 0.;
    start[2] = 0.;
    goal[0] = boost::lexical_cast<double>(argv[1]);
    goal[1] = boost::lexical_cast<double>(argv[2]);
    goal[2] = boost::lexical_cast<double>(argv[3]);

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // set the planner
    ob::PlannerPtr planner(new og::RRT(ss.getSpaceInformation()));
    
    ob::ParamSet& params = planner->params();
    if (params.hasParam(std::string("range")))
        params.setParam(std::string("range"), boost::lexical_cast<std::string>(1));
        
    ss.setPlanner(planner);

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
        std::fstream fs;
        fs.open ("path.txt", std::fstream::out | std::fstream::trunc);
        //ss.simplifySolution();
        og::PathGeometric& p = ss.getSolutionPath();
        p.interpolate(100);
        p.printAsMatrix(fs);
        fs.close();
    }
    else
        std::cout << "No solution found" << std::endl;

    /*ob::ScopedState<> s1(fromPropSpace), s2(fromPropSpace);
    s1[0] = 1.23822;
    s1[1] = 0.484691;
    s1[2] = -2.73113;
    s2[0] = boost::lexical_cast<double>(argv[1]);
    s2[1] = boost::lexical_cast<double>(argv[2]);
    s2[2] = boost::lexical_cast<double>(argv[3]);

    ob::State *istate = fromPropSpace->allocState();
    
    std::fstream fs;
    fs.open ("test.txt", std::fstream::out | std::fstream::trunc);
          
    for (double i = 0; i <= 1.0001; i+= 0.01)
    {
        fromPropSpace->interpolate(s1.get(),s2.get(),i,istate);
       
        fs << istate->as<ob::SE2StateSpace::StateType>()->getX() << "\t"
                  << istate->as<ob::SE2StateSpace::StateType>()->getY() << "\t"
                  << istate->as<ob::SE2StateSpace::StateType>()->getYaw() << std::endl;
    }

    fs.close();

    int i = 0;
    while (grsp.type_[i] != 0 && i < 5)
    {
        std::cout << std::abs(grsp.length_[i]);
        if (grsp.length_[i] > 0) // Forward
            std::cout << " Forward ";
        else // Backwards
            std::cout << " Backward ";

        if (grsp.type_[i] == 1) // Left
            std::cout << "Left" << std::endl;
        else if (grsp.type_[i] == 3) // Right
            std::cout << "Right" << std::endl;
        else // Straight
            std::cout << "Straight" << std::endl;

        ++i;
    }*/

    return 0;
}
