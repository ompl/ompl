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

#include <ompl/util/PPM.h>
#include <ompl/config.h>
#include <../tests/resources/config.h>

#include <boost/filesystem.hpp>
#include <iostream>

#include <fstream>
#include <limits>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

// TODO: generalize for any radius.
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
        const int steering = (int)control->as<oc::RealVectorControlSpace::ControlType>()->values[1];
        const int forward = (int)control->as<oc::RealVectorControlSpace::ControlType>()->values[0];

        s->setXY(state->as<ob::ReedsSheppStateSpace::StateType>()->getX(), state->as<ob::ReedsSheppStateSpace::StateType>()->getY());
        s->setYaw(state->as<ob::ReedsSheppStateSpace::StateType>()->getYaw());
        phi = s->getYaw();

        if (forward > 0)
             v =  duration;
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

    virtual bool steer (const ob::State *from, const ob::State *to, std::vector<oc::TimedControl> &controls, double &duration) const
    {
        ob::ReedsSheppStateSpace::ReedsSheppPath rsp = rs_.reedsShepp(from, to);
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

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

class CarIn2D
{
public:

    CarIn2D(const char *ppm_file)
    {
        bool ok = false;
        try
        {
            ppm_.loadFile(ppm_file);
            ok = true;
        }
        catch(ompl::Exception &ex)
        {
            OMPL_ERROR("Unable to load %s.\n%s", ppm_file, ex.what());
        }
        if (ok)
        {
            // construct the state space we are planning in
            ob::StateSpacePtr space(new ob::SE2StateSpace());

            // set the bounds for the R^2 part of SE(2)
            ob::RealVectorBounds bounds(2);
            bounds.setLow(0, 0);
            bounds.setHigh(0, ppm_.getWidth());
            bounds.setLow(1, 0);
            bounds.setHigh(1, ppm_.getHeight());
            space->as<ob::SE2StateSpace>()->setBounds(bounds);
            maxWidth_ = ppm_.getWidth() - 1;
            maxHeight_ = ppm_.getHeight() - 1;

            // create a control space - RealVector space is not the most correct space, but easier to use.
            oc::ControlSpacePtr cspace(new oc::RealVectorControlSpace(space, 2));
            ob::RealVectorBounds cbounds(2);
            cbounds.setLow(-1);
            cbounds.setHigh(1);
            cspace->as<oc::RealVectorControlSpace>()->setBounds(bounds);

            // define the space information of the real, controlled system
            siC_.reset(new  oc::SpaceInformation(space,cspace));

            // set the state propagation routine
            oc::StatePropagatorPtr sp (new ReedsSheppStatePropagator(siC_));

            ////////////////////////////
            // and now, with the control space information and state propagator
            // we create the geometric version of the problem.
            ////////////////////////////
            ob::StateSpace *fromPropSpace (new ob::FromPropagatorStateSpace<ob::SE2StateSpace>(sp));
            fromPropSpace->as<ob::FromPropagatorStateSpace<ob::SE2StateSpace> >()->setBounds(bounds);

            // creating the new simple setup for geometric planning
            ss_.reset(new og::SimpleSetup(ob::StateSpacePtr(fromPropSpace)));
            ss_->setStateValidityChecker(boost::bind(&CarIn2D::isStateValid, this, _1));
            ss_->getSpaceInformation()->setStateValidityCheckingResolution(1.0 / fromPropSpace->getMaximumExtent());

        }
    }

    bool plan(unsigned int goal_row, unsigned int goal_col,  double goal_theta)
    {
        if (!ss_)
            return false;

        ob::ScopedState<> start(ss_->getStateSpace()), goal(ss_->getStateSpace());
        start[0] = 800.;
        start[1] = 500.;
        start[2] = 0.;
        goal[0] = goal_row;
        goal[1] = goal_col;
        goal[2] = goal_theta;

        // set the start and goal states
        ss_->setStartAndGoalStates(start, goal);

        // set the planner
        ob::PlannerPtr planner(new og::RRT(ss_->getSpaceInformation()));

        ss_->setPlanner(planner);

        // attempt to solve the problem within one second of planning time
        ss_->solve(1.0);

        if (ss_->haveSolutionPath())
            return true;
        else
            return false;
    }

    void recordSolution()
    {
        if (!ss_ || !ss_->haveSolutionPath())
            return;
        og::PathGeometric &p = ss_->getSolutionPath();
        p.interpolate();
        for (std::size_t i = 0 ; i < p.getStateCount() ; ++i)
        {
            const int w = std::min(maxWidth_, (int)p.getState(i)->as<ob::FromPropagatorStateSpace<ob::SE2StateSpace>::StateType>()->getX());
            const int h = std::min(maxHeight_, (int)p.getState(i)->as<ob::FromPropagatorStateSpace<ob::SE2StateSpace>::StateType>()->getY());
            ompl::PPM::Color &c = ppm_.getPixel(h, w);
            c.red = 255;
            c.green = 0;
            c.blue = 0;
        }
    }

    void save(const char *filename)
    {
        if (!ss_)
            return;
        ppm_.saveFile(filename);
    }

private:

    bool isStateValid(const ob::State *state) const
    {
        const int w = std::min((int)state->as<ob::FromPropagatorStateSpace<ob::SE2StateSpace>::StateType>()->getX(), maxWidth_);
        const int h = std::min((int)state->as<ob::FromPropagatorStateSpace<ob::SE2StateSpace>::StateType>()->getY(), maxHeight_);

        const ompl::PPM::Color &c = ppm_.getPixel(h, w);
        return c.red > 127 && c.green > 127 && c.blue > 127;
    }

    og::SimpleSetupPtr ss_;
    oc::SpaceInformationPtr siC_;
    int maxWidth_;
    int maxHeight_;
    ompl::PPM ppm_;

};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
    boost::filesystem::path path(TEST_RESOURCES_DIR);
    CarIn2D env((path / "ppm/floor.ppm").string().c_str());

    if (env.plan(boost::lexical_cast<int>(argv[1]),boost::lexical_cast<int>(argv[2]), boost::lexical_cast<double>(argv[3])))
    {
        env.recordSolution();
        env.save("result_demo.ppm");
    }

    return 0;
}
