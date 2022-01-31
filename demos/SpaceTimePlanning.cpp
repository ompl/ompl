/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2021, Technische Universität Berlin (TU Berlin)
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
*   * Neither the name of the TU Berlin nor the names of its
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

/* Author: Francesco Grothe */

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SpaceTimeStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/STRRTstar.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

/**
 * Demonstration of planning through space time using the Animation state space and the SpaceTimeRRT* planner.
 */

bool isStateValid(const ob::State *state)
{
    // extract the space component of the state and cast it to what we expect
    const auto pos = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(0)->values[0];

    // extract the time component of the state and cast it to what we expect
    const auto t = state->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;

    // check validity of state defined by pos & t (e.g. check if constraints are satisfied)...

    // return a value that is always true
    return t >= 0 && pos < std::numeric_limits<double>::infinity();
}

class SpaceTimeMotionValidator : public ob::MotionValidator {

public:
    explicit SpaceTimeMotionValidator(const ob::SpaceInformationPtr &si) : MotionValidator(si),
      vMax_(si_->getStateSpace().get()->as<ob::SpaceTimeStateSpace>()->getVMax()),
      stateSpace_(si_->getStateSpace().get()) {};

    bool checkMotion(const ob::State *s1, const ob::State *s2) const override
    {
        // assume motion starts in a valid configuration, so s1 is valid
        if (!si_->isValid(s2)) {
            invalid_++;
            return false;
        }

        // check if motion is forward in time and is not exceeding the speed limit
        auto *space = stateSpace_->as<ob::SpaceTimeStateSpace>();
        auto deltaPos = space->distanceSpace(s1, s2);
        auto deltaT = s2->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position -
                      s1->as<ob::CompoundState>()->as<ob::TimeStateSpace::StateType>(1)->position;

        if (!(deltaT > 0 && deltaPos / deltaT <= vMax_)) {
            invalid_++;
            return false;
        }

        // check if the path between the states is unconstrained (perform interpolation)...

        return true;
    }

    bool checkMotion(const ompl::base::State *, const ompl::base::State *,
                     std::pair<ob::State *, double> &) const override
    {
        throw ompl::Exception("SpaceTimeMotionValidator::checkMotion", "not implemented");
    }

private:
    double vMax_; // maximum velocity
    ob::StateSpace *stateSpace_; // the animation state space for distance calculation
};

void plan(void)
{
    // set maximum velocity
    double vMax = 0.2;

    // construct the state space we are planning in
    auto vectorSpace(std::make_shared<ob::RealVectorStateSpace>(1));
    auto space = std::make_shared<ob::SpaceTimeStateSpace>(vectorSpace, vMax);

    // set the bounds for R1
    ob::RealVectorBounds bounds(1);
    bounds.setLow(-1.0);
    bounds.setHigh(1.0);
    vectorSpace->setBounds(bounds);

    // set time bounds. Planning with unbounded time is also possible when using ST-RRT*.
    space->setTimeBounds(0.0, 10.0);

    // create the space information class for the space
    ob::SpaceInformationPtr si = std::make_shared<ob::SpaceInformation>(space);

    // set state validity checking for this space
    si->setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });
    si->setMotionValidator(std::make_shared<SpaceTimeMotionValidator>(si));

    // define a simple setup class
    og::SimpleSetup ss(si);

    // create a start state
    ob::ScopedState<> start(space);
    start[0] = 0; // pos

    // create a goal state
    ob::ScopedState<> goal(space);
    goal[0] = 1; // pos

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // construct the planner
    auto *strrtStar = new og::STRRTstar(si);

    // set planner parameters
    strrtStar->setRange(vMax);

    // set the used planner
    ss.setPlanner(ob::PlannerPtr(strrtStar));

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve(1.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen
        ss.getSolutionPath().print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;

}

int main(int /*argc*/, char ** /*argv*/)
{
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    plan();

    return 0;
}