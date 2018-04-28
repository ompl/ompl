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

/* Author: Mark Moll */

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>
#include <iostream>
#include <thread>

namespace ob = ompl::base;
namespace og = ompl::geometric;

/// @cond IGNORE


// This is a problem-specific sampler that automatically generates valid
// states; it doesn't need to call SpaceInformation::isValid. This is an
// example of constrained sampling. If you can explicitly describe the set valid
// states and can draw samples from it, then this is typically much more
// efficient than generating random samples from the entire state space and
// checking for validity.
class MyValidStateSampler : public ob::ValidStateSampler
{
public:
    MyValidStateSampler(const ob::SpaceInformation *si) : ValidStateSampler(si)
    {
        name_ = "my sampler";
    }
    // Generate a sample in the valid part of the R^3 state space
    // Valid states satisfy the following constraints:
    // -1<= x,y,z <=1
    // if .25 <= z <= .5, then |x|>.8 and |y|>.8
    bool sample(ob::State *state) override
    {
        double* val = static_cast<ob::RealVectorStateSpace::StateType*>(state)->values;
        double z = rng_.uniformReal(-1,1);

        if (z>.25 && z<.5)
        {
            double x = rng_.uniformReal(0,1.8), y = rng_.uniformReal(0,.2);
            switch(rng_.uniformInt(0,3))
            {
                case 0: val[0]=x-1;  val[1]=y-1;  break;
                case 1: val[0]=x-.8; val[1]=y+.8; break;
                case 2: val[0]=y-1;  val[1]=x-1;  break;
                case 3: val[0]=y+.8; val[1]=x-.8; break;
            }
        }
        else
        {
            val[0] = rng_.uniformReal(-1,1);
            val[1] = rng_.uniformReal(-1,1);
        }
        val[2] = z;
        assert(si_->isValid(state));
        return true;
    }
    // We don't need this in the example below.
    bool sampleNear(ob::State* /*state*/, const ob::State* /*near*/, const double /*distance*/) override
    {
        throw ompl::Exception("MyValidStateSampler::sampleNear", "not implemented");
        return false;
    }
protected:
    ompl::RNG rng_;
};

/// @endcond

// this function is needed, even when we can write a sampler like the one
// above, because we need to check path segments for validity
bool isStateValid(const ob::State *state)
{
    const ob::RealVectorStateSpace::StateType& pos = *state->as<ob::RealVectorStateSpace::StateType>();
    // Let's pretend that the validity check is computationally relatively
    // expensive to emphasize the benefit of explicitly generating valid
    // samples
    std::this_thread::sleep_for(ompl::time::seconds(.0005));
    // Valid states satisfy the following constraints:
    // -1<= x,y,z <=1
    // if .25 <= z <= .5, then |x|>.8 and |y|>.8
    return !(fabs(pos[0])<.8 && fabs(pos[1])<.8 && pos[2]>.25 && pos[2]<.5);
}

// return an obstacle-based sampler
ob::ValidStateSamplerPtr allocOBValidStateSampler(const ob::SpaceInformation *si)
{
    // we can perform any additional setup / configuration of a sampler here,
    // but there is nothing to tweak in case of the ObstacleBasedValidStateSampler.
    return std::make_shared<ob::ObstacleBasedValidStateSampler>(si);
}

// return an instance of my sampler
ob::ValidStateSamplerPtr allocMyValidStateSampler(const ob::SpaceInformation *si)
{
    return std::make_shared<MyValidStateSampler>(si);
}


void plan(int samplerIndex)
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(3));

    // set the bounds
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);
    space->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    // set state validity checking for this space
    ss.setStateValidityChecker(isStateValid);

    // create a start state
    ob::ScopedState<> start(space);
    start[0] = start[1] = start[2] = 0;

    // create a goal state
    ob::ScopedState<> goal(space);
    goal[0] = goal[1] = 0.;
    goal[2] = 1;

    // set the start and goal states
    ss.setStartAndGoalStates(start, goal);

    // set sampler (optional; the default is uniform sampling)
    if (samplerIndex==1)
        // use obstacle-based sampling
        ss.getSpaceInformation()->setValidStateSamplerAllocator(allocOBValidStateSampler);
    else if (samplerIndex==2)
        // use my sampler
        ss.getSpaceInformation()->setValidStateSamplerAllocator(allocMyValidStateSampler);

    // create a planner for the defined space
    auto planner(std::make_shared<og::PRM>(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    // attempt to solve the problem within ten seconds of planning time
    ob::PlannerStatus solved = ss.solve(10.0);
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
    std::cout << "Using default uniform sampler:" << std::endl;
    plan(0);
    std::cout << "\nUsing obstacle-based sampler:" << std::endl;
    plan(1);
    std::cout << "\nUsing my sampler:" << std::endl;
    plan(2);

    return 0;
}
