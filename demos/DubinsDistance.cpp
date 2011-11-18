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

#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// The easy problem is the standard narrow passage problem: two big open
// spaces connected by a narrow passage. The hard problem is essentially
// one long narrow passage with the robot facing towards the long walls
// in both the start and goal configurations.

bool isStateValidEasy(const ob::SpaceInformation *si, const ob::State *state)
{
    const ob::DubinsStateSpace::StateType *s = state->as<ob::DubinsStateSpace::StateType>();
    double x=s->getX(), y=s->getY();
    return si->satisfiesBounds(s) && (x<5 || x>13 || (y>8.5 && y<9.5));
}

bool isStateValidHard(const ob::SpaceInformation *si, const ob::State *state)
{
    const ob::DubinsStateSpace::StateType *s = state->as<ob::DubinsStateSpace::StateType>();
    double x=s->getX(), y=s->getY();
    return si->satisfiesBounds(state);
}

void plan(ob::StateSpacePtr space, bool easy)
{
    ob::ScopedState<> start(space), goal(space);
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    if (easy)
        bounds.setHigh(18);
    else
    {
        bounds.high[0] = 6;
        bounds.high[1] = .6;
    }
    space->as<ob::DubinsStateSpace>()->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);

    // use the special Dubins motion validator
    ob::SpaceInformationPtr si(ss.getSpaceInformation());
    si->setMotionValidator(ob::MotionValidatorPtr(new ob::DubinsMotionValidator(si)));

    // set state validity checking for this space
    ss.setStateValidityChecker(boost::bind(
        easy ? &isStateValidEasy : &isStateValidHard, si.get(), _1));

    // set the start and goal states
    if (easy)
    {
        start[0] = start[1] = 1.; start[2] = 0.;
        goal[0] = goal[1] = 17; goal[2] = -.99*boost::math::constants::pi<double>();
    }
    else
    {
        start[0] = start[1] = .5; start[2] = .5*boost::math::constants::pi<double>();;
        goal[0] = 5.5; goal[1] = .5; goal[2] = .5*boost::math::constants::pi<double>();
    }
    ss.setStartAndGoalStates(start, goal);

    // this call is optional, but we put it in to get more output information
    ss.getSpaceInformation()->setStateValidityCheckingResolution(0.005);
    ss.setup();
    ss.print();

    // attempt to solve the problem within one second of planning time
    bool solved = ss.solve(100.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // We can't use regular simplify because it also tries to use spline interpolation,
        // which doesn't work for Dubins curves.
        //ss.simplifySolution();
        og::PathGeometric path = ss.getSolutionPath();
        og::PathSimplifierPtr ps = ss.getPathSimplifier();
        ps->reduceVertices(path);
        ps->collapseCloseVertices(path);
        path.interpolate(100);
        for (unsigned int i=0; i<path.states.size(); ++i)
        {
            const ob::DubinsStateSpace::StateType& s = *path.states[i]->as<ob::DubinsStateSpace::StateType>();
            std::cout << "path " << s.getX() <<' '<< s.getY() << ' ' << s.getYaw() << std::endl;
        }
    }
    else
        std::cout << "No solution found" << std::endl;
}

int main(int argc, char* argv[])
{
    const unsigned int num_pts = 200;
    ob::StateSpacePtr space(new ob::DubinsStateSpace);
    ob::ScopedState<> from(space), to(space), s(space);
    std::vector<double> reals;

    from[0] = from[1] = from[2] = 0.;

    // if 3 command line arguments are given, it will interpret them as
    // an SE(2) state, and print interpolated points along the Dubins path
    // from (0,0,0) to the input state.
    if (argc == 4)
    {
        to[0] = atof(argv[1]);
        to[1] = atof(argv[2]);
        to[2] = atof(argv[3]);

        std::cout << "distance: " << space->distance(from(), to()) << "\npath:\n";
        for (unsigned int i=0; i<=num_pts; ++i)
        {
            space->interpolate(from(), to(), (double)i/num_pts, s());
            reals = s.reals();
            std::cout << reals[0] << ' ' << reals[1] << ' ' << reals[2] << ' ' << std::endl;
        }
    }
    else if (argc == 2 && !strcmp(argv[1], "grid"))
    // if 1 dummy command line argument is given, print the Dubins distance
    // for (x,y,theta) for all points in a 3D grid in SE(2) over
    // (-5,5] x (-5, 5] x (-pi,pi].
    //
    // The output should be redirected to a file, say, distance.txt. This
    // can then be read and plotted in Matlab like so:
    //     x = reshape(load('distance.txt'),200,200,200);
    //     for i=1:200,
    //         contourf(squeeze(x(i,:,:)),30);
    //         axis equal; axis tight; colorbar; pause;
    //     end;
    {
        for (unsigned int i=0; i<num_pts; ++i)
            for (unsigned int j=0; j<num_pts; ++j)
                for (unsigned int k=0; k<num_pts; ++k)
                {
                    to[0] = 5. * (1. - 2. * (double)i/num_pts);
                    to[1] = 5. * (1. - 2. * (double)j/num_pts);
                    to[2] = boost::math::constants::pi<double>() * (1. - 2. * (double)k/num_pts);
                    std::cout << space->distance(from(), to()) << '\n';
                }
    }
    else
    // otherwise, solve a simple planning problem
    {
        bool easy = argc > 1 ? !strcmp(argv[1], "easy") : true;
        plan(space, easy);
    }
}