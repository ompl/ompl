/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Rice University
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

/* Author: Zachary Kingston */

#include <iostream>
#include <fstream>

#include <boost/program_options.hpp>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>

#include <ompl/base/Constraint.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/spaces/constraint/AtlasChart.h>
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>

#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/est/BiEST.h>

#include <ompl/geometric/planners/prm/PRM.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

#include <ompl/geometric/planners/bitstar/BITstar.h>

#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>

using namespace std;
using namespace placeholders;
using namespace ompl::base;
using namespace ompl::geometric;
using namespace boost::program_options;
using namespace Eigen;

class SphereConstraint : public Constraint
{
public:
    SphereConstraint() : Constraint(3, 1)
    {
    }

    void function(const Ref<const VectorXd> &x, Ref<VectorXd> out) const override
    {
        out[0] = x.norm() - 1;
    }

    void jacobian(const Ref<const VectorXd> &x, Ref<MatrixXd> out) const override
    {
        out = x.transpose().normalized();
    }
};

bool isValid(const State *state)
{
    auto x = state->as<ConstrainedStateSpace::StateType>()->constVectorView();

    if (-0.75 < x[2] && x[2] < -0.60)
    {
        if (-0.05 < x[1] && x[1] < 0.05)
            return x[0] > 0;
        return false;
    }
    else if (-0.1 < x[2] && x[2] < 0.1)
    {
        if (-0.05 < x[0] && x[0] < 0.05)
            return x[1] < 0;
        return false;
    }
    else if (0.60 < x[2] && x[2] < 0.75)
    {
        if (-0.05 < x[1] && x[1] < 0.05)
            return x[0] < 0;
        return false;
    }

    return true;
}

int main(int argc, char **argv)
{
    // Create the ambient space state space for the problem.
    StateSpacePtr rvss(new RealVectorStateSpace(3));

    RealVectorBounds bounds(3);
    bounds.setLow(-1);
    bounds.setHigh(1);

    rvss->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

    // Create a shared pointer to our constraint.
    ConstraintPtr constraint(new SphereConstraint);

    // Combine the ambient state space and the constraint to create the
    // constrained state space.
    ConstrainedStateSpacePtr css(new ProjectedStateSpace(rvss, constraint));

    ConstrainedSpaceInformationPtr csi(new ConstrainedSpaceInformation(css));
    SimpleSetupPtr ss(new SimpleSetup(csi));

    ss->setStateValidityChecker(bind(isValid, _1));

    csi->setValidStateSamplerAllocator([](const SpaceInformation *si) -> ValidStateSamplerPtr {
        return ValidStateSamplerPtr(new ConstrainedValidStateSampler(si));
    });

    ScopedState<> start(css);
    ScopedState<> goal(css);
    start->as<ProjectedStateSpace::StateType>()->vectorView() << 0, 0, -1;
    goal->as<ProjectedStateSpace::StateType>()->vectorView() << 0, 0, 1;

    ss->setStartAndGoalStates(start, goal);

    PlannerPtr planner(new RRTConnect(csi));
    ss->setPlanner(planner);

    ss->setup();

    auto tstart = clock();
    PlannerStatus stat = ss->solve(5.);
    if (stat)
    {
        const double time = ((double)(clock() - tstart)) / CLOCKS_PER_SEC;
        cout << "Took " << time << " seconds." << endl;

        auto path = ss->getSolutionPath();
        if (!css->checkPath(path))
            cout << "Path does not satisfy constraints!" << endl;

        cout << "Simplifying solution..." << endl;
        double originalLength = path.length();
        ss->simplifySolution(5.);

        cout << "Path Length " << originalLength << " -> " << path.length() << endl;

        if (!css->checkPath(path))
            cout << "Simplified path does not satisfy constraints!" << endl;

        if (stat == PlannerStatus::APPROXIMATE_SOLUTION)
            cout << "Solution is approximate." << endl;

        cout << "Interpolating path..." << endl;
        path.interpolate();

        if (!css->checkPath(path))
            cout << "Interpolated path does not satisfy constraints!" << endl;

        cout << "Dumping animation file..." << endl;

        ofstream file("sphere_path.txt");
        path.printAsMatrix(file);
        file.close();
    }
    else
        cout << "No solution found." << endl;

    return 0;
}
