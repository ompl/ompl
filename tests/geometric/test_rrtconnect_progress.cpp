/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Rice University
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
 *   * Neither the name of the copyright holder nor the names of its
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

/* Author: Clayton W. Ramsey */

// Regression test for RRTConnect's connect loop never ending.
// The loop keeps growing the other tree toward a state for as long as each step advances.
// In a space whose interpolated state lands farther from the target than its source, every step finds the same
// nearest motion and adds the same state again, and the loop never reads the termination condition.

#define BOOST_TEST_MODULE "RRTConnect progress regression"
#include <boost/test/unit_test.hpp>

#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

#include <stdexcept>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace
{
    constexpr unsigned int INTERPOLATION_LIMIT = 100000;

    /// A space with swervy interpolations.
    class SwervingSpace : public ob::RealVectorStateSpace
    {
    public:
        SwervingSpace() : ob::RealVectorStateSpace(2)
        {
            setBounds(0., 1.);
        }

        void interpolate(const ob::State *from, const ob::State *to, double t, ob::State *state) const override
        {
            if (++interpolations_ > INTERPOLATION_LIMIT)
                throw std::runtime_error("the planner kept interpolating the same step");

            const double *a = from->as<StateType>()->values;
            const double *b = to->as<StateType>()->values;
            double *out = state->as<StateType>()->values;
            const double dx = b[0] - a[0];
            const double dy = b[1] - a[1];
            out[0] = a[0] - t * dy;
            out[1] = a[1] + t * dx;
        }

    private:
        mutable unsigned int interpolations_ = 0;
    };
}  // namespace

BOOST_AUTO_TEST_CASE(ConnectingStopsWhenAStepGetsNoNearer)
{
    auto space = std::make_shared<SwervingSpace>();
    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker([](const ob::State *) { return true; });
    si->setup();

    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    ob::ScopedState<> start(space), goal(space);
    start[0] = 0.1;
    start[1] = 0.5;
    goal[0] = 0.9;
    goal[1] = 0.5;
    pdef->setStartAndGoalStates(start, goal);

    auto planner = std::make_shared<og::RRTConnect>(si);
    planner->setRange(0.1);
    planner->setProblemDefinition(pdef);
    planner->setup();

    unsigned int iterations = 0;
    const ob::PlannerTerminationCondition ptc([&iterations] { return ++iterations > 1000; });
    BOOST_CHECK_NO_THROW(planner->solve(ptc));
}
