/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, Rice University
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
 *   * Neither the name of Rice University nor the names of its
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

/* Author: Sahruday Patti */

#define BOOST_TEST_MODULE VampSetupTest
#include <boost/test/unit_test.hpp>

#include <ompl/vamp/VAMPSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>

#include <vamp/collision/factory.hh>
#include <vamp/robots/panda.hh>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using Robot = vamp::robots::Panda;

static auto makeTestEnvironment() -> vamp::collision::Environment<float>
{
    vamp::collision::Environment<float> env;
    env.spheres.emplace_back(vamp::collision::factory::sphere::array({0.5f, 0.f, 0.5f}, 0.1f));
    env.spheres.emplace_back(vamp::collision::factory::sphere::array({-0.3f, 0.3f, 0.3f}, 0.15f));
    env.sort();
    return env;
}

static const std::vector<double> START = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
static const std::vector<double> GOAL = {2.35, 1., 0., -0.8, 0., 2.5, 0.785};

BOOST_AUTO_TEST_CASE(VAMPSetupConstruction)
{
    auto env = makeTestEnvironment();
    ompl::vamp::VAMPSetup<Robot> setup(env);

    BOOST_CHECK(setup.getSpaceInformation() != nullptr);
    BOOST_CHECK(setup.getStateSpace() != nullptr);
    BOOST_CHECK_EQUAL(setup.getStateSpace()->getDimension(), Robot::dimension);
}

BOOST_AUTO_TEST_CASE(VAMPSetupRRTConnect)
{
    auto env = makeTestEnvironment();
    ompl::vamp::VAMPSetup<Robot> setup(env);
    setup.setStartGoal(START, GOAL);
    setup.setPlanner(std::make_shared<og::RRTConnect>(setup.getSpaceInformation()));

    ob::PlannerStatus status = setup.solve(5.0);
    BOOST_CHECK(status == ob::PlannerStatus::EXACT_SOLUTION);
    BOOST_CHECK(setup.getSolutionPath().getStateCount() > 1);
}

BOOST_AUTO_TEST_CASE(VAMPSetupRRT)
{
    auto env = makeTestEnvironment();
    ompl::vamp::VAMPSetup<Robot> setup(env);
    setup.setStartGoal(START, GOAL);
    setup.setPlanner(std::make_shared<og::RRT>(setup.getSpaceInformation()));

    ob::PlannerStatus status = setup.solve(5.0);
    BOOST_CHECK(status == ob::PlannerStatus::EXACT_SOLUTION);
}

BOOST_AUTO_TEST_CASE(VAMPSetupKPIECE)
{
    auto env = makeTestEnvironment();
    ompl::vamp::VAMPSetup<Robot> setup(env);
    setup.setStartGoal(START, GOAL);
    setup.setPlanner(std::make_shared<og::KPIECE1>(setup.getSpaceInformation()));

    ob::PlannerStatus status = setup.solve(5.0);
    BOOST_CHECK(status == ob::PlannerStatus::EXACT_SOLUTION);
}

BOOST_AUTO_TEST_CASE(VAMPSetupBITstar)
{
    auto env = makeTestEnvironment();
    ompl::vamp::VAMPSetup<Robot> setup(env);
    setup.setStartGoal(START, GOAL);
    setup.setPlanner(std::make_shared<og::BITstar>(setup.getSpaceInformation()));

    ob::PlannerStatus status = setup.solve(5.0);
    BOOST_CHECK(status == ob::PlannerStatus::EXACT_SOLUTION);
}

BOOST_AUTO_TEST_CASE(VAMPSetupClearAndReuse)
{
    auto env = makeTestEnvironment();
    ompl::vamp::VAMPSetup<Robot> setup(env);

    // First solve
    setup.setStartGoal(START, GOAL);
    setup.setPlanner(std::make_shared<og::RRTConnect>(setup.getSpaceInformation()));
    ob::PlannerStatus status1 = setup.solve(5.0);
    BOOST_CHECK(status1 == ob::PlannerStatus::EXACT_SOLUTION);

    // Clear and reuse with a different planner
    setup.clear();
    setup.setStartGoal(START, GOAL);
    setup.setPlanner(std::make_shared<og::RRT>(setup.getSpaceInformation()));
    ob::PlannerStatus status2 = setup.solve(5.0);
    BOOST_CHECK(status2 == ob::PlannerStatus::EXACT_SOLUTION);
}
