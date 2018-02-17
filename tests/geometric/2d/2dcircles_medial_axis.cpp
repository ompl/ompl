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

/* Authors: Bryce Willey */

#define BOOST_TEST_MODULE "GeometricPlanningMedialAxisSampling"
#include <boost/test/unit_test.hpp>

#include "2DcirclesSetup.h"
#include "2DArticulatedSetup.h"
#include "../../resources/sdf2D.h"
#include <iostream>

#include "ompl/base/samplers/GradientMedialAxisValidStateSampler.h"
#include "ompl/geometric/planners/prm/PRM.h"
#include "ompl/base/goals/GoalState.h"

namespace ompl
{
class TestPlanner
{
public:
    TestPlanner()
    {

    }
};
}

BOOST_AUTO_TEST_CASE(geometric_medialAxisSampling)
{
    Circles2D circles;
    boost::filesystem::path path(TEST_RESOURCES_DIR);
    circles.loadCircles((path / "circle_obstacles.txt").string());
    circles.loadQueries((path / "circle_queries.txt").string());

    // Get a statespace /sampler.
    ompl::base::SpaceInformationPtr si = ompl::geometric::spaceInformation2DCircles(circles);
    ompl::base::ValidStateSamplerAllocator alloc = [](const ompl::base::SpaceInformation *space_info)
    {
        return std::make_shared<ompl::base::GradientMedialAxisValidStateSampler>(space_info);
    };
    si->setValidStateSamplerAllocator(alloc);
    ompl::base::ValidStateSamplerPtr gradientSampler = si->allocValidStateSampler();

    // Do like 20 different samples, see what happens.
    ompl::base::State *state = si->allocState();
    int total = 200;
    int worked = 0;
    for (int i = 0; i < total; i++)
    {
        bool success = gradientSampler->sample(state);
        worked = (success) ? worked + 1 : worked;
    }
 
    std::cout << "Gradient sampling in point space worked for " << worked << " of " << total << 
                 " times, or " << (1.0 * worked) / (1.0 * total) * 100 << "%" << std::endl;
}

BOOST_AUTO_TEST_CASE(geometric_articulatedMedialAxisSampling)
{
    Articulated2D environment;
    boost::filesystem::path path(TEST_RESOURCES_DIR);
    environment.loadCircles((path / "artic_obstacles.txt").string());
    environment.loadQueries((path / "artic_queries.txt").string());

    ompl::base::SpaceInformationPtr si = ompl::geometric::spaceInformation2DArticulated(environment);
    ompl::base::ValidStateSamplerAllocator alloc = [](const ompl::base::SpaceInformation *space_info)
    {
        return std::make_shared<ompl::base::GradientMedialAxisValidStateSampler>(space_info);
    };
    si->setValidStateSamplerAllocator(alloc);
    ompl::base::ValidStateSamplerPtr gradientSampler = si->allocValidStateSampler();

    ompl::base::State *state = si->allocState();
    int total = 200;
    int worked = 0;
    for (int i = 0; i < total; i++)
    {
        bool success = gradientSampler->sample(state);
        worked = (success) ? worked + 1 : worked;
    }

    std::cout << "Gradient sampling in articulated space worked for " << worked << " of " << total << 
                 " times, or " << (1.0 * worked) / (1.0 * total) * 100 << "%" << std::endl;
}

ompl::base::GoalPtr genGoalFromQuery(const Articulated2D::Query &q,
                                      const ompl::base::SpaceInformationPtr &si)
{
    ompl::base::ScopedState<> goal2D(si);
    goal2D[0] = q.goal_one_;
    goal2D[1] = q.goal_two_;

    auto goalState(std::make_shared<ompl::base::GoalState>(si));
    goalState->setState(goal2D);
    goalState->setThreshold(1e-3);
    return goalState;
}


void setupProblem(const Articulated2D::Query &q,
                  const ompl::base::SpaceInformationPtr &si,
                  ompl::base::ProblemDefinitionPtr &pdef)
{
    ompl::base::ScopedState<> start(si);
    start[0] = q.start_one_;
    start[1] = q.start_two_;

    ompl::base::GoalPtr goal = genGoalFromQuery(q, si);

    pdef->clearStartStates();
    pdef->addStartState(start);
    pdef->clearGoal();
    pdef->setGoal(goal);
}

BOOST_AUTO_TEST_CASE(geometric_articulatedMedialAxisPlanning)
{
    Articulated2D environment;
    boost::filesystem::path path(TEST_RESOURCES_DIR);
    environment.loadCircles((path / "artic_obstacles.txt").string());
    environment.loadQueries((path / "artic_queries.txt").string());

    ompl::base::SpaceInformationPtr si = ompl::geometric::spaceInformation2DArticulated(environment);
    ompl::base::ValidStateSamplerAllocator alloc = [](const ompl::base::SpaceInformation *space_info)
    {
        return std::make_shared<ompl::base::GradientMedialAxisValidStateSampler>(space_info);
    };
    si->setValidStateSamplerAllocator(alloc);

    // Setup a planner and plan: PRM for starters.
    auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
    auto prm(std::make_shared<ompl::geometric::PRM>(si));
    prm->setProblemDefinition(pdef);
    prm->setup();
    ompl::base::PlannerPtr planner = prm;

    std::ofstream of;
    of.open("/tmp/tmpfile.txt");
    for (std::size_t i = 0; i < environment.getQueryCount(); i++)
    {
        const Articulated2D::Query &q = environment.getQuery(i);
        setupProblem(q, si, pdef);

        planner->clear();
        pdef->clearSolutionPaths();

        bool solved = planner->solve(10.0);
        BOOST_CHECK(solved);
        if (solved)
        {
            ompl::geometric::PathGeometric *path = static_cast<ompl::geometric::PathGeometric*>(pdef->getSolutionPath().get());
            path->printAsMatrix(of);
        }
    }
    of.close();
}

BOOST_AUTO_TEST_CASE(geometric_calc_medialAxis)
{
    Circles2D circles;
    boost::filesystem::path path(TEST_RESOURCES_DIR);
    circles.loadCircles((path / "circle_obstacles.txt").string());
    circles.loadQueries((path / "circle_queries.txt").string());

    SignedDistanceField2D sdf(circles.minX_, circles.maxX_, circles.minY_, circles.maxY_);
    std::function<bool(double, double)> isValid = [circles](double x, double y)
    {
        return circles.noOverlap(x, y);
    };
    sdf.calculateSignedDistance(0.1, isValid);
}