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
#include <boost/test/included/unit_test.hpp>

#include "2DcirclesSetup.h"
#include "2DArticulatedSetup.h"
#include "2DsdfSetup.h"
#include "../../resources/sdf2D.h"
#include <iostream>

#include "ompl/base/samplers/GradientObstacleValidStateSampler.h"
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

BOOST_AUTO_TEST_CASE(geometric_obstacleSampling)
{
    Circles2D circles;
    boost::filesystem::path path(TEST_RESOURCES_DIR);
    circles.loadCircles((path / "circle_obstacles.txt").string());
    circles.loadQueries((path / "circle_queries.txt").string());

    // Get a statespace /sampler.
    ompl::base::SpaceInformationPtr si = ompl::geometric::spaceInformation2DCircles(circles);
    ompl::base::ValidStateSamplerAllocator alloc = [](const ompl::base::SpaceInformation *space_info)
    {
        return std::make_shared<ompl::base::GradientObstacleValidStateSampler>(space_info);
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
 
    std::cout << "Obstacle Gradient sampling in point space worked for " << worked << " of " 
              << total << " times, or " << (1.0 * worked) / (1.0 * total) * 100 << "%" 
              << std::endl;
}

BOOST_AUTO_TEST_CASE(geometric_articulatedObstacleSampling)
{
    Circles2D circles;
    boost::filesystem::path path(TEST_RESOURCES_DIR);
    circles.loadCircles((path / "artic_obstacles.txt").string());
    circles.loadQueries((path / "artic_queries.txt").string());
    Articulated2D environment(&circles);

    ompl::base::SpaceInformationPtr si = ompl::geometric::spaceInformation2DArticulated(environment);
    ompl::base::ValidStateSamplerAllocator alloc = [](const ompl::base::SpaceInformation *space_info)
    {
        return std::make_shared<ompl::base::GradientObstacleValidStateSampler>(space_info);
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

    std::cout << "Obstacle gradient sampling in articulated space worked for " << worked << " of "
              << total << " times, or " << (1.0 * worked) / (1.0 * total) * 100 << "%" 
              << std::endl;
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

BOOST_AUTO_TEST_CASE(geometric_articulatedObstaclePlanning)
{
    Circles2D circles;
    boost::filesystem::path path(TEST_RESOURCES_DIR);
    circles.loadCircles((path / "artic_obstacles.txt").string());
    circles.loadQueries((path / "artic_queries.txt").string());
    Articulated2D environment(&circles);

    ompl::base::SpaceInformationPtr si = ompl::geometric::spaceInformation2DArticulated(environment);
    ompl::base::ValidStateSamplerAllocator alloc = [](const ompl::base::SpaceInformation *space_info)
    {
        return std::make_shared<ompl::base::GradientObstacleValidStateSampler>(space_info);
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
    for (std::size_t i = 0; i < circles.getQueryCount(); i++)
    {
        const Circles2D::Query &q = circles.getQuery(i);
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

BOOST_AUTO_TEST_CASE(geometric_sdf_vs_circles)
{
    Circles2D circles;
    boost::filesystem::path path(TEST_RESOURCES_DIR);
    circles.loadCircles((path / "circle_obstacles.txt").string());
    circles.loadQueries((path / "circle_queries.txt").string());

    SignedDistanceField2D sdf(circles.minX_, circles.maxX_, circles.minY_, circles.maxY_);
    auto isValid = [circles](double x, double y, int &i)
    {
        return circles.noOverlap(x, y, i);
    };
    sdf.calculateSignedDistance(0.1, isValid);

    ompl::base::SpaceInformationPtr si1 = ompl::geometric::spaceInformation2DSdf(sdf);
    auto svc1 = si1->getStateValidityChecker();
    ompl::base::SpaceInformationPtr si2 = ompl::geometric::spaceInformation2DCircles(circles);
    auto svc2 = si2->getStateValidityChecker();
    ompl::base::StateSamplerPtr sampler = si1->allocStateSampler();

    // Do like 20 different samples, see what happens.
    ompl::base::State *state = si1->allocState();
    int total = 40;
    for (int i = 0; i < total; i++)
    {
        sampler->sampleUniform(state);
        Eigen::MatrixXd grad(1, 2);
        Eigen::MatrixXd grad2(1, 2);
        bool aval;
        double dist1 = svc1->clearanceWithClosestGradient(state, grad, aval);
        double dist2 = svc2->clearanceWithClosestGradient(state, grad2, aval);
        BOOST_CHECK(std::abs(dist1 - dist2) < 0.5);
        BOOST_CHECK(std::abs(grad(0) - grad2(0)) < 0.8);
        BOOST_CHECK(std::abs(grad(1) - grad2(1)) < 0.8);
    }
}

BOOST_AUTO_TEST_CASE(geometric_medialAxis_Samples)
{
    Circles2D circles;
    boost::filesystem::path path(TEST_RESOURCES_DIR);
    circles.loadCircles((path / "circle_obstacles.txt").string());
    circles.loadQueries((path / "circle_queries.txt").string());

    SignedDistanceField2D sdf(circles.minX_, circles.maxX_, circles.minY_, circles.maxY_);
    auto isValid = [circles](double x, double y, int &i)
    {
        return circles.noOverlap(x, y, i);
    };
    sdf.calculateSignedDistance(0.1, isValid);

    // Get a statespace /sampler.
    ompl::base::SpaceInformationPtr si = ompl::geometric::spaceInformation2DSdf(sdf);
    ompl::base::ValidStateSamplerAllocator alloc = [](const ompl::base::SpaceInformation *space_info)
    {
        return std::make_shared<ompl::base::GradientMedialAxisValidStateSampler>(space_info, 0.3);
    };
    si->setValidStateSamplerAllocator(alloc);
    ompl::base::ValidStateSamplerPtr gradientSampler = si->allocValidStateSampler();

    // Do like 20 different samples, see what happens.
    ompl::base::State *state = si->allocState();
    int total = 10000;
    int worked = 0;
    for (int i = 0; i < total; i++)
    {
        //bool success = gradientSampler->sample(state);
        //worked = (success) ? worked + 1 : worked;
    }
 
    std::cout << "Gradient sampling in point space worked for " << worked << " of " << total << 
                 " times, or " << (1.0 * worked) / (1.0 * total) * 100 << "%" << std::endl;
}

BOOST_AUTO_TEST_CASE(geometric_articulatedMedialSampling)
{
    Circles2D circles;
    boost::filesystem::path path(TEST_RESOURCES_DIR);
    circles.loadCircles((path / "artic_obstacles.txt").string());
    circles.loadQueries((path / "artic_queries.txt").string());
    
    SignedDistanceField2D sdf(circles.minX_, circles.maxX_, circles.minY_, circles.maxY_);
    auto isValid = [circles](double x, double y, int &i)
    {
        return circles.noOverlap(x, y, i);
    };
    sdf.calculateSignedDistance(0.1, isValid);

    Articulated2D environment(&sdf);

    ompl::base::SpaceInformationPtr si = ompl::geometric::spaceInformation2DArticulated(environment);
    ompl::base::ValidStateSamplerAllocator alloc = [](const ompl::base::SpaceInformation *space_info)
    {
        return std::make_shared<ompl::base::GradientObstacleValidStateSampler>(space_info);
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

    std::cout << "Obstacle gradient sampling in articulated space worked for " << worked << " of "
              << total << " times, or " << (1.0 * worked) / (1.0 * total) * 100 << "%" 
              << std::endl;
}