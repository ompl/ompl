/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Rice University
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

#define BOOST_TEST_MODULE "GeometricPlanningOptimizingPlanners"
#include <boost/test/unit_test.hpp>

#include "2DcirclesSetup.h"
#include <iostream>

#include "ompl/geometric/planners/trajopt/TrajOpt.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/objectives/JointDistanceObjective.h"
#include "ompl/base/objectives/ConvexifiableOptimization.h"
#include "ompl/base/objectives/ObstacleConstraint.h"
#include "ompl/base/goals/GoalState.h"

using namespace ompl;

class TestPlanner
{
public:
    TestPlanner()
    {
        msg::setLogLevel(msg::LOG_ERROR);
    }

    void test2DCircles(const Circles2D& circles, double safetyDistance, int waypoints)
    {
        base::SpaceInformationPtr si = geometric::spaceInformation2DCircles(circles);
        auto pdef(std::make_shared<base::ProblemDefinition>(si));
        auto opt(std::make_shared<base::MultiConvexifiableOptimization>(si));
        opt->addObjective(std::make_shared<base::JointDistanceObjective>(si));
        ompl::base::WorkspaceCollisionFn collisions = [circles, safetyDistance](std::vector<double> configuration,
                                                             std::vector<double>& signedDist,
                                                             std::vector<Eigen::Vector3d>& point,
                                                             std::vector<std::string>& link_name,
                                                             std::vector<Eigen::Vector3d>& normal)
        {
            // The configuration is the position of the circle.
            double x = configuration[0];
            double y = configuration[1];
            signedDist.push_back(circles.signedDistance(x, y));
            // TODO: vector3d should be changed to the workspace dimension.
            point.push_back(Eigen::Vector3d(x, y, 0));
            // ignore link name, I guess.
            link_name.push_back("point");
            // Normals move the object out of collision.
            normal.push_back(circles.minimalTranslationNormal(x, y));
            return signedDist[0] <= safetyDistance * 2;
        };

        ompl::base::WorkspaceContinuousCollisionFn continuousCollisions = [circles, safetyDistance](std::vector<double> configuration0,
                                                                std::vector<double> configuration1,
                                                                std::vector<double>& signedDist,
                                                                std::vector<Eigen::Vector3d>& point_swept,
                                                                std::vector<Eigen::Vector3d>& point0,
                                                                std::vector<Eigen::Vector3d>& point1,
                                                                std::vector<std::string>& link_name,
                                                                std::vector<Eigen::Vector3d>& normal)
        {
            double x1 = configuration0[0], x2 = configuration1[0];
            double y1 = configuration0[1], y2 = configuration1[1];
            Eigen::Vector2d closestPoint;
            signedDist.push_back(circles.lineSignedDistance(x1, y1, x2, y2, closestPoint));
            point_swept.push_back(Eigen::Vector3d(closestPoint[0], closestPoint[1], 0));
            point0.push_back(Eigen::Vector3d(x1, y1, 0));
            point1.push_back(Eigen::Vector3d(x2, y2, 0));
            link_name.push_back("point");
            normal.push_back(circles.minimalTranslationNormal(closestPoint[0], closestPoint[1]));
            return signedDist[0] <= safetyDistance * 2;
        };

        ompl::base::JacobianFn jacobian = [circles](std::vector<double> configuration,
                                                    Eigen::Vector3d point, std::string link)
        {
            // Ignore the point and link. There's one point and one link.
            // The Jacobian of a point
            Eigen::MatrixXd j(3, 2);
            j << 1, 0,
                 0, 1,
                 0, 0;
            return j;
        };
        //opt->addObjective(std::make_shared<base::ObstacleConstraint>(si, safetyDistance));
        //opt->addObjective(std::make_shared<base::ObstacleConstraint>(si, safetyDistance, collisions, jacobian));
        opt->addObjective(std::make_shared<base::ObstacleConstraint>(si, safetyDistance, continuousCollisions, jacobian));
        pdef->setOptimizationObjective(opt);

        /* instantiate motion planner */
        auto trajopt(std::make_shared<geometric::TrajOpt>(si));
        trajopt->setTimeStepCount(waypoints);
        trajopt->setProblemDefinition(pdef);
        trajopt->setup();
        base::PlannerPtr planner = trajopt;

        std::size_t nt = std::min<std::size_t>(10, circles.getQueryCount());

        for (std::size_t i = 0 ; i < nt ; ++i)
        {
            const Circles2D::Query &q = circles.getQuery(i);
            setupProblem(q, si, pdef);

            planner->clear();
            pdef->clearSolutionPaths();

            // we change the optimization objective so the planner runs until the first solution
            opt->setCostThreshold(opt->infiniteCost());

            bool solved = planner->solve(10.0); // 10.0 is solution time.
            // TODO: assume that we should solve all queries for now.
            BOOST_CHECK(solved);
            if (solved)
            {
                geometric::PathGeometric *path = static_cast<geometric::PathGeometric*>(pdef->getSolutionPath().get());
                //std::cout << "Final path:" << std::endl;
                //path->printAsMatrix(std::cout);
            }
        }
    }


        static void setupProblem(const Circles2D::Query &q,
                                 const base::SpaceInformationPtr &si,
                                 base::ProblemDefinitionPtr &pdef)
        {
            base::ScopedState<> start(si);
            start[0] = q.startX_;
            start[1] = q.startY_;

            base::GoalPtr goal = genGoalFromQuery(q, si);

            pdef->clearStartStates();
            pdef->addStartState(start);
            pdef->clearGoal();
            pdef->setGoal(goal);
        }

        static base::GoalPtr genGoalFromQuery(const Circles2D::Query &q,
                                              const base::SpaceInformationPtr &si)
        {
            base::ScopedState<> goal2D(si);
            goal2D[0] = q.goalX_;
            goal2D[1] = q.goalY_;

            auto goalState(std::make_shared<base::GoalState>(si));
            goalState->setState(goal2D);
            goalState->setThreshold(1e-3);
            return goalState;
        }
};


BOOST_FIXTURE_TEST_SUITE(MyPlanTestFixture, TestPlanner)

BOOST_AUTO_TEST_CASE(geometric_TrajOpt)
{
    const int TRIALS = 30;

    // Test simple cases, they should all work.
    Circles2D circles;
    boost::filesystem::path path(TEST_RESOURCES_DIR);
    circles.loadCircles((path / "simple_circle_obstacles.txt").string());
    circles.loadQueries((path / "simple_circle_queries.txt").string());

    test2DCircles(circles, 0.3, 6);

    // Test the complex cases, TODO: see how much of those should work.
    Circles2D complex;
    complex.loadCircles((path / "circle_obstacles.txt").string());
    complex.loadQueries((path / "circle_queries.txt").string());

    //test2DCircles(complex, 0.1, 30);

    // Write the results in CSV in this file.
    FILE* fd = fopen("/tmp/trajopt_benchmark_waypoints.csv", "w");
    fprintf(fd, "\"waypoints\", \"average time (%d trials)\"\n", TRIALS);

    // Get performance testing on # of waypoints
    for (int waypoints = 5; waypoints < 50; waypoints+=2) {
        // Solve the simple problems 30 times.
        auto t1 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < TRIALS; i++) {
            test2DCircles(circles, 0.3, waypoints);
        }
        auto t2 = std::chrono::high_resolution_clock::now();
        auto total_time = std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count();
        auto single_time = total_time / (TRIALS * 1.0);
        std::cout << "*********** " << waypoints << " waypoints **************" << std::endl;
        std::cout << "30 times simple: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count()
                  << " milliseconds" << std::endl;
        std::cout << "On avg, 1 took "
                  << single_time
                  << " milliseconds" << std::endl;
        fprintf(fd, "%d, %f\n", waypoints, single_time);
    }

    fclose(fd);
    // TODO: see if we can compare discrete and continuous timing (with
    //       same waypoints and different waypoints.)
}

BOOST_AUTO_TEST_SUITE_END()
