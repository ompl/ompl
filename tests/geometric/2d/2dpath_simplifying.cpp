/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2018, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Bryce Willey */

#define BOOST_TEST_MODULE "GeometricPlanningSimplifying"
#include <boost/test/unit_test.hpp>

#include "2DcirclesSetup.h"
#include <iostream>

#include "ompl/base/Goal.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/geometric/PathHybridization.h"

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/objectives/MaximizeMinClearanceObjective.h"

#include "../../base/PlannerTest.h"

using namespace ompl;

static const bool VERBOSE = true;


class SimplifyTest
{
public:

    SimplifyTest()
    {
        verbose_ = VERBOSE;
        boost::filesystem::path path(TEST_RESOURCES_DIR);
        circles_.loadCircles((path / "circle_obstacles.txt").string());
        si_ = geometric::spaceInformation2DCircles(circles_);
        paths_ = geometric::readPathsFromFile(si_, (path / "circle_paths_to_simplify.txt").string());
    }

    template<typename T>
    void run_simplifier(int runs)
    {
        base::OptimizationObjectivePtr obj(new T(si_));
        geometric::PathSimplifier simplifier(si_, ompl::base::GoalPtr(), obj);
        for (int path_idx = 0; path_idx < 2; path_idx++)
        {
            double avg_costs = 0.0;
            base::Cost original_cost = paths_[path_idx]->cost(obj);
            geometric::PathGeometric *path;
            for (int i = 0; i < runs; i++)
            {
                path = new geometric::PathGeometric(*paths_[path_idx]);
                simplifier.shortcutPath(*path, 100, 100, 0.33, 0.005);
                avg_costs += path->cost(obj).value();
            }
            avg_costs /= runs;
            printf("Average cost: %f, original cost: %f\n", avg_costs, original_cost.value());
            BOOST_CHECK(obj->isCostBetterThan(base::Cost(avg_costs), original_cost) ||
                        obj->isCostEquivalentTo(base::Cost(avg_costs), original_cost));
        }
    }

    template<typename T>
    void run_hybridizer()
    {
        base::OptimizationObjectivePtr obj(new T(si_));
        geometric::PathHybridization hybrid(si_, obj);
        std::cout << hybrid.getName() << std::endl;
        geometric::PathGeometricPtr path1(paths_[0]);
        geometric::PathGeometricPtr path2(paths_[1]);
        hybrid.recordPath(path1, true);
        hybrid.recordPath(path2, true);
        hybrid.computeHybridPath();
        // Output path should be greater than both inputs.
        const base::PathPtr final_path = hybrid.getHybridPath();
        base::Cost final_cost = final_path->cost(obj);
        base::Cost first_cost = path1->cost(obj);
        base::Cost second_cost = path2->cost(obj);
        BOOST_CHECK(obj->isCostBetterThan(final_cost, first_cost) ||
                    obj->isCostEquivalentTo(final_cost, first_cost));
        BOOST_CHECK(obj->isCostBetterThan(final_cost, second_cost) ||
                    obj->isCostEquivalentTo(final_cost, second_cost));
    }

    template<typename T>
    void run_perturber(int runs)
    {
        base::OptimizationObjectivePtr obj(new T(si_));
        geometric::PathSimplifier simplifier(si_, ompl::base::GoalPtr(), obj);

        for (int path_idx = 0; path_idx < 2; path_idx++)
        {
            double avg_costs = 0.0;
            base::Cost original_cost = paths_[path_idx]->cost(obj);
            geometric::PathGeometric *path;
            for (int i = 0; i < runs; i++)
            {
                path = new geometric::PathGeometric(*paths_[path_idx]);
                simplifier.perturbPath(*path, 2.0, 100, 100, 0.005);
                avg_costs += path->cost(obj).value();
            }
            avg_costs /= runs;
            printf("Average cost: %f, original cost: %f\n", avg_costs, original_cost.value());
            BOOST_CHECK(obj->isCostBetterThan(base::Cost(avg_costs), original_cost) ||
                        obj->isCostEquivalentTo(base::Cost(avg_costs), original_cost));
        }
    }

protected:
    bool verbose_;
    Circles2D circles_;
    base::SpaceInformationPtr si_;
    std::vector<geometric::PathGeometric *> paths_;
};

BOOST_FIXTURE_TEST_SUITE(MySimplifierTestFixture, SimplifyTest)

BOOST_AUTO_TEST_CASE(geometric_PathLengthSimplifier)
{
    if (VERBOSE)
        printf("\n\n\n**************************************************\n"
               "Testing path length simplifier\n");
    run_simplifier<base::PathLengthOptimizationObjective>(20);
    if (VERBOSE)
        printf("Done with path length simplifier\n");
}

BOOST_AUTO_TEST_CASE(geomtric_PathLengthHybridization)
{
    if (VERBOSE)
        printf("\n\n\n**************************************************\n"
               "Testing path length hybridization\n");
    run_hybridizer<base::PathLengthOptimizationObjective>();
    if (VERBOSE)
        printf("Done with path length hybridization\n");
}

BOOST_AUTO_TEST_CASE(geometric_PathClearanceShortcutting)
{
    if (VERBOSE)
        printf("\n\n\n**************************************************\n"
               "Testing path clearance shorcutting\n");
    run_simplifier<base::MaximizeMinClearanceObjective>(20);
    if (VERBOSE)
        printf("Done with path clearance shortcutting\n");
}

BOOST_AUTO_TEST_CASE(geometric_PathClearancePerturber)
{
    if (VERBOSE)
        printf("\n\n\n**************************************************\n"
               "Testing path clearance perturbation\n");
    run_perturber<base::MaximizeMinClearanceObjective>(20);
    if (VERBOSE)
        printf("Done with path clearance perturbation\n");
}

BOOST_AUTO_TEST_CASE(geomtric_PathClearanceHybridization)
{
    if (VERBOSE)
        printf("\n\n\n**************************************************\n"
               "Testing path clearance hybridization\n");
    run_hybridizer<base::MaximizeMinClearanceObjective>();
    if (VERBOSE)
        printf("Done with path clerance hybridization\n");
}

BOOST_AUTO_TEST_SUITE_END()