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

#include "ompl/geometric/PathGeometric.h"
#include "ompl/geometric/PathSimplifier.h"
#include "ompl/geometric/PathHybridization.h"

#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/objectives/MaximizeMinClearanceObjective.h"

#include "../../base/PlannerTest.h"

using namespace ompl;

static const double VERBOSE = true;


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
    void run_simplifier()
    {
        base::OptimizationObjectivePtr obj(new T(si_));
        // Just use the first of the paths.
        geometric::PathGeometric *path = paths_[0];
        // TODO: make general to costs.
        geometric::PathSimplifier simplifier(si_);
        simplifier.simplify(*path, 5.0); // simplify for 10 seconds.
        // Output the new path.
        path->printAsMatrix(std::cout);
    }

    template<typename T>
    void run_hybridizer()
    {
        base::OptimizationObjectivePtr obj(new T(si_));
        geometric::PathHybridization hybrid(si_, obj);
        base::PathPtr path1(paths_[1]);
        base::PathPtr path2(paths_[2]);
        hybrid.recordPath(path1, true);
        hybrid.recordPath(path2, true);
        hybrid.computeHybridPath();
        const base::PathPtr final_path = hybrid.getHybridPath();
        final_path->as<geometric::PathGeometric>()->printAsMatrix(std::cout);
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
        printf("\n\n\n**************************************************\nTesting path length simplifier\n");
    run_simplifier<base::PathLengthOptimizationObjective>();
    if (VERBOSE)
        printf("Done with path length simplifier\n");
}

BOOST_AUTO_TEST_CASE(geomtric_PathLengthHybridization)
{
    if (VERBOSE)
        printf("\n\n\n**************************************************\nTesting path length hybridization\n");
    run_simplifier<base::PathLengthOptimizationObjective>();
    if (VERBOSE)
        printf("Done with path clearance hybridization\n");
}

BOOST_AUTO_TEST_CASE(geomtric_PathClearanceHybridization)
{
    if (VERBOSE)
        printf("\n\n\n**************************************************\nTesting path clearance hybridization\n");
    run_simplifier<base::MaximizeMinClearanceObjective>();
    if (VERBOSE)
        printf("Done with path clearance hybridization\n");
}


BOOST_AUTO_TEST_SUITE_END()