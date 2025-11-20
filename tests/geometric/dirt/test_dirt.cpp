/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2025
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

#define BOOST_TEST_MODULE "GeometricPlannersDIRT"
#include <boost/test/unit_test.hpp>

#include "ompl/geometric/planners/dirt/DIRT.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"

using namespace ompl;

BOOST_AUTO_TEST_CASE(SimpleDIRTTest)
{
    auto space = std::make_shared<base::RealVectorStateSpace>(2);
    base::RealVectorBounds bounds(2);
    bounds.setLow(-10);
    bounds.setHigh(10);
    space->setBounds(bounds);

    auto si = std::make_shared<base::SpaceInformation>(space);
    si->setStateValidityChecker([](const base::State *) { return true; });
    si->setup();

    auto start = space->allocState()->as<base::RealVectorStateSpace::StateType>();
    start->values[0] = -5;
    start->values[1] = -5;

    auto goal = space->allocState()->as<base::RealVectorStateSpace::StateType>();
    goal->values[0] = 5;
    goal->values[1] = 5;

    auto pdef = std::make_shared<base::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start, goal, 0.1);
    pdef->setOptimizationObjective(std::make_shared<base::PathLengthOptimizationObjective>(si));

    auto planner = std::make_shared<geometric::DIRT>(si);
    planner->setProblemDefinition(pdef);
    planner->setup();

    base::PlannerStatus solved = planner->solve(base::timedPlannerTerminationCondition(1.0));
    
    BOOST_CHECK(solved);
    if (solved)
    {
        auto path = pdef->getSolutionPath();
        BOOST_CHECK(path != nullptr);
    }

    space->freeState(start);
    space->freeState(goal);
}

BOOST_AUTO_TEST_CASE(DIRTParametersTest)
{
    auto space = std::make_shared<base::RealVectorStateSpace>(2);
    auto si = std::make_shared<base::SpaceInformation>(space);
    
    auto planner = std::make_shared<geometric::DIRT>(si);
    
    planner->setRange(10.0);
    BOOST_CHECK_EQUAL(planner->getRange(), 10.0);
    
    planner->setBlossomNumber(3);
    BOOST_CHECK_EQUAL(planner->getBlossomNumber(), 3);
    
    planner->setUsePruning(false);
    BOOST_CHECK_EQUAL(planner->getUsePruning(), false);
}
