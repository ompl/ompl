/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, University of Stuttgart
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
 *   * Neither the name of the University of Stuttgart nor the names
 *     of its contributors may be used to endorse or promote products
 *     derived from this software without specific prior written
 *     permission.
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

/* Author: Andreas Orthey */

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/multilevel/planners/qrrt/QRRT.h>
#include <iostream>
#include <boost/math/constants/constants.hpp>

using namespace ompl::base;

// Path Planning on fiber bundle SE2 \rightarrow R2

bool boxConstraint(const double values[])
{
    const double &x = values[0] - 0.5;
    const double &y = values[1] - 0.5;
    double pos_cnstr = sqrt(x * x + y * y);
    return (pos_cnstr > 0.2);
}
bool isStateValid_SE2(const State *state)
{
    const auto *SE2state = state->as<SE2StateSpace::StateType>();
    const auto *R2 = SE2state->as<RealVectorStateSpace::StateType>(0);
    const auto *SO2 = SE2state->as<SO2StateSpace::StateType>(1);
    return boxConstraint(R2->values) && (SO2->value < boost::math::constants::pi<double>() / 2.0);
}
bool isStateValid_R2(const State *state)
{
    const auto *R2 = state->as<RealVectorStateSpace::StateType>();
    return boxConstraint(R2->values);
}

int main()
{
    // Setup SE2
    auto SE2(std::make_shared<SE2StateSpace>());
    RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(1);
    SE2->setBounds(bounds);
    SpaceInformationPtr si_SE2(std::make_shared<SpaceInformation>(SE2));
    si_SE2->setStateValidityChecker(isStateValid_SE2);

    // Setup Quotient-Space R2
    auto R2(std::make_shared<RealVectorStateSpace>(2));
    R2->setBounds(0, 1);
    SpaceInformationPtr si_R2(std::make_shared<SpaceInformation>(R2));
    si_R2->setStateValidityChecker(isStateValid_R2);

    // Create vector of spaceinformationptr
    std::vector<SpaceInformationPtr> si_vec;
    si_vec.push_back(si_R2);
    si_vec.push_back(si_SE2);

    // Define Planning Problem
    using SE2State = ScopedState<SE2StateSpace>;
    SE2State start_SE2(SE2);
    SE2State goal_SE2(SE2);
    start_SE2->setXY(0, 0);
    start_SE2->setYaw(0);
    goal_SE2->setXY(1, 1);
    goal_SE2->setYaw(0);

    ProblemDefinitionPtr pdef = std::make_shared<ProblemDefinition>(si_SE2);
    pdef->setStartAndGoalStates(start_SE2, goal_SE2);

    // Setup Planner using vector of spaceinformationptr
    auto planner = std::make_shared<ompl::multilevel::QRRT>(si_vec);

    // Planner can be used as any other OMPL algorithm
    planner->setProblemDefinition(pdef);
    planner->setup();

    PlannerStatus solved = planner->Planner::solve(1.0);

    if (solved)
    {
        std::cout << std::string(80, '-') << std::endl;
        std::cout << "Bundle Space Path (SE2):" << std::endl;
        std::cout << std::string(80, '-') << std::endl;
        pdef->getSolutionPath()->print(std::cout);

        std::cout << std::string(80, '-') << std::endl;
        std::cout << "Base Space Path (R2)   :" << std::endl;
        std::cout << std::string(80, '-') << std::endl;
        const ProblemDefinitionPtr pdefR2 = planner->getProblemDefinition(0);
        pdefR2->getSolutionPath()->print(std::cout);
        std::cout << std::string(80, '-') << std::endl;
    }
    return 0;
}
