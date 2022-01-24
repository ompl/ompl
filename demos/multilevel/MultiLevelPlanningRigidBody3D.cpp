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

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/multilevel/planners/qrrt/QRRT.h>
#include <iostream>
#include <boost/math/constants/constants.hpp>

using namespace ompl::base;

using SE3State = ScopedState<SE3StateSpace>;
using SO3State = ScopedState<SO3StateSpace>;
using R3State = ScopedState<RealVectorStateSpace>;
const double pi = boost::math::constants::pi<double>();

// Path Planning on fiber bundle SE3 \rightarrow R3

bool isInCollision(double *val)
{
    const double &x = val[0] - 0.5;
    const double &y = val[1] - 0.5;
    const double &z = val[2] - 0.5;
    double d = sqrt(x * x + y * y + z * z);
    return (d > 0.2);
}

bool isStateValid_SE3(const State *state)
{
    static auto SO3(std::make_shared<SO3StateSpace>());
    static SO3State SO3id(SO3);
    SO3id->setIdentity();

    const auto *SE3state = state->as<SE3StateSpace::StateType>();
    const auto *R3state = SE3state->as<RealVectorStateSpace::StateType>(0);
    const State *SO3state = SE3state->as<SO3StateSpace::StateType>(1);
    const State *SO3stateIdentity = SO3id->as<SO3StateSpace::StateType>();

    double d = SO3->distance(SO3state, SO3stateIdentity);
    return isInCollision(R3state->values) && (d < pi / 4.0);
}

bool isStateValid_R3(const State *state)
{
    const auto *R3 = state->as<RealVectorStateSpace::StateType>();
    return isInCollision(R3->values);
}

int main()
{
    //############################################################################
    // Step 1: Setup planning problem using several quotient-spaces
    //############################################################################
    // Setup SE3
    auto SE3(std::make_shared<SE3StateSpace>());
    RealVectorBounds bounds(3);
    bounds.setLow(0);
    bounds.setHigh(1);
    SE3->setBounds(bounds);
    SpaceInformationPtr si_SE3(std::make_shared<SpaceInformation>(SE3));
    si_SE3->setStateValidityChecker(isStateValid_SE3);

    // Setup Quotient-Space R2
    auto R3(std::make_shared<RealVectorStateSpace>(3));
    R3->setBounds(0, 1);
    SpaceInformationPtr si_R3(std::make_shared<SpaceInformation>(R3));
    si_R3->setStateValidityChecker(isStateValid_R3);

    // Create vector of spaceinformationptr (last one is original cspace)
    std::vector<SpaceInformationPtr> si_vec;
    si_vec.push_back(si_R3);
    si_vec.push_back(si_SE3);

    // Define Planning Problem
    SE3State start_SE3(SE3);
    SE3State goal_SE3(SE3);
    start_SE3->setXYZ(0, 0, 0);
    start_SE3->rotation().setIdentity();
    goal_SE3->setXYZ(1, 1, 1);
    goal_SE3->rotation().setIdentity();

    ProblemDefinitionPtr pdef = std::make_shared<ProblemDefinition>(si_SE3);
    pdef->setStartAndGoalStates(start_SE3, goal_SE3);

    //############################################################################
    // Step 2: Do path planning as usual but with a sequence of
    // spaceinformationptr
    //############################################################################
    auto planner = std::make_shared<ompl::multilevel::QRRT>(si_vec);

    // Planner can be used as any other OMPL algorithm
    planner->setProblemDefinition(pdef);
    planner->setup();

    PlannerStatus solved = planner->Planner::solve(1.0);

    if (solved)
    {
        std::cout << std::string(80, '-') << std::endl;
        std::cout << "Bundle-Space Path (SE3):" << std::endl;
        std::cout << std::string(80, '-') << std::endl;
        pdef->getSolutionPath()->print(std::cout);

        std::cout << std::string(80, '-') << std::endl;
        std::cout << "Base-Space Path (R3)   :" << std::endl;
        std::cout << std::string(80, '-') << std::endl;
        const ProblemDefinitionPtr pdefR3 = planner->getProblemDefinition(0);
        pdefR3->getSolutionPath()->print(std::cout);
    }
    return 0;
}
