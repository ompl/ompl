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
*  * Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*  * Redistributions in binary form must reproduce the above
*    copyright notice, this list of conditions and the following
*    disclaimer in the documentation and/or other materials provided
*    with the distribution.
*  * Neither the name of the University of Stuttgart nor the names
*    of its contributors may be used to endorse or promote products
*    derived from this software without specific prior written
*    permission.
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
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/quotientspace/MultiQuotient.h>
#include <ompl/geometric/planners/quotientspace/QRRT.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

//Path Planning in SE2 = R2 \times SO2
//using quotient-spaces R2 and SE2
bool isStateValid_SE2(const ob::State *state) 
{
    // disallow going through the middle of the square [0,1]x[0,1]
    // to make it more interesting (i.e. an R2 path might be invalid)
    const auto *SE2state = state->as<ob::SE2StateSpace::StateType>();
    const auto *R2 = SE2state->as<ob::RealVectorStateSpace::StateType>(0);
    const auto *SO2 = SE2state->as<ob::SO2StateSpace::StateType>(1);
    double *pos = R2->values;
    double pos_cnstr =
            sqrt((pos[0] - 0.5) * (pos[0] - 0.5) + (pos[1] - 0.5) * (pos[1] - 0.5));
    return (pos_cnstr > 0.2) && (SO2->value < M_PI / 2.0);
}

bool isStateValid_R2(const ob::State *state) 
{ 
    return true; 
}

int main(int argc, const char **argv) 
{
    // Setup SE2
    ob::StateSpacePtr SE2(std::make_shared<ob::SE2StateSpace>());
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0);
    bounds.setHigh(1);
    std::static_pointer_cast<ob::SE3StateSpace>(SE2)->setBounds(bounds);
    ob::SpaceInformationPtr si_SE2(std::make_shared<ob::SpaceInformation>(SE2));
    si_SE2->setStateValidityChecker(isStateValid_SE2);

    // Setup Quotient-Space R2
    ob::StateSpacePtr R2(std::make_shared<ob::RealVectorStateSpace>(2));
    std::static_pointer_cast<ob::RealVectorStateSpace>(R2)->setBounds(0, 1);
    ob::SpaceInformationPtr si_R2(std::make_shared<ob::SpaceInformation>(R2));
    si_R2->setStateValidityChecker(isStateValid_R2);

    // Create vector of spaceinformationptr
    std::vector<ob::SpaceInformationPtr> si_vec;
    si_vec.push_back(si_R2);
    si_vec.push_back(si_SE2);

    // Define Planning Problem
    typedef ob::ScopedState<ob::SE2StateSpace> SE2State;
    typedef ob::ScopedState<ob::RealVectorStateSpace> R2State;
    SE2State start_SE2(SE2);
    SE2State goal_SE2(SE2);
    start_SE2->setXY(0, 0);
    start_SE2->setYaw(0);
    goal_SE2->setXY(1, 1);
    goal_SE2->setYaw(0);

    R2State start_R2(R2);
    R2State goal_R2(R2);
    start_R2[0] = start_R2[1] = 0;
    goal_R2[0] = goal_R2[1] = 1;

    // Create vector of ProblemDefinitionPtr
    std::vector<ob::ProblemDefinitionPtr> pdef_vec;
    ob::ProblemDefinitionPtr pdef_SE2 =
            std::make_shared<ob::ProblemDefinition>(si_SE2);
    pdef_SE2->setStartAndGoalStates(start_SE2, goal_SE2);
    ob::ProblemDefinitionPtr pdef_R2 =
            std::make_shared<ob::ProblemDefinition>(si_R2);
    pdef_R2->setStartAndGoalStates(start_R2, goal_R2);

    pdef_vec.push_back(pdef_R2);
    pdef_vec.push_back(pdef_SE2);

    // Setup Planner using vector of spaceinformationptr
    typedef og::MultiQuotient<og::QRRT> MultiQuotient;
    ob::PlannerPtr planner = std::make_shared<MultiQuotient>(si_vec);
    std::static_pointer_cast<MultiQuotient>(planner)
            ->setProblemDefinition(pdef_vec);

    // Planner can be used as any other OMPL algorithm
    planner->setup();

    ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

    if (solved) 
    {
        std::cout << std::string(80, '-') << std::endl;
        std::cout << "Configuration-Space Path (SE2):" << std::endl;
        std::cout << std::string(80, '-') << std::endl;
        pdef_vec.back()->getSolutionPath()->print(std::cout);
        std::cout << std::string(80, '-') << std::endl;
        std::cout << "Quotient-Space Path (R2):" << std::endl;
        std::cout << std::string(80, '-') << std::endl;
        pdef_vec.front()->getSolutionPath()->print(std::cout);

        std::vector<int> nodes =
          std::static_pointer_cast<MultiQuotient>(planner)->getFeasibleNodes();

        std::cout << std::string(80, '-') << std::endl;
        for(uint k = 0; k < nodes.size(); k++)
        {
            std::cout << "QuotientSpace" << k << " has " << nodes.at(k) << " nodes." << std::endl;
        }
    }
    return 0;
}
