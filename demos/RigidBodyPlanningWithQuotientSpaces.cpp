#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/quotientspace/multiquotient.h>
#include <ompl/geometric/planners/quotientspace/quotient.h>
#include <ompl/geometric/planners/quotientspace/QRRT.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid_SE2(const ob::State *state)
{
  return true;
}
bool isStateValid_R2(const ob::State *state)
{
  return true;
}

int main(int argc,const char** argv)
{
  //Setup SE2
  ob::StateSpacePtr SE2(std::make_shared<ob::SE2StateSpace>());
  ob::RealVectorBounds bounds(2);
  bounds.setLow(0);
  bounds.setHigh(1);
  std::static_pointer_cast<ob::SE3StateSpace>(SE2)->setBounds(bounds);
  ob::SpaceInformationPtr si_SE2(std::make_shared<ob::SpaceInformation>(SE2));
  si_SE2->setStateValidityChecker(isStateValid_SE2);

  //Setup Quotient-Space R2
  ob::StateSpacePtr R2(std::make_shared<ob::RealVectorStateSpace>(2));
  std::static_pointer_cast<ob::RealVectorStateSpace>(R2)->setBounds(0,1);
  ob::SpaceInformationPtr si_R2(std::make_shared<ob::SpaceInformation>(R2));
  si_R2->setStateValidityChecker(isStateValid_R2);

  //Create vector of spaceinformationptr
  std::vector<ob::SpaceInformationPtr> si_vec;
  si_vec.push_back(si_R2);
  si_vec.push_back(si_SE2);

  //Create vector of ProblemDefinitionPtr
  std::vector<ob::ProblemDefinitionPtr> pdef_vec; 

  typedef ob::ScopedState<ob::SE2StateSpace> SE2State;
  typedef ob::ScopedState<ob::RealVectorStateSpace> R2State;

  //Define Planning Problem
  SE2State start_SE2(SE2);
  SE2State goal_SE2(SE2);
  start_SE2->setXY(0,0); start_SE2->setYaw(0);
  goal_SE2->setXY(1,1); goal_SE2->setYaw(0);

  R2State start_R2(R2);
  R2State goal_R2(R2);
  start_R2[0]=start_R2[1]=0;
  goal_R2[0]=goal_R2[1]=1;

  ob::ProblemDefinitionPtr pdef_SE2 = std::make_shared<ob::ProblemDefinition>(si_SE2);
  pdef_SE2->setStartAndGoalStates(start_SE2, goal_SE2);
  ob::ProblemDefinitionPtr pdef_R2 = std::make_shared<ob::ProblemDefinition>(si_R2);
  pdef_R2->setStartAndGoalStates(start_R2, goal_R2);

  pdef_vec.push_back(pdef_R2);
  pdef_vec.push_back(pdef_SE2);

  //Setup Planner using vector of spaceinformationptr
  typedef og::MultiQuotient<og::QRRT> MultiQuotient;
  ob::PlannerPtr planner = std::make_shared<MultiQuotient>(si_vec);
  std::static_pointer_cast<MultiQuotient>(planner)->setProblemDefinition(pdef_vec);

  //Planner can be used as any other OMPL algorithm
  planner->setup();

  ob::PlannerStatus solved = planner->ob::Planner::solve(1.0);

  if (solved)
  {
    ob::PathPtr path = pdef_vec.back()->getSolutionPath();
    std::cout << "Found solution:" << std::endl;
    path->print(std::cout);
  }
  return 0;
}
