#include <ompl/geometric/planners/explorer/PathVisibilityChecker.h>
#include <ompl/base/State.h>
#include <ompl/base/ScopedState.h>

#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/pdst/PDST.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/StateSpaceTypes.h>

#include <boost/math/constants/constants.hpp>

using namespace ompl::geometric;

PathVisibilityChecker::PathVisibilityChecker(const base::SpaceInformationPtr &si):
  si_(si)
{
  // ob::StateSpacePtr space = si_->getStateSpace();
  // if(space->getType() == ob::STATE_SPACE_REAL_VECTOR){
  //   Test1();
  // }
  // if(space->getType() == ob::STATE_SPACE_SE2){
  //   Test2();
  //   exit(0);
  // }
  // Test3();
  // exit(0);
  lastValidState = si_->allocState();

  space = std::make_shared<ob::RealVectorStateSpace>(2);
  R2 = space->as<ob::RealVectorStateSpace>();

  ob::RealVectorBounds bounds(2);
  bounds.setLow(0);
  bounds.setHigh(1);
  R2->setBounds(bounds);

}

PathVisibilityChecker::~PathVisibilityChecker(void)
{
}

class pathPathValidityChecker : public ob::StateValidityChecker
{
public:
  pathPathValidityChecker(const ob::SpaceInformationPtr &si, const ob::SpaceInformationPtr &si_local, std::vector<ob::State*> s1, std::vector<ob::State*> s2) : ob::StateValidityChecker(si), si_local_(si_local)
  {
    path1_ = s1;
    path2_ = s2;
    path1_length_ = 0;
    path2_length_ = 0;

    path1_interp_state_ = si_->allocState();
    path2_interp_state_ = si_->allocState();

    path1_distances_.clear();
    path2_distances_.clear();

    computePathLength(path1_, path1_distances_, path1_length_);
    computePathLength(path2_, path2_distances_, path2_length_);

    ////############################################################################
    ////TODO: DEBUG
    ////############################################################################

    ////Compute Start and End points are the same
    //createStateAt(path1_, path1_length_, path1_distances_, 0, path1_interp_state_);
    //createStateAt(path2_, path2_length_, path2_distances_, 0, path2_interp_state_);

    //assert( si_->distance(path1_interp_state_, path2_interp_state_) < 1e-10);

    //createStateAt(path1_, path1_length_, path1_distances_, path1_length_, path1_interp_state_);
    //createStateAt(path2_, path2_length_, path2_distances_, path2_length_, path2_interp_state_);

    //assert( si_->distance(path1_interp_state_, path2_interp_state_) < 1e-10);

  }
  virtual ~pathPathValidityChecker(){
      si_->freeState(path1_interp_state_);
      si_->freeState(path2_interp_state_);
  }

  virtual bool isValid(const ob::State *state) const
  {
    const ob::RealVectorStateSpace::StateType *RnSpace = state->as<ob::RealVectorStateSpace::StateType>();

    double t1 = RnSpace->values[0] * path1_length_;
    createStateAt(path1_, path1_length_, path1_distances_, t1, path1_interp_state_);

    double t2 = RnSpace->values[1] * path2_length_;
    createStateAt(path2_, path2_length_, path2_distances_, t2, path2_interp_state_);

    //############################################################################
    //DEBUG
    //############################################################################
    // CheckValidityExitOnFalse(path1_interp_state_);
    // CheckValidityExitOnFalse(path2_interp_state_);
    //This means we have an edge which intersects an obstacle for less than
    //discretization step distance. We like to ignore that for now.
    if(!si_->getStateValidityChecker()->isValid(path1_interp_state_)) return true;
    if(!si_->getStateValidityChecker()->isValid(path2_interp_state_)) return true;

    bool visible = si_->checkMotion(path1_interp_state_, path2_interp_state_);
    //############################################################################
    //DEBUG
    //############################################################################
    // if(!visible){
    //   std::cout << "States not visible" << std::endl;
    //   si_->printState(path1_interp_state_);
    //   si_->printState(path2_interp_state_);

    //   // ob::State *s_interpolate = si_->allocState();
    //   // double dstep = 0.01;
    //   // std::cout << std::string(80, '-') << std::endl;
    //   // for(double d = dstep; d < 1; d+=dstep){
    //   //     si_->getStateSpace()->interpolate(path1_interp_state_, path2_interp_state_, d, s_interpolate);
    //   //     // bool visible = si_->checkMotion(path1_interp_state_, s_interpolate);
    //   //     bool val = si_->getStateValidityChecker()->isValid(s_interpolate);
    //   //     if(!val){
    //   //       si_->printState(s_interpolate);
    //   //       std::cout << "INVALID" << std::endl;
    //   //     }
    //   // }
    //   // exit(0);
    // }
    //############################################################################

    return visible;
  }
  void CheckValidityExitOnFalse(ob::State* s) const
  {
      bool val = si_->getStateValidityChecker()->isValid(s);
      if(!val){
          ompl::msg::setLogLevel(ompl::msg::LOG_DEV2);
          OMPL_WARN("State is invalid!");
          //remove path entirely?

          // si_->printState(s);
          // std::cout << std::string(80, '-') << std::endl;
          // for(uint k = 0; k < path1_.size(); k++){
          //     si_->printState(path1_.at(k));
          // }
          // for(uint k = 0; k < path2_.size(); k++){
          //     si_->printState(path2_.at(k));
          // }
          // // std::cout << "Path1: " << (this->CheckValidity(path1_)?"VALIDPATH":"INVALID") << std::endl;
          // // std::cout << "Path2: " << (this->CheckValidity(path2_)?"VALIDPATH":"INVALID") << std::endl;
          // exit(0);
      }
  }

  //computes the distance between each two adjacent states in the path and the overall path-length
  //saves the accumulated (!) distance (distance from start to the point at this index) for each point in the stateDistances vector
  //computes the overall pathLength which is equal to the last entry of stateDistances
  void computePathLength(const std::vector<ob::State*> &path, std::vector<double> &stateDistances, double &pathLength) 
  {
    pathLength = 0;
    if (path.size() > 1) {
      for (uint i = 0; i < path.size() - 1; i++) {
        double distStateState = si_->distance(path.at(i), path.at(i+1));
        pathLength += distStateState;
        stateDistances.push_back(pathLength);
      }
    } else {
      stateDistances.push_back(0);
    }
  }

  //creates a new state at exactly the given position within the given path
  void createStateAt(const std::vector<ob::State*> &path, const double &pathLength, const std::vector<double> &distances, const double newPosition, ob::State* s_interpolate) const 
  {

    assert( newPosition <= pathLength);

    int idx = -1;
    for(uint i = 0; i < distances.size(); i++) {
      if (distances.at(i) >= newPosition) {
        idx = i;
        break;
      }
    }
    assert( idx >= 0 );
    assert( idx <= distances.size()-1 );

    double lastDistance = (idx > 0)? distances.at(idx-1) : 0.0;
    double distanceIdxIdxNext = distances.at(idx) - lastDistance;

    double lineFraction = (newPosition - lastDistance)/distanceIdxIdxNext;
    if(lineFraction < 0 || lineFraction > 1)
    {
      OMPL_ERROR("lineFraction: %f. length: %f, newPos: %f, distanceNext: %f, distanceCur: %f",
          lineFraction, pathLength, newPosition, distanceIdxIdxNext, distances.at(idx));
      exit(0);
    }

    si_->getStateSpace()->interpolate(path.at(idx), path.at(idx+1), lineFraction, s_interpolate);

  }


  private:

  ob::SpaceInformationPtr si_local_;
  std::vector<ob::State*> path1_;
  std::vector<ob::State*> path2_;
  std::vector<double> path1_distances_;
  std::vector<double> path2_distances_;
  double path1_length_;
  double path2_length_;
  ob::State* path1_interp_state_;
  ob::State* path2_interp_state_;
};

bool PathVisibilityChecker::IsPathVisible(std::vector<QuotientSpaceGraph::Vertex> &v1, std::vector<QuotientSpaceGraph::Vertex> &v2, QuotientSpaceGraph::Graph &graph)
{
  std::vector<ob::State*> s1;
  std::vector<ob::State*> s2;
  for(uint k = 0; k < v1.size(); k++){
    s1.push_back(graph[v1.at(k)]->state);
  }
  for(uint k = 0; k < v2.size(); k++){
    s2.push_back(graph[v2.at(k)]->state);
  }
  return IsPathVisible(s1, s2);
}

bool PathVisibilityChecker::CheckValidity(const std::vector<ob::State*> &s)
{
  for(uint k = 0; k < s.size()-1; k++){
    ob::State *sk = s.at(k);
    ob::State *skk = s.at(k+1);
    if(!si_->isValid(sk)){
      OMPL_ERROR("State invalid");
      si_->printState(sk);
      exit(0);
    }
    // std::pair<ob::State *, double> lastValid;
    // lastValid.first = lastValidState;
    // bool val = si_->checkMotion(sk, skk, lastValid);
    bool val = si_->checkMotion(sk, skk);
    if(!val) return false;
  }
  return true;
}


/// \brief Test if a path goes clockwise (increasing angle) or counterclockwise
//(decreasing angle). We encounter a problem whenever the path goes above +PI,
//which would change the next state to something lower. 

bool PathVisibilityChecker::isPathClockwise(std::vector<ob::State*> &spath)
{
  ob::StateSpacePtr space = si_->getStateSpace();
  assert(space->getType() == ob::STATE_SPACE_SO2);

  const double a1 = spath.at(0)->as<ob::SO2StateSpace::StateType>()->value;
  const double a2 = spath.at(1)->as<ob::SO2StateSpace::StateType>()->value;
  const double diff = a2 - a1;
  const double pi = boost::math::constants::pi<double>();

  return (fabs(diff) <= pi)? a2>=a1 : a2<a1;
}

/// \brief On SO(2), the circle space, two paths are visible if they both go
//clockwise or counterclockwise. Otherwise they are not visible.

bool PathVisibilityChecker::IsPathVisibleSO2(std::vector<ob::State*> &s1, std::vector<ob::State*> &s2)
{
    bool s1cw = isPathClockwise(s1);
    bool s2cw = isPathClockwise(s2);

    return (s1cw == s2cw);
}

bool PathVisibilityChecker::IsPathVisible(std::vector<ob::State*> &s1, std::vector<ob::State*> &s2)
{
  //Disable logging for Checker
  ompl::msg::LogLevel logLevelInitial = ompl::msg::getLogLevel();
  ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
  // ompl::msg::setLogLevel(ompl::msg::LOG_DEV2);

  //Assert Non-empty paths with at least a designated end and start 
  //configuration
  assert(s1.size()>=2);
  assert(s2.size()>=2);

  //Assert Same start and end point
  assert( si_->distance(s1.front(), s2.front()) < 1e-10);
  assert( si_->distance(s1.back(), s2.back()) < 1e-10);

  //Paths need to be feasible
  if(!CheckValidity(s1)) return false;
  if(!CheckValidity(s2)) return false;

  //Handle edge case of SO(2)
  if(si_->getStateSpace()->getType() == ob::STATE_SPACE_SO2)
  {
      return IsPathVisibleSO2(s1, s2);
  }else{

  }

  const float max__planning_time_path_path = 0.3;
  const float epsilon_goalregion = 0.01;

  ob::ScopedState<> start(space);
  ob::ScopedState<> goal(space);
  start[0]=start[1]=0.0;
  goal[0]=goal[1]=1.0;

  og::SimpleSetup ss(space);
  const ob::SpaceInformationPtr si_local = ss.getSpaceInformation();

  ss.setStateValidityChecker( std::make_shared<pathPathValidityChecker>(si_, si_local,  s1, s2) );
  ob::PlannerPtr linear_homotopy_planner = std::make_shared<og::RRTConnect>(si_local);

  ss.setStartAndGoalStates(start, goal, epsilon_goalregion);
  ss.setPlanner(linear_homotopy_planner);
  ss.setup();

  // ob::ProblemDefinitionPtr pdef = ss.getProblemDefinition();
  ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(max__planning_time_path_path) );

  ss.solve(ptc);
  // std::cout << ss.getLastPlanComputationTime() << std::endl;
  bool solved = ss.haveExactSolutionPath();

  //############################################################################
  //DEBUG
  //############################################################################
  // ob::PlannerDataPtr pd( new ob::PlannerData(si_local) );
  // linear_homotopy_planner->getPlannerData(*pd);
  // pd->decoupleFromPlanner();
  // pd->printGraphviz(std::cout);
  // std::cout << pd->numVertices() << std::endl;
  // std::cout << pd->numStartVertices() << std::endl;
  // std::cout << pd->numGoalVertices() << std::endl;
  // if(!solved)
  // {
  //   std::cout << "Failed Deforming" << std::endl;
  // }else{
  // // for(uint k = 0; k < pd->numVertices(); k++){
  // //   si_local->printState(pd->getVertex(k).getState());
  // // }
  // }
  //############################################################################


  ompl::msg::setLogLevel(logLevelInitial);
  return solved;

}

////############################################################################
////TESTING
////############################################################################
//std::vector<ob::State*> PathVisibilityChecker::StatesFromVector(
//    const std::vector<double> &sx, 
//    const std::vector<double> &sy)
//{
//  ob::StateSpacePtr space = si_->getStateSpace();
//  assert(space->getType() == ob::STATE_SPACE_REAL_VECTOR);

//  std::vector<ob::State*> spath;
//  assert(sx.size() == sy.size());
//  for(uint k = 0; k < sx.size(); k++){
//    const double x = sx.at(k);
//    const double y = sy.at(k);
//    ob::State *state = space->allocState();
//    ob::RealVectorStateSpace::StateType *pos = state->as<ob::RealVectorStateSpace::StateType>();
//    pos->values[0] = x; pos->values[1] = y;
//    spath.push_back(state);
//  }
//  return spath;
//}
//std::vector<ob::State*> PathVisibilityChecker::StatesFromVector(
//    const std::vector<double> &sx, 
//    const std::vector<double> &sy, 
//    const std::vector<double> &st)
//{
//  ob::StateSpacePtr space = si_->getStateSpace();
//  assert(space->getType() == ob::STATE_SPACE_SE2);

//  std::vector<ob::State*> spath;
//  assert(sx.size() == sy.size());
//  for(uint k = 0; k < sx.size(); k++){
//    const double x = sx.at(k);
//    const double y = sy.at(k);
//    const double t = st.at(k);
//    ob::State *state = space->allocState();
//    ob::SE2StateSpace::StateType *pos = state->as<ob::SE2StateSpace::StateType>();
//    pos->setX(x);
//    pos->setY(y);
//    pos->setYaw(t);
//    spath.push_back(state);
//  }
//  return spath;
//}
//std::vector<ob::State*> PathVisibilityChecker::StatesFromVectorSO2R1(
//    const std::vector<double> &st, 
//    const std::vector<double> &sx)
//{
//  ob::StateSpacePtr space = si_->getStateSpace();

//  std::vector<ob::State*> spath;
//  assert(sx.size() == st.size());
//  for(uint k = 0; k < st.size(); k++){
//    const double t = st.at(k);
//    const double x = sx.at(k);
//    ob::State *state = space->allocState();
//    ob::SO2StateSpace::StateType *posT = state->as<ob::CompoundState>()->as<ob::SO2StateSpace::StateType>(0);
//    ob::RealVectorStateSpace::StateType *posX = state->as<ob::CompoundState>()->as<ob::RealVectorStateSpace::StateType>(1);

//    posT->value = t;
//    posX->values[0] = x;

//    spath.push_back(state);
//  }
//  return spath;
//}

//void PathVisibilityChecker::Test3(int F)
//{
//  std::vector<double> s1t{0.000000,-1.071710,-1.097130,-1.116430,-1.138230,-1.165030,-1.197700,-3.140000};
//  std::vector<double> s1x{0.000000,-1.078120,-1.082550,-1.076350,-1.067040,-1.051500,-1.028730,0.800000};
//  std::vector<double> s2t{0.000000,-1.014960,-1.085780,-1.145660,-3.140000};
//  std::vector<double> s2x{0.000000,-1.039350,-1.090600,-1.079550,0.800000};

//  std::vector<double> s3t{0.000000,-1.078860,-3.140000};
//  std::vector<double> s3x{0.000000,-1.187160,0.800000};
//  std::vector<double> s4t{0.000000,-1.072940,-1.103600,-1.141770,-1.211700,-3.140000};
//  std::vector<double> s4x{0.000000,-1.082170,-1.085270,-1.066930,-1.017820,0.800000};

//  std::vector<ob::State*> s1 = StatesFromVectorSO2R1(s1t, s1x);
//  std::vector<ob::State*> s2 = StatesFromVectorSO2R1(s2t, s2x);
//  std::vector<ob::State*> s3 = StatesFromVectorSO2R1(s3t, s3x);
//  std::vector<ob::State*> s4 = StatesFromVectorSO2R1(s4t, s4x);

//  // std::cout << "Test3: " << (IsPathVisible(s1, s2)?"OK":"Failed") << std::endl;
//  std::cout << "Test3(" << F << "):" << (IsPathVisible(s3, s4)?"OK":"Failed") << std::endl;
//}

//void PathVisibilityChecker::Test1()
//{

//  std::vector<double> s1x{-2.000000,-0.512838,0.442596,0.474337,0.509194,2.000000};
//  std::vector<double> s1y{0.000000,-0.436827,-0.452332,-0.445104,-0.435831,-0.000000};
//  std::vector<double> s2x{-2.000000,-0.768499,-0.491926,0.432292,1.404630,2.000000};
//  std::vector<double> s2y{0.000000,-0.380547,-0.437810,-0.458197,-0.213621,-0.000000};

//  std::vector<double> s3x{-2.000000,-0.573877,0.363747,2.000000};
//  std::vector<double> s3y{0.000000,0.434022,0.507960,-0.000000};
//  std::vector<double> s4x{-2.000000,-0.502340,0.492229,0.517778,2.000000};
//  std::vector<double> s4y{0.000000,0.437578,0.437748,0.433951,-0.000000};


//  std::vector<ob::State*> s1 = StatesFromVector(s1x, s1y);
//  std::vector<ob::State*> s2 = StatesFromVector(s2x, s2y);
//  std::vector<ob::State*> s3 = StatesFromVector(s3x, s3y);
//  std::vector<ob::State*> s4 = StatesFromVector(s4x, s4y);

//  std::cout << "Test1a: " << (IsPathVisible(s1, s2)?"OK":"Failed") << std::endl;
//  std::cout << "Test1b: " << (IsPathVisible(s3, s4)?"OK":"Failed") << std::endl;
//  std::cout << "Test1c: " << (IsPathVisible(s4, s3)?"OK":"Failed") << std::endl;

//}
//void PathVisibilityChecker::Test2()
//{
//  std::vector<double> s1x{-2.00000,-0.72904,-0.53428,-0.34795,0.47254,0.73711,2.00000};
//  std::vector<double> s1y{0.00000,-0.42063,-0.48938,-0.51364,-0.50763,-0.45193,-0.00000};
//  std::vector<double> s1t{0.00000,-0.34727,-0.25106,-0.12199,0.09351,0.16478,0.00000};
//  std::vector<double> s2x{-2.00000,-0.62846,-0.44692,-0.44443,-0.44194,-0.35541,0.37325,0.67675,0.73934,2.00000};
//  std::vector<double> s2y{0.00000,-0.44790,-0.49638,-0.49698,-0.49758,-0.50851,-0.51348,-0.47290,-0.45123,-0.00000};
//  std::vector<double> s2t{0.00000,-0.37757,-0.17117,-0.16775,-0.16433,-0.10402,-0.02477,0.10157,0.16481,0.00000};
//  std::vector<double> s3x{-2.00000,-0.75950,-0.69805,-0.68531,-0.67900,-0.67268,-0.66637,-0.57544,-0.41726,-0.36587,0.23391,0.34742,0.46283,0.56933,1.53505,2.00000};
//  std::vector<double> s3y{0.00000,-0.41519,-0.43626,-0.44063,-0.44277,-0.44492,-0.44706,-0.47636,-0.50654,-0.50951,-0.50995,-0.50786,-0.50367,-0.48640,-0.16180,-0.00000};
//  std::vector<double> s3t{0.00000,-0.29672,-0.28403,-0.27863,-0.27590,-0.27318,-0.27045,-0.22802,-0.14212,-0.11944,0.07773,0.11854,0.17925,0.27803,0.15030,0.00000};

//  std::vector<double> s4x{-2.00000,-1.35536,-0.71073,-0.25617,0.19840,0.74338,2.00000};
//  std::vector<double> s4y{0.00000,0.24306,0.48612,0.52731,0.56851,0.43237,-0.00000};
//  std::vector<double> s4t{0.00000,0.03532,0.07065,-0.08761,-0.24587,-0.24999,0.00000};
//  std::vector<double> s5x{-2.00000,-0.76598,-0.68495,-0.45379,0.28916,0.29405,0.29894,0.36504,0.72607,2.00000};
//  std::vector<double> s5y{0.00000,0.44129,0.46610,0.49207,0.54172,0.54126,0.54081,0.53051,0.43268,-0.00000};
//  std::vector<double> s5t{0.00000,0.17336,0.15659,0.04551,-0.24555,-0.24643,-0.24731,-0.25543,-0.26018,0.00000};

//  std::vector<double> s6x{-2.00000,-0.48142,-0.45790,-0.37944,-0.30362,0.35280,0.41819,0.45289,2.00000};
//  std::vector<double> s6y{0.00000,-0.50848,-0.51413,-0.52562,-0.52811,-0.51142,-0.50204,-0.49574,-0.00000};
//  std::vector<double> s6t{0.00000,2.75165,2.79052,2.89891,2.95514,-3.01023,-2.99079,-2.96867,0.00000};

//  std::vector<double> s7x{-2.00000,-0.46890,-0.41915,-0.35847,-0.27946,0.42024,0.43681,2.00000};
//  std::vector<double> s7y{0.00000,-0.51185,-0.52068,-0.52578,-0.52914,-0.50332,-0.50004,-0.00000};
//  std::vector<double> s7t{0.00000,2.77296,2.84880,2.91146,2.97442,-3.00769,2.99234,0.00000};


//  std::vector<ob::State*> s1 = StatesFromVector(s1x, s1y, s1t);
//  std::vector<ob::State*> s2 = StatesFromVector(s2x, s2y, s2t);
//  std::vector<ob::State*> s3 = StatesFromVector(s3x, s3y, s3t);
//  std::vector<ob::State*> s4 = StatesFromVector(s4x, s4y, s4t);
//  std::vector<ob::State*> s5 = StatesFromVector(s5x, s5y, s5t);
//  std::vector<ob::State*> s6 = StatesFromVector(s6x, s6y, s6t);
//  std::vector<ob::State*> s7 = StatesFromVector(s7x, s7y, s7t);

//  // std::cout << "Test2a: " << (IsPathVisible(s1, s2)?"OK":"Failed") << std::endl;
//  // std::cout << "Test2b: " << (IsPathVisible(s2, s3)?"OK":"Failed") << std::endl;
//  // std::cout << "Test2c: " << (IsPathVisible(s1, s3)?"OK":"Failed") << std::endl;
//  // std::cout << "Test2d: " << (IsPathVisible(s4, s5)?"OK":"Failed") << std::endl;
//  // std::cout << "Test2e: " << (IsPathVisible(s5, s4)?"OK":"Failed") << std::endl;
//  std::cout << "Test2f: " << (IsPathVisible(s6, s7)?"OK":"Failed") << std::endl;
//  // std::cout << "Test2g: " << (IsPathVisible(s7, s6)?"OK":"Failed") << std::endl;
//  // for(uint k = 0; k < 10; k++){
//  //   bool visible = IsPathVisible(s7, s6) && IsPathVisible(s6, s7);
//  //   std::cout << std::string(80, '-') << std::endl;
//  //   std::cout << "Test" << std::to_string(k) << ":" << (visible?"OK":"Failed") << std::endl;
//  //   if(!visible){
//  //     std::cout << "FAILED" << std::endl;
//  //     exit(0);
//  //   }
//  // }
//}

