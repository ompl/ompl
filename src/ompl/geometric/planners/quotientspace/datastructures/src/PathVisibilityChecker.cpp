#include <ompl/geometric/planners/quotientspace/datastructures/PathVisibilityChecker.h>
#include <ompl/base/State.h>
#include <ompl/base/ScopedState.h>
#include <ompl/util/Exception.h>

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
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/StateSpaceTypes.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/DynamicalMotionValidator.h>

#include <ompl/control/Control.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/DirectedControlSampler.h>
#include <ompl/control/SimpleDirectedControlSampler.h>


#include <boost/math/constants/constants.hpp>

using namespace ompl::geometric;

PathVisibilityChecker::PathVisibilityChecker(const base::SpaceInformationPtr &si):
  si_(si)
{
  lastValidState = si_->allocState();

  R2space_ = std::make_shared<ob::RealVectorStateSpace>(2);
  R2 = R2space_->as<ob::RealVectorStateSpace>();

  ob::RealVectorBounds bounds(2);
  bounds.setLow(0);
  bounds.setHigh(1);
  R2->setBounds(bounds);

  start = std::make_shared<ob::ScopedState<>>(R2space_);
  goal = std::make_shared<ob::ScopedState<>>(R2space_);

  (*start)[0] = 0.;
  (*start)[1] = 0.;
  (*goal)[0] = 1.;
  (*goal)[1] = 1.;

  ss = std::make_shared<og::SimpleSetup>(R2space_);
  si_local = ss->getSpaceInformation();

  ompl::control::SpaceInformationPtr siC = std::dynamic_pointer_cast<ompl::control::SpaceInformation>(si);
  if(siC==nullptr) {
    isDynamic = false;
  }else{
    isDynamic = true;
  }
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

      throw ompl::Exception("LineFraction out of bounds.");
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
      throw ompl::Exception("Invalid State");
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

void PathVisibilityChecker::computePathLength(ob::SpaceInformationPtr si_,const std::vector<ob::State*> &path, std::vector<double> &stateDistances, double &pathLength) 
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

//copies the createStateAt-method of the pathPathValidityChecker, but only for control::SpaceInformation
void PathVisibilityChecker::createStateAt(ob::SpaceInformationPtr si_,const std::vector<ob::State*> &path, const double &pathLength, const std::vector<double> &distances, const double newPosition, ob::State* s_interpolate) const 
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

//not necessary (probably), use si_->checkMotion instead

//bool PathVisibilityChecker::isStepDynamicallyFeasible(const ob::State* s_start, ob::State* s_target, const ompl::control::Control* c_previous, //ompl::control::Control* c_current, const double targetRegion, const ompl::control::SpaceInformation* siC, const //ompl::control::DirectedControlSamplerPtr sampler){
//  ob::State* s_target_copy = siC->allocState();
//  siC->copyState(s_target_copy, s_target);
//  sampler->sampleTo(c_current, c_previous, s_start, s_target);
//  if(siC->getStateSpace()->distance(s_target_copy, s_target) <= targetRegion){
//    siC->freeState(s_target_copy);
//    return true;
//  } else {
//    siC->freeState(s_target_copy);
//    return false;
//  }
//}

//@TODO Change this function so that it can be used as a directedControlSamplerAllocator
//allocator for the SimpleDirectedControlSampler
//change samples for higher/lower accuracy
//ompl::control::DirectedControlSamplerPtr simpleDirectedControlSamplerAllocator(const ompl::control::SpaceInformationPtr siC) const {
//    unsigned int samples = 20;
//    ompl::control::DirectedControlSamplerPtr sampler = std::make_shared<ompl::control::SimpleDirectedControlSampler>(siC, samples);
//    return sampler;
//}

bool PathVisibilityChecker::IsPathDynamicallyVisible(std::vector<ob::State*> &s1, std::vector<ob::State*> &s2, std::vector<ob::State*> &sLocal)
{
    std::cout << "Checked two different paths" << std::endl;
    siC = dynamic_cast<ompl::control::SpaceInformation*>(si_.get());
    //siC->setMotionValidator(std::make_shared<ob::DynamicalMotionValidator>(siC));
    //siC->setup();

    //initialize everything again
    ob::State* state_path_1 = siC->allocState();
    ob::State* state_path_2 = siC->allocState();
    std::vector<double> path_1_distances;
    std::vector<double> path_2_distances;
    double path_1_length = 0;
    double path_2_length = 0;

    //if(!(isPathDynamicallyFeasible(s1) && isPathDynamicallyFeasible(s2))){
    //  std::cout << "Initial Paths are dynamically not feasible" << std::endl;
    //  return false;
    //}
    
//#######################################################


    computePathLength(si_, s1, path_1_distances, path_1_length);
    computePathLength(si_, s2, path_2_distances, path_2_length);


    //@TODO Set own Allocator for DirectedControlSampler
    //siC->setDirectedControlSamplerAllocator(simpleDirectedControlSamplerAllocator);
    sDCSampler = siC->allocDirectedControlSampler();

    for(int i = 0; i < pathSamples-1.; i++){
      statesDyn.push_back(si_->allocState());
      si_->copyState(statesDyn.at(i), s1.at(0));
      statesDyn_next.push_back(si_->allocState());
    }

    for(uint k = 0; k < sLocal.size(); k++){
      std::cout << "Got to interpolation state " << k << std::endl;
      const double &pathspace_x = sLocal.at(k)->as<ob::RealVectorStateSpace::StateType>()->values[0];
      const double &pathspace_y = sLocal.at(k)->as<ob::RealVectorStateSpace::StateType>()->values[1];

      createStateAt(si_, s1, path_1_length, path_1_distances, pathspace_x*path_1_length, state_path_1);
      createStateAt(si_, s2, path_2_length, path_2_distances, pathspace_y*path_2_length, state_path_2);
      

      for(int i = 0; i < pathSamples - 1; i++) {
        //create States between the two given paths
        si_->getStateSpace()->interpolate(state_path_1, state_path_2, (i+1)/pathSamples, statesDyn_next.at(i));
        
        testCheckMotion(state_path_1, state_path_2);
	
        stepFeasible = si_->checkMotion(statesDyn.at(i), statesDyn_next.at(i));
        if(!stepFeasible){
	  //std::cout << "Paths were dynamically different" << std::endl;
          //return false;
        }
        //std::cout << k << std::endl;

	//save this set of statesDyn_next in statesDyn to use in next iteration
        si_->copyState(statesDyn.at(i), statesDyn_next.at(i));
      }
    }
    std::cout << "Paths were dynamically identical" << std::endl;
    return true;
}

void PathVisibilityChecker::testCheckMotion(const ob::State* s1, const ob::State* s2){
  ob::State* s2_copy = siC->allocState();
  ompl::control::Control* cont = siC->allocControl();
  sDCSampler->sampleTo(cont, s1, s2_copy);
  //s2 is reachable
  bool motionFeasible = siC->checkMotion(s1,s2_copy);
  if(motionFeasible){
    std::cout << "correct" << std::endl;
  } else {
    std::cout << "false false false" << std::endl;
  }

}

bool PathVisibilityChecker::isPathDynamicallyFeasible(const std::vector<ompl::base::State*> path) const {
  for(unsigned int i = 0; i < path.size() - 1; i++) {
    bool stepValid = siC->checkMotion(path.at(i), path.at(i+1));
    if(!stepValid) {
      return false;
    }
  }
  std::cout << "checked initial paths, are ok" << std::endl;
  return true;
}


bool PathVisibilityChecker::IsPathVisible(std::vector<ob::State*> &s1, std::vector<ob::State*> &s2)
{
  //Disable logging for Checker
  ompl::msg::LogLevel logLevelInitial = ompl::msg::getLogLevel();
  ompl::msg::setLogLevel(ompl::msg::LOG_NONE);
  // ompl::msg::setLogLevel(ompl::msg::LOG_DEV2);

  ////Assert Non-empty paths with at least a designated end and start 
  ////configuration
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
  }


  ss->setStateValidityChecker( std::make_shared<pathPathValidityChecker>(si_, si_local,  s1, s2) );
  ob::PlannerPtr linear_homotopy_planner = std::make_shared<og::RRTConnect>(si_local);

  ss->setStartAndGoalStates(*start, *goal, epsilon_goalregion);
  ss->setPlanner(linear_homotopy_planner);
  ss->setup();

  ob::PlannerTerminationCondition ptc( ob::timedPlannerTerminationCondition(max_planning_time_path_path) );

  ss->solve(ptc);

  bool solved = ss->haveExactSolutionPath();


  //############################################################################
  //############################################################################
  if(solved && isDynamic){
    std::vector<ob::State*> spath = ss->getSolutionPath().getStates();
    solved = IsPathDynamicallyVisible(s1, s2, spath);
  }
  //############################################################################
  //############################################################################



  ompl::msg::setLogLevel(logLevelInitial);
  return solved;

}
