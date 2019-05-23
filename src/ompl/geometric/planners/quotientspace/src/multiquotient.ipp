#include "plannerdata_vertex_annotated.h"
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/util/Time.h>
#include <queue>

using namespace og;

template <class T, class Tlast>
MultiQuotient<T,Tlast>::MultiQuotient(std::vector<ob::SpaceInformationPtr> &si_vec_, std::string type):
  ob::Planner(si_vec_.back(), type), si_vec(si_vec_)
{
  T::resetCounter();
  for(uint k = 0; k < si_vec.size(); k++){
    og::Quotient* parent = nullptr;
    if(k>0) parent = quotientSpaces.back();

    if(k>0 && k>=si_vec.size()-1){
      Tlast* ss = new Tlast(si_vec.at(k), parent);
      quotientSpaces.push_back(ss);
    }else{
      T* ss = new T(si_vec.at(k), parent);
      quotientSpaces.push_back(ss);
    }
    quotientSpaces.back()->SetLevel(k);
  }
  stopAtLevel = quotientSpaces.size();
  if(DEBUG) std::cout << "Created hierarchy with " << si_vec.size() << " levels." << std::endl;
}

template <class T, class Tlast>
int MultiQuotient<T,Tlast>::GetLevels() const
{
  return stopAtLevel;
}
template <class T, class Tlast>
std::vector<int> MultiQuotient<T,Tlast>::GetNodes() const
{
  std::vector<int> nodesPerLevel;
  for(uint k = 0; k < stopAtLevel; k++){
    uint Nk = quotientSpaces.at(k)->GetTotalNumberOfSamples();
    nodesPerLevel.push_back(Nk);
  }
  return nodesPerLevel;
}
template <class T, class Tlast>
std::vector<int> MultiQuotient<T,Tlast>::GetFeasibleNodes() const
{
  std::vector<int> feasibleNodesPerLevel;
  for(uint k = 0; k < quotientSpaces.size(); k++){
    uint Nk = quotientSpaces.at(k)->GetTotalNumberOfFeasibleSamples();
    feasibleNodesPerLevel.push_back(Nk);
  }
  return feasibleNodesPerLevel;
}


template <class T, class Tlast>
std::vector<int> MultiQuotient<T,Tlast>::GetDimensionsPerLevel() const
{
  std::vector<int> dimensionsPerLevel;
  for(uint k = 0; k < quotientSpaces.size(); k++){
    uint Nk = quotientSpaces.at(k)->GetDimension();
    dimensionsPerLevel.push_back(Nk);
  }
  return dimensionsPerLevel;
}


template <class T, class Tlast>
MultiQuotient<T,Tlast>::~MultiQuotient(){
}

template <class T, class Tlast>
void MultiQuotient<T,Tlast>::setup(){

  Planner::setup();
  for(uint k = 0; k < stopAtLevel; k++){
    quotientSpaces.at(k)->setup();
  }
  currentQuotientLevel = 0;
}

template <class T, class Tlast>
void MultiQuotient<T,Tlast>::SetStopLevel(uint level_)
{
  if(level_ > quotientSpaces.size()){
    stopAtLevel = quotientSpaces.size();
  }else{
    stopAtLevel = level_;
  }
  std::cout << "new stop level: " << stopAtLevel << " from " << quotientSpaces.size() << std::endl;
}

template <class T, class Tlast>
void MultiQuotient<T,Tlast>::clear(){
  Planner::clear();

  for(uint k = 0; k < quotientSpaces.size(); k++){
    quotientSpaces.at(k)->clear();
  }
  currentQuotientLevel = 0;

  while(!Q.empty()) Q.pop();
  foundKLevelSolution = false;

  solutions.clear();
  pdef_->clearSolutionPaths();
  for(uint k = 0; k < pdef_vec.size(); k++){
    pdef_vec.at(k)->clearSolutionPaths();
  }
}

template <class T, class Tlast>
ob::PlannerStatus MultiQuotient<T,Tlast>::solve(const base::PlannerTerminationCondition &ptc)
{
  
  static const double T_GROW = 0.01; //time to grow before Checking if solution exists

  ompl::time::point t_start = ompl::time::now();

  for(uint k = currentQuotientLevel; k < stopAtLevel; k++){
    foundKLevelSolution = false;

    if(Q.size()<=currentQuotientLevel) Q.push(quotientSpaces.at(k));

    base::PlannerTerminationCondition ptcOrSolutionFound([this, &ptc]
                                   { return ptc || foundKLevelSolution; });

    while (!ptcOrSolutionFound())
    {
      og::Quotient* jQuotient = Q.top();
      Q.pop();
      jQuotient->Grow(T_GROW);

      bool hasSolution = quotientSpaces.at(k)->HasSolution();
      if(hasSolution){
        base::PathPtr sol_k;
        quotientSpaces.at(k)->GetSolution(sol_k);
        solutions.push_back(sol_k);
        if(DEBUG){
          double t_k_end = ompl::time::seconds(ompl::time::now() - t_start);
          std::cout << std::string(80, '#') << std::endl;
          std::cout << "Found Solution on Level " << k << " after " << t_k_end << " seconds." << std::endl;
          std::cout << *quotientSpaces.at(k) << std::endl;
        }
        foundKLevelSolution = true;
        currentQuotientLevel = k+1;
      }
      Q.push(jQuotient);
    }

    if(!foundKLevelSolution){
      if(DEBUG){
        // std::cout << std::string(80, '#') << std::endl;
        // std::cout << "could not find a solution on level " << k << std::endl;
        std::cout << std::string(80, '#') << std::endl;
        for(uint i = 0; i < k+1; i++){
          std::cout << *quotientSpaces.at(i) << std::endl;
        }
      }
      return ob::PlannerStatus::TIMEOUT;
    }
  }
  if(DEBUG){
    double t_end = ompl::time::seconds(ompl::time::now() - t_start);
    std::cout << std::string(80, '#') << std::endl;
    std::cout << "Found exact solution after " << t_end << " seconds." << std::endl;
    std::cout << std::string(80, '#') << std::endl;
  }

  base::PathPtr sol;
  if(quotientSpaces.at(currentQuotientLevel-1)->GetSolution(sol))
  {
    base::PlannerSolution psol(sol);
    psol.setPlannerName(getName());
    pdef_->addSolutionPath(psol);
  }

  return ob::PlannerStatus::EXACT_SOLUTION;
}



template <class T, class Tlast>
void MultiQuotient<T,Tlast>::setProblemDefinition(std::vector<ob::ProblemDefinitionPtr> &pdef_)
{
  pdef_vec = pdef_;
  ob::Planner::setProblemDefinition(pdef_vec.back());
  for(uint k = 0; k < pdef_vec.size(); k++){
    quotientSpaces.at(k)->setProblemDefinition(pdef_vec.at(k));
  }
}

template <class T, class Tlast>
void MultiQuotient<T,Tlast>::setProblemDefinition(const ob::ProblemDefinitionPtr &pdef)
{
  this->Planner::setProblemDefinition(pdef);
}

template <class T, class Tlast>
void MultiQuotient<T,Tlast>::getPlannerData(ob::PlannerData &data) const
{
  uint Nvertices = data.numVertices();
  if(Nvertices>0){
    std::cout << "cannot get planner data if plannerdata is already populated" << std::endl;
    std::cout << "PlannerData has " << Nvertices << " vertices." << std::endl;
    exit(0);
  }

  uint K = std::min(solutions.size()+1,quotientSpaces.size());
  K = std::min(K, stopAtLevel);

  std::vector<int> fn = GetFeasibleNodes();
  std::vector<int> n = GetNodes();
  int fn_sum = 0;
  int n_sum = 0;
  for(uint k = 0; k < fn.size(); k++){
    std::cout << fn.at(k) << "/" << n.at(k) << std::endl;
    fn_sum += fn.at(k);
    n_sum += n.at(k);
  }
  std::cout << std::string(80, '-') << std::endl;
  std::cout << fn_sum << "/" << n_sum << std::endl;

  for(uint k = 0; k < K; k++){
    og::Quotient *Qk = quotientSpaces.at(k);
    Qk->getPlannerData(data);

    //label all new vertices
    uint ctr = 0;
    for(uint vidx = Nvertices; vidx < data.numVertices(); vidx++){
      PlannerDataVertexAnnotated &v = *static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(vidx));
      v.SetLevel(k);
      v.SetMaxLevel(K);

      ob::State *s_lift = Qk->getSpaceInformation()->cloneState(v.getState());
      v.setQuotientState(s_lift);

      for(uint m = k+1; m < quotientSpaces.size(); m++){
        og::Quotient *Qm = quotientSpaces.at(m);

        if(Qm->GetX1() != nullptr){
          ob::State *s_X1 = Qm->GetX1()->allocState();
          ob::State *s_Q1 = Qm->getSpaceInformation()->allocState();
          if(Qm->GetX1()->getStateSpace()->getType() == ob::STATE_SPACE_SO3) {
            static_cast<ob::SO3StateSpace::StateType*>(s_X1)->setIdentity();
          }
          if(Qm->GetX1()->getStateSpace()->getType() == ob::STATE_SPACE_SO2) {
            static_cast<ob::SO2StateSpace::StateType*>(s_X1)->setIdentity();
          }
          Qm->MergeStates(s_lift, s_X1, s_Q1);
          s_lift = Qm->getSpaceInformation()->cloneState(s_Q1);

          Qm->GetX1()->freeState(s_X1);
          Qm->GetQ1()->freeState(s_Q1);
        }
      }
      v.setState(s_lift);
      ctr++;
    }
    Nvertices = data.numVertices();
  }

  // for(uint vidx = 0; vidx < data.numVertices(); vidx++){
  //   PlannerDataVertexAnnotated &v = *static_cast<PlannerDataVertexAnnotated*>(&data.getVertex(vidx));
  //   std::cout << "[MultiQuotient] vertex " << vidx << " " << v.GetOpenNeighborhoodDistance() << std::endl;
  // }
  //exit(0);
}
