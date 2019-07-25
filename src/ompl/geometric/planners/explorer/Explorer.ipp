#include "QuotientGraphSparse.h"
#include <ompl/geometric/planners/quotientspace/PlannerDataVertexAnnotated.h>
#include <ompl/geometric/planners/quotientspace/QuotientSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/util/Time.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <queue>

using namespace og;
using namespace ob;

template <class T>
MotionExplorer<T>::MotionExplorer(std::vector<ob::SpaceInformationPtr> &siVec, std::string type)
  : ob::Planner(siVec.back(), type), siVec_(siVec)
{
    T::resetCounter();
    for (unsigned int k = 0; k < siVec_.size(); k++)
    {
        og::QuotientSpace *parent = nullptr;
        if (k > 0)
            parent = quotientSpaces_.back();

        T *ss = new T(siVec_.at(k), parent);
        quotientSpaces_.push_back(ss);
        quotientSpaces_.back()->setLevel(k);
    }
    root = quotientSpaces_.front();
    current = root;

    OMPL_DEVMSG1("Created hierarchy with %d leves.", siVec_.size());
    std::cout << quotientSpaces_.size() << std::endl;
}

template <class T>
MotionExplorer<T>::~MotionExplorer()
{
}

template <class T>
void MotionExplorer<T>::setup()
{
    Planner::setup();
    for (unsigned int k = 0; k < quotientSpaces_.size(); k++)
    {
        quotientSpaces_.at(k)->setup();
    }
}

template <class T>
void MotionExplorer<T>::clear()
{
    Planner::clear();

    for (unsigned int k = 0; k < quotientSpaces_.size(); k++)
    {
        quotientSpaces_.at(k)->clear();
    }

    pdef_->clearSolutionPaths();
    selectedPath_.clear();
}

template <class T>
void MotionExplorer<T>::setSelectedPath( std::vector<int> selectedPath){
  selectedPath_ = selectedPath;
  for(uint k = 0; k < selectedPath.size(); k++){
    //selected path implies path bias, which implies a sampling bias towards the
    //selected path
    quotientSpaces_.at(k)->selectedPath = selectedPath.at(k);
  }
  std::cout << "[SELECTION CHANGE] QuotientSpaces set to [";
  for(uint k = 0; k < selectedPath.size(); k++){
    int sk = selectedPath.at(k);
    std::cout << sk << " ";
  }
  std::cout << "]" << std::endl;
}

template <class T>
ob::PlannerStatus MotionExplorer<T>::solve(const ob::PlannerTerminationCondition &ptc)
{
  uint K = selectedPath_.size();
  if(K>=quotientSpaces_.size()){
    K = K-1;
  }
  og::QuotientGraphSparse *jQuotient = quotientSpaces_.at(K);
  std::cout << *jQuotient << std::endl;

  uint ctr = 0;
  uint M = jQuotient->getNumberOfPaths();
  uint Mg = M;

  while (!ptc())
  {
    // std::cout << "Growing QuotientSpace " << jQuotient->getName() << std::endl;
    jQuotient->grow();
    ctr++;
    Mg = jQuotient->getNumberOfPaths();
    //stop at topological phase shift
    if(Mg > M) return ob::PlannerStatus::APPROXIMATE_SOLUTION;
  }
  std::cout << "Grow QuotientSpace " << jQuotient->getName() << " for " << ctr << " iters." << std::endl;
  std::cout << "Changed #paths from " << M << " to " << Mg << std::endl;

  return ob::PlannerStatus::TIMEOUT;
}

template <class T>
void MotionExplorer<T>::setProblemDefinition(const ob::ProblemDefinitionPtr &pdef)
{
    this->Planner::setProblemDefinition(pdef);

    //Compute projection of qInit and qGoal onto QuotientSpaces
    ob::Goal *goal = pdef_->getGoal().get();
    ob::GoalState *goalRegion = dynamic_cast<base::GoalState *>(goal);
    double epsilon = goalRegion->getThreshold();
    assert(quotientSpaces_.size() == siVec_.size());

    ob::State *sInit = pdef->getStartState(0);
    ob::State *sGoal = goalRegion->getState();

    OMPL_DEVMSG1("Projecting start and goal onto QuotientSpaces.");

    quotientSpaces_.back()->setProblemDefinition(pdef);
    for (unsigned int k = siVec_.size()-1; k > 0 ; k--)
    {
        og::QuotientSpace *quotientParent = quotientSpaces_.at(k);
        og::QuotientSpace *quotientChild = quotientSpaces_.at(k-1);
        ob::SpaceInformationPtr sik = quotientChild->getSpaceInformation();
        ob::ProblemDefinitionPtr pdefk = std::make_shared<base::ProblemDefinition>(sik);

        ob::State *sInitK = sik->allocState();
        ob::State *sGoalK = sik->allocState();

        quotientParent->projectQ0Subspace(sInit, sInitK);
        quotientParent->projectQ0Subspace(sGoal, sGoalK);

        pdefk->setStartAndGoalStates(sInitK, sGoalK, epsilon);
        quotientChild->setProblemDefinition(pdefk);

        sInit = sInitK;
        sGoal = sGoalK;
    }
}

template <class T>
void MotionExplorer<T>::getPlannerData(ob::PlannerData &data) const
{
    unsigned int Nvertices = data.numVertices();
    if (Nvertices > 0)
    {
        OMPL_ERROR("Cannot get planner data if plannerdata is already populated");
        OMPL_ERROR("PlannerData has %d vertices.", Nvertices);
        exit(0);
    }

    unsigned int K = quotientSpaces_.size();
    std::vector<uint> countVerticesPerQuotientSpace;

    for (unsigned int k = 0; k < K; k++)
    {
        og::QuotientSpace *Qk = quotientSpaces_.at(k);
        static_cast<QuotientGraphSparse*>(Qk)->enumerateAllPaths();
        Qk->getPlannerData(data);
        // label all new vertices
        unsigned int ctr = 0;

        for (unsigned int vidx = Nvertices; vidx < data.numVertices(); vidx++)
        {
          ob::PlannerDataVertexAnnotated &v = *static_cast<ob::PlannerDataVertexAnnotated *>(&data.getVertex(vidx));
            v.setLevel(k);
            v.setMaxLevel(K);

            ob::State *s_lift = Qk->getSpaceInformation()->cloneState(v.getState());
            v.setQuotientState(s_lift);

            for (unsigned int m = k + 1; m < quotientSpaces_.size(); m++)
            {
                og::QuotientSpace *Qm = quotientSpaces_.at(m);

                if (Qm->getX1() != nullptr)
                {
                    ob::State *s_X1 = Qm->getX1()->allocState();
                    ob::State *s_Q1 = Qm->getSpaceInformation()->allocState();
                    if (Qm->getX1()->getStateSpace()->getType() == ob::STATE_SPACE_SO3)
                    {
                        static_cast<ob::SO3StateSpace::StateType *>(s_X1)->setIdentity();
                    }
                    if (Qm->getX1()->getStateSpace()->getType() == ob::STATE_SPACE_SO2)
                    {
                        static_cast<ob::SO2StateSpace::StateType *>(s_X1)->setIdentity();
                    }
                    Qm->mergeStates(s_lift, s_X1, s_Q1);
                    s_lift = Qm->getSpaceInformation()->cloneState(s_Q1);

                    Qm->getX1()->freeState(s_X1);
                    Qm->getQ1()->freeState(s_Q1);
                }
            }
            v.setState(s_lift);
            ctr++;
        }
        countVerticesPerQuotientSpace.push_back(data.numVertices() - Nvertices);
        Nvertices = data.numVertices();

    }
    std::cout << "Created PlannerData with " << data.numVertices() << " vertices ";
    std::cout << "(";
    for(uint k = 0; k < countVerticesPerQuotientSpace.size(); k++){
       uint ck = countVerticesPerQuotientSpace.at(k);
       std::cout << ck << (k < countVerticesPerQuotientSpace.size()-1?", ":"");
    }
    std::cout << ")" << std::endl;
}

