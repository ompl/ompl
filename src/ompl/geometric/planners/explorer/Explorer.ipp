#include "QuotientGraphSparse.h"
#include <ompl/geometric/planners/quotientspace/PlannerDataVertexAnnotated.h>
#include <ompl/geometric/planners/quotientspace/QuotientSpace.h>
#include <ompl/geometric/planners/quotientspace/MultiQuotient.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/util/Time.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <queue>

using namespace og;
using namespace ob;

template <class T>
MotionExplorer<T>::MotionExplorer(std::vector<ob::SpaceInformationPtr> &siVec, std::string type)
  : BaseT(siVec, type)
{
    root = static_cast<og::QuotientGraphSparse*>(this->quotientSpaces_.front());
    current = root;

    // OMPL_DEVMSG1("Created hierarchy with %d leaves.", siVec_.size());
}

template <class T>
MotionExplorer<T>::~MotionExplorer()
{
}

template <class T>
void MotionExplorer<T>::setup()
{
    Planner::setup();
}

template <class T>
void MotionExplorer<T>::clear()
{
    Planner::clear();
    selectedPath_.clear();
    root = nullptr;
    current = nullptr;
}

template <class T>
void MotionExplorer<T>::setSelectedPath( std::vector<int> selectedPath){
    selectedPath_ = selectedPath;
    for(uint k = 0; k < selectedPath.size(); k++){
      //selected path implies path bias, which implies a sampling bias towards the
      //selected path
      og::QuotientGraphSparse *qgraph = 
        static_cast<og::QuotientGraphSparse*>(this->quotientSpaces_.at(k));
          
      qgraph->selectedPath = selectedPath.at(k);
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
  if(K>=this->quotientSpaces_.size()){
    K = K-1;
  }
  og::QuotientGraphSparse *jQuotient = 
    static_cast<og::QuotientGraphSparse*>(this->quotientSpaces_.at(K));
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
void MotionExplorer<T>::getPlannerData(ob::PlannerData &data) const
{
    unsigned int Nvertices = data.numVertices();
    if (Nvertices > 0)
    {
        OMPL_ERROR("Cannot get planner data if plannerdata is already populated");
        OMPL_ERROR("PlannerData has %d vertices.", Nvertices);
        exit(0);
    }

    unsigned int K = this->quotientSpaces_.size();
    std::vector<uint> countVerticesPerQuotientSpace;

    for (unsigned int k = 0; k < K; k++)
    {
        og::QuotientSpace *Qk = this->quotientSpaces_.at(k);
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

            for (unsigned int m = k + 1; m < this->quotientSpaces_.size(); m++)
            {
                og::QuotientSpace *Qm = this->quotientSpaces_.at(m);

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

