#include <ompl/geometric/planners/quotientspace/datastructures/PlannerDataVertexAnnotated.h>
#include <ompl/geometric/planners/quotientspace/datastructures/QuotientSpace.h>
#include <ompl/geometric/planners/quotientspace/datastructures/QuotientSpaceGraphSparse.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/util/Time.h>
#include <ompl/util/Exception.h>
#include <queue>

using namespace og;
using namespace ob;

template <class T>
ompl::geometric::MotionExplorerImpl<T>::MotionExplorerImpl(std::vector<ob::SpaceInformationPtr> &siVec, std::string type)
  : BaseT(siVec, type)
{
    root = static_cast<og::QuotientSpaceGraphSparse*>(this->quotientSpaces_.front());
    current = root;
}

template <class T>
ompl::geometric::MotionExplorerImpl<T>::~MotionExplorerImpl()
{
}

template <class T>
void ompl::geometric::MotionExplorerImpl<T>::setup()
{
    BaseT::setup();
}

template <class T>
void ompl::geometric::MotionExplorerImpl<T>::clear()
{
    BaseT::clear();
    selectedPath_.clear();
    root = nullptr;
    current = nullptr;
}

template <class T>
void ompl::geometric::MotionExplorerImpl<T>::setSelectedPath( std::vector<int> selectedPath){
    std::vector<int> oldSelectedPath = selectedPath_;
    unsigned int N = selectedPath.size();
    unsigned int Nold = oldSelectedPath.size();

    selectedPath_ = selectedPath;
    for(uint k = 0; k < selectedPath.size(); k++){
      //selected path implies path bias, which implies a sampling bias towards the
      //selected path
      og::QuotientSpaceGraphSparse *qgraph = 
        static_cast<og::QuotientSpaceGraphSparse*>(this->quotientSpaces_.at(k));
          
      qgraph->selectedPath = selectedPath.at(k);
    }

    std::cout << "[SELECTION CHANGE] QuotientSpaces set from [";
    for(uint k = 0; k < oldSelectedPath.size(); k++){
      int sk = oldSelectedPath.at(k);
      std::cout << sk << " ";
    }
    std::cout << "] to [";
    for(uint k = 0; k < selectedPath.size(); k++){
      int sk = selectedPath.at(k);
      std::cout << sk << " ";
    }
    std::cout << "]" << std::endl;

    //User changed to different folder (and the files inside have not been
    //generated yet)
    if(N==Nold && N>0 && (N < this->quotientSpaces_.size())){
        unsigned int M = selectedPath.back();
        unsigned int Mold = oldSelectedPath.back();
        if(M!=Mold){
            std::cout << "Changed Folder. Clear quotient-spaces [" 
              << N << "]" << std::endl;
            this->quotientSpaces_.at(N)->clear();
        }

    }

}

template <class T>
ob::PlannerStatus MotionExplorerImpl<T>::solve(const ob::PlannerTerminationCondition &ptc)
{
    ompl::msg::setLogLevel(ompl::msg::LOG_DEV2);
    uint K = selectedPath_.size();

    if(K>=this->quotientSpaces_.size()){
        K = K-1;
    }

    //find lowest dimensional QS which has a solution. Then take the next QS to
    //expand
    while(K>0)
    {
        og::QuotientSpaceGraphSparse *kQuotient = 
          static_cast<og::QuotientSpaceGraphSparse*>(this->quotientSpaces_.at(K-1));
        if(kQuotient->getNumberOfPaths()>0){
          break;
        }else{
          K = K-1;
        }
    }

    //Check which 
    og::QuotientSpaceGraphSparse *jQuotient = 
      static_cast<og::QuotientSpaceGraphSparse*>(this->quotientSpaces_.at(K));

    uint ctr = 0;

    std::cout << "Searching space " << jQuotient->getName() << " until solution found." << std::endl;

    //grow at least PTC, then grow until solution (that way we do not stop after
    //finding just one single path)
    while (!ptc())
    {
        jQuotient->grow();
        ctr++;
    }
    while (true)
    {
        jQuotient->grow();
        ctr++;
        if(jQuotient->hasSolution()){
          //Note: jQuotient will not have solution, because we have no
          //enumerated paths
          std::cout << "has solution" << std::endl;
          break;
        }
        if(ctr%10000==0){
            std::cout << "CURRENT STATUS" << std::endl;
            std::cout << *jQuotient << std::endl;
        }
    }
    std::cout << std::string(80, '-') << std::endl;
    std::cout << *jQuotient << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    return ob::PlannerStatus::TIMEOUT;
}

template <class T>
void MotionExplorerImpl<T>::getPlannerData(ob::PlannerData &data) const
{
    unsigned int Nvertices = data.numVertices();
    if (Nvertices > 0)
    {
        OMPL_ERROR("PlannerData has %d vertices.", Nvertices);
        throw ompl::Exception("cannot get planner data if plannerdata is already populated");
    }

    unsigned int K = this->quotientSpaces_.size();
    std::vector<uint> countVerticesPerQuotientSpace;

    for (unsigned int k = 0; k < K; k++)
    {
        og::QuotientSpace *Qk = this->quotientSpaces_.at(k);
        static_cast<QuotientSpaceGraphSparse*>(Qk)->enumerateAllPaths();
        static_cast<QuotientSpaceGraphSparse*>(Qk)->getPlannerData(data);
        // Qk->getPlannerData(data);
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

