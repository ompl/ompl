#include <ompl/geometric/planners/multilevel/datastructures/PlannerDataVertexAnnotated.h>
#include <ompl/geometric/planners/multilevel/datastructures/BundleSpace.h>
#include <ompl/geometric/planners/explorer/datastructures/PathSpace.h>
#include <ompl/geometric/planners/explorer/algorithms/PathSpaceSparseOptimization.h>

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/util/Time.h>
#include <ompl/util/Exception.h>
#include <queue>

using namespace og;
using namespace ob;

template <class T>
ompl::geometric::MultiLevelPathSpace<T>::MultiLevelPathSpace(std::vector<ob::SpaceInformationPtr> &siVec, std::string type)
  : BaseT(siVec, type)
{
    root = this->bundleSpaces_.front();
    current = root;
}

template <class T>
ompl::geometric::MultiLevelPathSpace<T>::~MultiLevelPathSpace()
{
}

template <class T>
void ompl::geometric::MultiLevelPathSpace<T>::setup()
{
    BaseT::setup();
}

template <class T>
void ompl::geometric::MultiLevelPathSpace<T>::clear()
{
    BaseT::clear();
    selectedLocalMinimum_.clear();
    root = nullptr;
    current = nullptr;
}

template <class T>
void ompl::geometric::MultiLevelPathSpace<T>::setLocalMinimumSelection( std::vector<int> selection)
{
    std::vector<int> oldSelectedLocalMinimum = selectedLocalMinimum_;
    selectedLocalMinimum_ = selection;

    unsigned int N = selectedLocalMinimum_.size();
    unsigned int Nold = oldSelectedLocalMinimum.size();

    for(uint k = 0; k < selectedLocalMinimum_.size(); k++){
      og::PathSpace *pathspace =
        static_cast<og::PathSpace*>(this->bundleSpaces_.at(k));
          
      pathspace->setSelectedPath(selectedLocalMinimum_.at(k));
    }

    std::cout << "[SELECTION CHANGE] BundleSpaces set from [";
    for(uint k = 0; k < oldSelectedLocalMinimum.size(); k++){
      int sk = oldSelectedLocalMinimum.at(k);
      std::cout << sk << " ";
    }
    std::cout << "] to [";
    for(uint k = 0; k < selectedLocalMinimum_.size(); k++){
      int sk = selectedLocalMinimum_.at(k);
      std::cout << sk << " ";
    }
    std::cout << "]" << std::endl;

    //User changed to different folder (and the files inside have not been
    //generated yet)
    if(N==Nold && N>0 && (N < this->bundleSpaces_.size())){
        unsigned int M = selectedLocalMinimum_.back();
        unsigned int Mold = oldSelectedLocalMinimum.back();
        if(M!=Mold){
            std::cout << "Changed Folder. Clear Bundle-spaces [" 
              << N << "]" << std::endl;
            this->bundleSpaces_.at(N)->clear();
        }

    }

}

template <class T>
ob::PlannerStatus MultiLevelPathSpace<T>::solve(const ob::PlannerTerminationCondition &ptc)
{
    ompl::msg::setLogLevel(ompl::msg::LOG_DEV2);
    uint K = selectedLocalMinimum_.size();

    if(K>=this->bundleSpaces_.size()){
        K = K-1;
    }

    //find lowest dimensional QS which has a solution. Then take the next QS to
    //expand
    while(K>0)
    {
        og::PathSpace *kBundle =
          static_cast<og::PathSpace*>(this->bundleSpaces_.at(K-1));
        if(kBundle->getNumberOfPaths()>0){
          break;
        }else{
          K = K-1;
        }
    }

    //Check which 
    og::BundleSpace *jBundle =
      static_cast<og::BundleSpace*>(this->bundleSpaces_.at(K));

    uint ctr = 0;

    std::cout << "Searching space " << jBundle->getName() << " until solution found." << std::endl;

    //grow at least PTC, then grow until solution (that way we do not stop after
    //finding just one single path)
    while (!ptc())
    {
        jBundle->grow();
        ctr++;
    }
    while (true)
    {
        jBundle->grow();
        ctr++;
        if(jBundle->hasSolution())
        {
           break;
        }
        if(ctr%100==0)
        {
            break;
        }
    }
    std::cout << std::string(80, '-') << std::endl;
    std::cout << *jBundle << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    return ob::PlannerStatus::TIMEOUT;
}

template <class T>
void MultiLevelPathSpace<T>::getPlannerData(ob::PlannerData &data) const
{
    unsigned int Nvertices = data.numVertices();
    if (Nvertices > 0)
    {
        OMPL_ERROR("PlannerData has %d vertices.", Nvertices);
        throw ompl::Exception("cannot get planner data if plannerdata is already populated");
    }

    unsigned int K = this->bundleSpaces_.size();
    std::vector<uint> countVerticesPerBundleSpace;

    for (unsigned int k = 0; k < K; k++)
    {
        og::BundleSpaceGraph *Qk = static_cast<BundleSpaceGraph*>(this->bundleSpaces_.at(k));

        PathSpaceSparseOptimization *Qk_tmp = dynamic_cast<PathSpaceSparseOptimization*>(Qk);
        if(Qk_tmp != nullptr)
        {
            Qk_tmp->enumerateAllPaths();
            Qk_tmp->getPlannerData(data);
        }else
        {
            og::PathSpace *Pk = static_cast<PathSpace*>(this->bundleSpaces_.at(k));
            Pk->getPlannerData(data, Qk);
        }

        // label all new vertices
        unsigned int ctr = 0;

        for (unsigned int vidx = Nvertices; vidx < data.numVertices(); vidx++)
        {
          ob::PlannerDataVertexAnnotated &v = *static_cast<ob::PlannerDataVertexAnnotated *>(&data.getVertex(vidx));
            v.setLevel(k);
            v.setMaxLevel(K);

            ob::State *s_lift = Qk->getSpaceInformation()->cloneState(v.getState());
            v.setBaseState(s_lift);

            for (unsigned int m = k + 1; m < this->bundleSpaces_.size(); m++)
            {
                const og::BundleSpace *Qm = this->bundleSpaces_.at(m);

                if (Qm->getFiberDimension() > 0)
                {
                    ob::State *s_Bundle = Qm->allocIdentityStateBundle();
                    ob::State *s_Fiber = Qm->allocIdentityStateFiber();

                    Qm->liftState(s_lift, s_Fiber, s_Bundle);

                    Qm->getBase()->freeState(s_lift);
                    // TODO: Qm->liftState(s_lift, s_Fiber, s_Bundle);
                    s_lift = Qm->getBundle()->cloneState(s_Bundle);

                    Qm->getBundle()->freeState(s_Bundle);
                    Qm->getFiber()->freeState(s_Fiber);
                }
            }
            v.setState(s_lift);
            ctr++;
        }
        countVerticesPerBundleSpace.push_back(data.numVertices() - Nvertices);
        Nvertices = data.numVertices();

    }
    std::cout << "Created PlannerData with " << data.numVertices() << " vertices ";
    std::cout << "(";
    for(uint k = 0; k < countVerticesPerBundleSpace.size(); k++){
       uint ck = countVerticesPerBundleSpace.at(k);
       std::cout << ck << (k < countVerticesPerBundleSpace.size()-1?", ":"");
    }
    std::cout << ")" << std::endl;
}

