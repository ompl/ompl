#include <ompl/multilevel/datastructures/PlannerDataVertexAnnotated.h>
#include <ompl/multilevel/datastructures/BundleSpace.h>
#include <ompl/multilevel/planners/explorer/datastructures/PathSpace.h>
#include <ompl/multilevel/planners/explorer/datastructures/LocalMinimaTree.h>
// #include <ompl/multilevel/planners/explorer/algorithms/PathSpaceSparseOptimization.h>

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/util/Time.h>
#include <ompl/util/Exception.h>
#include <queue>

using namespace ompl::multilevel;

template <class T>
ompl::multilevel::MultiLevelPathSpace<T>::MultiLevelPathSpace(std::vector<base::SpaceInformationPtr> &siVec,
                                                              std::string type)
  : BaseT(siVec, type)
{
    current = this->bundleSpaces_.front();
    localMinimaTree_ = std::make_shared<LocalMinimaTree>(siVec);

    // connect all quotient spaces to local minima tree
    for (uint k = 0; k < this->bundleSpaces_.size(); k++)
    {
        PathSpace *Pk = static_cast<PathSpace *>(this->bundleSpaces_.at(k));
        Pk->setLocalMinimaTree(localMinimaTree_);
    }
}

template <class T>
ompl::multilevel::MultiLevelPathSpace<T>::~MultiLevelPathSpace()
{
}

template <class T>
void ompl::multilevel::MultiLevelPathSpace<T>::setup()
{
    BaseT::setup();
}

template <class T>
void ompl::multilevel::MultiLevelPathSpace<T>::clear()
{
    BaseT::clear();
    localMinimaTree_->clear();
    current = this->bundleSpaces_.front();
    OMPL_INFORM("Cleared multilevel path space structure.");
}
template <class T>
LocalMinimaTreePtr &MultiLevelPathSpace<T>::getLocalMinimaTree()
{
    return localMinimaTree_;
}

template <class T>
ompl::base::PlannerStatus MultiLevelPathSpace<T>::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
    ompl::msg::setLogLevel(ompl::msg::LOG_DEV2);

    std::vector<int> selectedLocalMinimum = localMinimaTree_->getSelectedPathIndex();
    uint K = selectedLocalMinimum.size();

    if (K >= this->bundleSpaces_.size())
    {
        K = K - 1;
    }

    // find lowest dimensional QS which has a solution. Then take the next QS to
    // expand
    while (K > 0)
    {
        if (localMinimaTree_->getNumberOfMinima(K - 1) > 0)
        {
            break;
        }
        else
        {
            K = K - 1;
        }
    }

    // Check which
    BundleSpace *jBundle = static_cast<BundleSpace *>(this->bundleSpaces_.at(K));

    uint ctr = 0;

    // grow at least PTC, then grow until solution (that way we do not stop after
    // finding just one single path)
    while (!ptc())
    {
        jBundle->grow();
        ctr++;
    }
    return base::PlannerStatus::TIMEOUT;
}

template <class T>
void MultiLevelPathSpace<T>::getPlannerData(base::PlannerData &data) const
{
    // TODO: just call BaseT (better: remove completely)
    unsigned int Nvertices = data.numVertices();
    if (Nvertices > 0)
    {
        OMPL_ERROR("PlannerData has %d vertices.", Nvertices);
        throw ompl::Exception("cannot get planner data if plannerdata is already populated");
    }

    unsigned int K = this->bundleSpaces_.size();
    std::vector<uint> countVerticesPerBundleSpace;

    BundleSpace *Qlast = this->bundleSpaces_.back();
    for (unsigned int k = 0; k < K; k++)
    {
        BundleSpaceGraph *Qk = static_cast<BundleSpaceGraph *>(this->bundleSpaces_.at(k));

        // PathSpaceSparseOptimization *Qk_tmp = dynamic_cast<PathSpaceSparseOptimization*>(Qk);
        // if(Qk_tmp != nullptr)
        // {
        //     Qk_tmp->enumerateAllPaths();
        // }
        Qk->getPlannerData(data);

        for (unsigned int vidx = Nvertices; vidx < data.numVertices(); vidx++)
        {
            multilevel::PlannerDataVertexAnnotated &v =
                *static_cast<multilevel::PlannerDataVertexAnnotated *>(&data.getVertex(vidx));
            v.setLevel(k);
            v.setMaxLevel(K);

            base::State *s_lift = BaseT::getTotalState(k, v.getBaseState());
            v.setTotalState(s_lift, Qlast->getBundle());
        }
        countVerticesPerBundleSpace.push_back(data.numVertices() - Nvertices);
        Nvertices = data.numVertices();
    }
}
