#include <ompl/geometric/planners/quotientspace/datastructures/importance/Greedy.h>


ompl::geometric::BundleSpaceImportanceGreedy::BundleSpaceImportanceGreedy(BundleSpaceGraph* graph):
  BaseT(graph)
{
}

double ompl::geometric::BundleSpaceImportanceGreedy::eval()
{
    if(bundleSpaceGraph_->hasSolution() && bundleSpaceGraph_->hasChild())
    {
        return 0.0;
    }else{
        return 1.0;
    }
}

