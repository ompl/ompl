#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_EXPLORERTEST_
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_EXPLORERTEST_
#include <ompl/geometric/planners/multilevel/datastructures/BundleSpaceSequence.h>
#include <ompl/geometric/planners/explorer/algorithms/MotionExplorerQMPImpl.h>
#include <ompl/geometric/planners/explorer/datastructures/MultiLevelPathSpace.h>

namespace ompl
{
    namespace geometric
    {

        using MotionExplorerQMP = MultiLevelPathSpace<MotionExplorerQMPImpl>;

    }  // namespace geometric
}  // namespace ompl

#endif
