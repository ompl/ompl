#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_EXPLORERQMP__
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_EXPLORERQMP__
#include <ompl/geometric/planners/multilevel/datastructures/BundleSpaceSequence.h>
#include <ompl/geometric/planners/explorer/algorithms/PathSpaceQMP.h>
#include <ompl/geometric/planners/explorer/datastructures/MultiLevelPathSpace.h>

namespace ompl
{
    namespace geometric
    {

        using MotionExplorerQMP = MultiLevelPathSpace<PathSpaceQMP>;

    }  // namespace geometric
}  // namespace ompl

#endif
