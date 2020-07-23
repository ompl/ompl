#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_EXPLORERQMP__
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_EXPLORERQMP__
#include <ompl/multilevel/datastructures/BundleSpaceSequence.h>
#include <ompl/multilevel/planners/explorer/algorithms/PathSpaceQMP.h>
#include <ompl/multilevel/planners/explorer/datastructures/MultiLevelPathSpace.h>

namespace ompl
{
    namespace multilevel
    {
        using MotionExplorerQMP = MultiLevelPathSpace<PathSpaceQMP>;
    }
}

#endif
