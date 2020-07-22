#ifndef OMPL_MULTILEVEL_PLANNERS_EXPLORER_MotionExplorer_
#define OMPL_MULTILEVEL_PLANNERS_EXPLORER_MotionExplorer_
#include <ompl/multilevel/planners/explorer/datastructures/MultiLevelPathSpace.h>
#include <ompl/multilevel/planners/explorer/algorithms/PathSpaceSparseOptimization.h>
#include <type_traits>
#include <queue>

namespace ompl
{
    namespace multilevel
    {
        using MotionExplorer = MultiLevelPathSpace<PathSpaceSparseOptimization>;
    }
}
#endif
