#ifndef OMPL_GEOMETRIC_PLANNERS_EXPLORER_MotionExplorer_
#define OMPL_GEOMETRIC_PLANNERS_EXPLORER_MotionExplorer_
#include <ompl/geometric/planners/explorer/datastructures/MultiLevelPathSpace.h>
#include <ompl/geometric/planners/explorer/algorithms/MotionExplorerImpl.h>
#include <type_traits>
#include <queue>

namespace ompl
{
    namespace geometric
    {
        using MotionExplorer = MultiLevelPathSpace<MotionExplorerImpl>;
    }
}
#endif
