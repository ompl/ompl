#ifndef OMPL_GEOMETRIC_PLANNERS_EXPLORER_MotionExplorer_
#define OMPL_GEOMETRIC_PLANNERS_EXPLORER_MotionExplorer_
#include "MultiQuotientExplorer.h"
#include <ompl/geometric/planners/explorer/ExplorerImpl.h>
#include <type_traits>
#include <queue>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl
{
    namespace geometric
    {
        typedef og::MotionExplorerImpl<og::ExplorerImpl> MotionExplorer;
    }
}
#endif
