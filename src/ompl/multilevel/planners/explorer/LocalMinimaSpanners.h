#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_LOCALMINIMASPANNERS__
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_LOCALMINIMASPANNERS__
#include <ompl/multilevel/datastructures/BundleSpaceSequence.h>
#include <ompl/multilevel/planners/explorer/algorithms/PathSpaceSparse.h>
#include <ompl/multilevel/planners/explorer/datastructures/MultiLevelPathSpace.h>

namespace ompl
{
    namespace multilevel
    {
        using LocalMinimaSpanners = MultiLevelPathSpace<PathSpaceSparse>;
    }
}

#endif
