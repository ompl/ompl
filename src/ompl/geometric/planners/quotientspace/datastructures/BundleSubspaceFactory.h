#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_SUBSPACE_FACTORY__
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_SUBSPACE_FACTORY__
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include "BundleSubspace.h"
namespace ompl
{
    namespace geometric
    {
        class BundleSubspaceFactory
        {
          public:
            BundleSubspaceFactory() = default;
            BundleSubspacePtr MakeSubspaceBundle(base::StateSpacePtr BundleSpace, base::StateSpacePtr BaseSpace)
            {


            }
        }
    }
}
#endif
