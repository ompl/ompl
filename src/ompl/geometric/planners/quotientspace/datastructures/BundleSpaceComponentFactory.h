#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_SUBSPACE_FACTORY__
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_SUBSPACE_FACTORY__
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include "BundleSpaceComponent.h"
#include "BundleSpaceComponentTypes.h"

namespace ompl
{
    namespace geometric
    {
        class BundleSpaceComponentFactory
        {
          public:
            BundleSpaceComponentFactory() = default;

            BundleSpaceComponentPtr MakeBundleSpaceComponent(
                base::StateSpacePtr BundleSpace, 
                base::StateSpacePtr BaseSpace);

            BundleSpaceComponentType
            identifyBundleSpaceComponentType(
                const base::StateSpacePtr BundleSpace, 
                const base::StateSpacePtr BaseSpace);
        }
    }
}
#endif
