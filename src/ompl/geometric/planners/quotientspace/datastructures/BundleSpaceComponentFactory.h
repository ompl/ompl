#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_SUBSPACE_FACTORY__
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_SUBSPACE_FACTORY__
#include "BundleSpaceComponent.h"
#include "BundleSpaceComponentTypes.h"
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>

namespace ompl
{
    namespace geometric
    {
        OMPL_CLASS_FORWARD(BundleSpaceComponent);
    }
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
        };
    }
}
#endif
