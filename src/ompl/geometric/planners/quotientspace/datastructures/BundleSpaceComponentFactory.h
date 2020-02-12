#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_SUBSPACE_FACTORY__
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_SUBSPACE_FACTORY__
#include "BundleSpaceComponent.h"
#include "BundleSpaceComponentTypes.h"
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>

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

            std::vector<BundleSpaceComponentPtr> MakeBundleSpaceComponents(
                base::SpaceInformationPtr Bundle, 
                base::SpaceInformationPtr Base);

            std::vector<BundleSpaceComponentPtr> MakeBundleSpaceComponents(
                base::SpaceInformationPtr Bundle);

          protected:
            BundleSpaceComponentPtr MakeBundleSpaceComponent(
                base::StateSpacePtr BundleSpace, 
                base::StateSpacePtr BaseSpace,
                bool);
            BundleSpaceComponentPtr MakeBundleSpaceComponent(
                base::StateSpacePtr BundleSpace);

            BundleSpaceComponentType
            identifyBundleSpaceComponentType(
                const base::StateSpacePtr BundleSpace, 
                const base::StateSpacePtr BaseSpace);

            int GetNumberOfComponents(base::StateSpacePtr space);
        };
    }
}
#endif
