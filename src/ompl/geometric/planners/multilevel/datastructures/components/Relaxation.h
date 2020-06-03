#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_RELAXATION__
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_RELAXATION__
#include <ompl/geometric/planners/multilevel/datastructures/components/Identity.h>

namespace ompl
{
    namespace geometric
    {
        class BundleSpaceComponent_Relaxation : public BundleSpaceComponent_Identity
        {
            using BaseT = BundleSpaceComponent_Identity;

        public:
            BundleSpaceComponent_Relaxation(base::StateSpacePtr BundleSpace, base::StateSpacePtr BaseSpace);

            ~BundleSpaceComponent_Relaxation() override = default;
        };
    }
}

#endif
