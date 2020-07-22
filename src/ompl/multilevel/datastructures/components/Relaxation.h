#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_RELAXATION__
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_RELAXATION__
#include <ompl/multilevel/datastructures/components/Identity.h>

namespace ompl
{
    namespace multilevel
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
