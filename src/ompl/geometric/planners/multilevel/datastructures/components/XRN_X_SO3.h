#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_SO3RN_SO3__
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_SO3RN_SO3__
#include <ompl/geometric/planners/multilevel/datastructures/components/XRN_X.h>

namespace ompl
{
    namespace geometric
    {
        class BundleSpaceComponent_SO3RN_SO3: public BundleSpaceComponent_XRN_X
        {
            using BaseT = BundleSpaceComponent_XRN_X;
          public:
            BundleSpaceComponent_SO3RN_SO3(
                base::StateSpacePtr BundleSpace,
                base::StateSpacePtr BaseSpace);

            ~BundleSpaceComponent_SO3RN_SO3() override = default;

            virtual void projectBase(
                const ompl::base::State *xBundle,
                ompl::base::State *xBase) const override;

            virtual void liftState(
                const ompl::base::State *xBase, 
                const ompl::base::State *xFiber, 
                ompl::base::State *xBundle) const override;

        };
    }
}

#endif

