#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_SO3RN_SO3RM__
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_SO3RN_SO3RM__
#include <ompl/multilevel/datastructures/components/XRN_XRM.h>

namespace ompl
{
    namespace multilevel
    {
        class BundleSpaceComponent_SO3RN_SO3RM : public BundleSpaceComponent_XRN_XRM
        {
            using BaseT = BundleSpaceComponent_XRN_XRM;

        public:
            BundleSpaceComponent_SO3RN_SO3RM(base::StateSpacePtr BundleSpace, base::StateSpacePtr BaseSpace);

            ~BundleSpaceComponent_SO3RN_SO3RM() override = default;

            virtual void projectBase(const ompl::base::State *xBundle, ompl::base::State *xBase) const override;

            virtual void liftState(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                                   ompl::base::State *xBundle) const override;
        };
    }
}

#endif
