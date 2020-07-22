#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_SE3RN_SE3__
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_SE3RN_SE3__
#include <ompl/multilevel/datastructures/components/XRN_X.h>

namespace ompl
{
    namespace multilevel
    {
        class BundleSpaceComponent_SE3RN_SE3 : public BundleSpaceComponent_XRN_X
        {
            using BaseT = BundleSpaceComponent_XRN_X;

        public:
            BundleSpaceComponent_SE3RN_SE3(base::StateSpacePtr BundleSpace, base::StateSpacePtr BaseSpace);

            ~BundleSpaceComponent_SE3RN_SE3() override = default;

            virtual void projectBase(const ompl::base::State *xBundle, ompl::base::State *xBase) const override;

            virtual void liftState(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                                   ompl::base::State *xBundle) const override;
        };
    }
}

#endif
