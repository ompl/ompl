#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_SE3RN_R3__
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_SE3RN_R3__
#include <ompl/multilevel/datastructures/BundleSpaceComponent.h>

namespace ompl
{
    namespace multilevel
    {
        class BundleSpaceComponent_SE3RN_R3 : public BundleSpaceComponent
        {
            using BaseT = BundleSpaceComponent;

        public:
            BundleSpaceComponent_SE3RN_R3(base::StateSpacePtr BundleSpace, base::StateSpacePtr BaseSpace);

            ~BundleSpaceComponent_SE3RN_R3() override = default;

            virtual void projectFiber(const ompl::base::State *xBundle, ompl::base::State *xFiber) const override;

            virtual void projectBase(const ompl::base::State *xBundle, ompl::base::State *xBase) const override;

            virtual void liftState(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                                   ompl::base::State *xBundle) const override;

        protected:
            ompl::base::StateSpacePtr computeFiberSpace() override;
        };
    }
}

#endif
