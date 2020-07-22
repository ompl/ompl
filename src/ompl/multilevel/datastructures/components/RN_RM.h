#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_RN_RM__
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_RN_RM__
#include <ompl/multilevel/datastructures/BundleSpaceComponent.h>

namespace ompl
{
    namespace multilevel
    {
        class BundleSpaceComponent_RN_RM : public BundleSpaceComponent
        {
            using BaseT = BundleSpaceComponent;

        public:
            BundleSpaceComponent_RN_RM(base::StateSpacePtr BundleSpace, base::StateSpacePtr BaseSpace);

            ~BundleSpaceComponent_RN_RM() override = default;

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
