#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_IDENTITY__
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_IDENTITY__
#include <ompl/multilevel/datastructures/BundleSpaceComponent.h>

namespace ompl
{
    namespace multilevel
    {
        class BundleSpaceComponent_Identity : public BundleSpaceComponent
        {
            using BaseT = BundleSpaceComponent;

        public:
            BundleSpaceComponent_Identity(base::StateSpacePtr BundleSpace, base::StateSpacePtr BaseSpace);

            ~BundleSpaceComponent_Identity() override = default;

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
