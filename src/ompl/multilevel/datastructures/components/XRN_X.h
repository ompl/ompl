#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_XRN_X__
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_XRN_X__
#include <ompl/multilevel/datastructures/BundleSpaceComponent.h>

namespace ompl
{
    namespace multilevel
    {
        class BundleSpaceComponent_XRN_X : public BundleSpaceComponent
        {
            using BaseT = BundleSpaceComponent;

        public:
            BundleSpaceComponent_XRN_X(base::StateSpacePtr BundleSpace, base::StateSpacePtr BaseSpace);

            virtual ~BundleSpaceComponent_XRN_X() override = default;

            virtual void projectFiber(const ompl::base::State *xBundle, ompl::base::State *xFiber) const override;

            virtual void projectBase(const ompl::base::State *xBundle, ompl::base::State *xBase) const = 0;

            virtual void liftState(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                                   ompl::base::State *xBundle) const = 0;

        protected:
            ompl::base::StateSpacePtr computeFiberSpace() override;
        };
    }
}

#endif
