#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_XRN_XRM__
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_XRN_XRM__
#include <ompl/geometric/planners/quotientspace/datastructures/BundleSpaceComponent.h>

namespace ompl
{
    namespace geometric
    {
        class BundleSpaceComponent_XRN_XRM: public BundleSpaceComponent
        {
            using BaseT = BundleSpaceComponent;
          public:
            BundleSpaceComponent_XRN_XRM(
                base::StateSpacePtr BundleSpace,
                base::StateSpacePtr BaseSpace);

            virtual ~BundleSpaceComponent_XRN_XRM() override = default;

            virtual void projectFiber(
                const ompl::base::State *xBundle,
                ompl::base::State *xFiber) const override;

            virtual void projectBase(
                const ompl::base::State *xBundle,
                ompl::base::State *xBase) const override = 0;

            virtual void mergeStates(
                const ompl::base::State *xBase, 
                const ompl::base::State *xFiber, 
                ompl::base::State *xBundle) const override = 0;

          protected:
            ompl::base::StateSpacePtr computeFiberSpace() override;
            unsigned int dimensionBaseFirstSubspace{0};


        };
    }
}

#endif

