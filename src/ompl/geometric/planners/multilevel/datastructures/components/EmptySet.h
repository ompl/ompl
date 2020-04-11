#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_EMPTYSETPROJECTION___
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_EMPTYSETPROJECTION___
#include <ompl/geometric/planners/multilevel/datastructures/BundleSpaceComponent.h>

namespace ompl
{
    namespace geometric
    {
        /**
             @anchor EmptySet
             @par Short description
             Representation of a Fiber Bundle $X \rightarrow \emptyset$
        */
        class BundleSpaceComponent_EmptySet: public BundleSpaceComponent
        {
            using BaseT = BundleSpaceComponent;
          public:
            BundleSpaceComponent_EmptySet(
                base::StateSpacePtr BundleSpace,
                base::StateSpacePtr BaseSpace);

            ~BundleSpaceComponent_EmptySet() override = default;

            virtual void projectFiber(
                const ompl::base::State *xBundle,
                ompl::base::State *xFiber) const override;

            virtual void projectBase(
                const ompl::base::State *xBundle,
                ompl::base::State *xBase) const override;

            virtual void liftState(
                const ompl::base::State *xBase, 
                const ompl::base::State *xFiber, 
                ompl::base::State *xBundle) const override;

          protected:
            ompl::base::StateSpacePtr computeFiberSpace() override;


        };
    }
}

#endif

