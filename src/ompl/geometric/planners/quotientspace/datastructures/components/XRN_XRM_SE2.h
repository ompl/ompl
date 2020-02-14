#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_SE2RN_SE2RM__
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_SE2RN_SE2RM__
#include <ompl/geometric/planners/quotientspace/datastructures/components/XRN_XRM.h>

namespace ompl
{
    namespace geometric
    {
        class BundleSpaceComponent_SE2RN_SE2RM: public BundleSpaceComponent_XRN_XRM
        {
            using BaseT = BundleSpaceComponent_XRN_XRM;
          public:
            BundleSpaceComponent_SE2RN_SE2RM(
                base::StateSpacePtr BundleSpace,
                base::StateSpacePtr BaseSpace);

            ~BundleSpaceComponent_SE2RN_SE2RM() override = default;

            virtual void projectBase(
                const ompl::base::State *xBundle,
                ompl::base::State *xBase) const override;

            virtual void mergeStates(
                const ompl::base::State *xBase, 
                const ompl::base::State *xFiber, 
                ompl::base::State *xBundle) const override;

        };
    }
}

#endif

