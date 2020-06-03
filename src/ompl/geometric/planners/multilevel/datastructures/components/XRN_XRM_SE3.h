#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_SE3RN_SE3RM__
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_COMPONENT_SE3RN_SE3RM__
#include <ompl/geometric/planners/multilevel/datastructures/components/XRN_XRM.h>

namespace ompl
{
    namespace geometric
    {
        class BundleSpaceComponent_SE3RN_SE3RM : public BundleSpaceComponent_XRN_XRM
        {
            using BaseT = BundleSpaceComponent_XRN_XRM;

        public:
            BundleSpaceComponent_SE3RN_SE3RM(base::StateSpacePtr BundleSpace, base::StateSpacePtr BaseSpace);

            ~BundleSpaceComponent_SE3RN_SE3RM() override = default;

            virtual void projectBase(const ompl::base::State *xBundle, ompl::base::State *xBase) const override;

            virtual void liftState(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                                   ompl::base::State *xBundle) const override;
        };
    }
}

#endif
