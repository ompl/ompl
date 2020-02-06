#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_SUBSPACE_SE2RN_R2__
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_SUBSPACE_SE2RN_R2__
#include "../BundleSubspace.h"

namespace ompl
{
    namespace geometric
    {
        class BundleSubspaceSE2_R2: public BundleSubspace
        {
          public:
            BundleSubspace(
                base::StateSpacePtr BundleSpace,
                base::StateSpacePtr BaseSpace) = default;

            virtual void projectFiber(
                const ompl::base::State *xBundle,
                ompl::base::State *xFiber) const override;

            virtual void projectBase(
                const ompl::base::State *xBundle,
                ompl::base::State *xBase) const override;

            virtual void mergeStates(
                const ompl::base::State *xBase, 
                const ompl::base::State *xFiber, 
                ompl::base::State *xBundle) const override;

            ompl::base::StateSpacePtr getFiberSpace() override;

            std::string getTypeAsString() override;

        };
    }
}

#endif

