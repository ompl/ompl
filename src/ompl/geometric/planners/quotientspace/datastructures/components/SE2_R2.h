#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_SUBSPACE_SE2RN_R2__
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_SUBSPACE_SE2RN_R2__
#include "../BundleSpaceComponent.h"

namespace ompl
{
    namespace geometric
    {
        class BundleSpaceComponent_SE2_R2: public BundleSpaceComponent
        {
            using BaseT = BundleSpaceComponent;
          public:
            BundleSpaceComponent_SE2_R2(
                base::StateSpacePtr BundleSpace,
                base::StateSpacePtr BaseSpace);

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

            std::string getTypeAsString() override;
            std::string getFiberTypeAsString() override;
            std::string getBundleTypeAsString() override;
            std::string getBaseTypeAsString() override;
          protected:
            ompl::base::StateSpacePtr computeFiberSpace() override;


        };
    }
}

#endif

