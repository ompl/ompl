#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_SUBSPACE_FACTORY__
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_SUBSPACE_FACTORY__
#include "BundleSpaceComponent.h"
#include "BundleSpaceComponentTypes.h"
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>

namespace ompl
{
    namespace geometric
    {
        OMPL_CLASS_FORWARD(BundleSpaceComponent);
    }
    namespace geometric
    {
        class BundleSpaceComponentFactory
        {
          public:
            BundleSpaceComponentFactory() = default;

            std::vector<BundleSpaceComponentPtr> MakeBundleSpaceComponents(
                base::SpaceInformationPtr Bundle, 
                base::SpaceInformationPtr Base);

            std::vector<BundleSpaceComponentPtr> MakeBundleSpaceComponents(
                base::SpaceInformationPtr Bundle);

          protected:
            BundleSpaceComponentPtr MakeBundleSpaceComponent(
                base::StateSpacePtr BundleSpace, 
                base::StateSpacePtr BaseSpace,
                bool);
            BundleSpaceComponentPtr MakeBundleSpaceComponent(
                base::StateSpacePtr BundleSpace);

            BundleSpaceComponentType
            identifyBundleSpaceComponentType(
                const base::StateSpacePtr BundleSpace, 
                const base::StateSpacePtr BaseSpace);

            bool isMapping_Identity(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_EmptyProjection(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_RN_to_RM(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_SE2_to_R2(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_SE2RN_to_R2(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_SE2RN_to_SE2(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_SE2RN_to_SE2RM(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_SE3_to_R3(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_SE3RN_to_R3(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_SE3RN_to_SE3(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_SE3RN_to_SE3RM(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_SO2RN_to_SO2(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_SO2RN_to_SO2RM(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_SO3RN_to_SO3(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_SO3RN_to_SO3RM(const base::StateSpacePtr, const base::StateSpacePtr);
            bool isMapping_RNSO2_to_RN(const base::StateSpacePtr, const base::StateSpacePtr);

            bool isMapping_XRN_to_XRM(
                const base::StateSpacePtr, 
                const base::StateSpacePtr,
                const base::StateSpaceType);
            bool isMapping_XRN_to_X(
                const base::StateSpacePtr, 
                const base::StateSpacePtr,
                const base::StateSpaceType);

            int GetNumberOfComponents(base::StateSpacePtr space);
        };
    }
}
#endif
