#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_SUBSPACE_
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_SUBSPACE_
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include "BundleSpaceComponent.h"
#include "BundleSpaceComponentTypes.h"

namespace ompl
{
    namespace geometric
    {
        class BundleSpaceComponent
        {
          public:
            BundleSpaceComponent(
                base::StateSpacePtr BundleSpace,
                base::StateSpacePtr BaseSpace);

            virtual void projectFiber(
                const ompl::base::State *xBundle,
                ompl::base::State *xFiber) const = 0;

            virtual void projectBase(
                const ompl::base::State *xBundle,
                ompl::base::State *xBase) const = 0;

            virtual void mergeStates(
                const ompl::base::State *xBase, 
                const ompl::base::State *xFiber, 
                ompl::base::State *xBundle) const = 0;

            ompl::base::StateSpacePtr getFiberSpace() const;

            void initFiberSpace();

            /// Dimension of Fiber Space
            unsigned int getFiberDimension() const;
            /// Dimension of Base Space
            unsigned int getBaseDimension() const;
            /// Dimension of Bundle Space
            unsigned int getDimension() const;
            /// Type of Bundle Space
            BundleSpaceComponentType getType() const;

            virtual std::string getTypeAsString() = 0;
            virtual std::string getFiberTypeAsString() = 0;
            virtual std::string getBundleTypeAsString() = 0;
            virtual std::string getBaseTypeAsString() = 0;

          protected:
            virtual ompl::base::StateSpacePtr computeFiberSpace() = 0;

            base::StateSpacePtr BundleSpace_{nullptr};
            base::StateSpacePtr BaseSpace_{nullptr};
            base::StateSpacePtr FiberSpace_{nullptr};

            BundleSpaceComponentType type_;
        };
    }
}

#endif
