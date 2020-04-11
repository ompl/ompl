#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_SUBSPACE_
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_BUNDLE_SUBSPACE_
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/StateSpaceTypes.h>
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

            virtual ~BundleSpaceComponent() = default;

            virtual void projectFiber(
                const ompl::base::State *xBundle,
                ompl::base::State *xFiber) const = 0;

            virtual void projectBase(
                const ompl::base::State *xBundle,
                ompl::base::State *xBase) const = 0;

            virtual void liftState(
                const ompl::base::State *xBase, 
                const ompl::base::State *xFiber, 
                ompl::base::State *xBundle) const = 0;

            ompl::base::StateSpacePtr getFiberSpace() const;

            void initFiberSpace();

            bool isDynamic() const;

            /// Dimension of Fiber Space
            unsigned int getFiberDimension() const;
            /// Dimension of Base Space
            unsigned int getBaseDimension() const;
            /// Dimension of Bundle Space
            unsigned int getDimension() const;
            /// Type of Bundle Space
            BundleSpaceComponentType getType() const;
            void setType(BundleSpaceComponentType&);

            std::string getTypeAsString() const;
            std::string getFiberTypeAsString() const;
            std::string getBundleTypeAsString() const;
            std::string getBaseTypeAsString() const;

            friend std::ostream &operator<<(std::ostream &out, const BundleSpaceComponent&);

          protected:
            virtual ompl::base::StateSpacePtr computeFiberSpace() = 0;
            virtual void print(std::ostream &out) const;
            std::string stateTypeToString(base::StateSpacePtr) const;

            base::StateSpacePtr BundleSpace_{nullptr};
            base::StateSpacePtr BaseSpace_{nullptr};
            base::StateSpacePtr FiberSpace_{nullptr};

            bool isDynamic_{false};

            BundleSpaceComponentType type_;
        };
    }
}

#endif
