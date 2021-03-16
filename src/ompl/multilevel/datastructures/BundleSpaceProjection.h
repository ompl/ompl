/* Author: Andreas Orthey */

#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_PROJECTION_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_PROJECTION_
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/StateSpaceTypes.h>
#include "BundleSpaceComponentTypes.h"

namespace ompl
{
    namespace multilevel
    {
        class BundleSpaceProjection
        {
        public:
            BundleSpaceProjection(
                base::StateSpacePtr BundleSpace, 
                base::StateSpacePtr BaseSpace);

            virtual ~BundleSpaceProjection() = default;

            /* \brief All subclasses need to be able to project onto base space
             * */
            virtual void project(
                const ompl::base::State *xBundle, 
                ompl::base::State *xBase) const = 0;

            /* \brief All subclasses need to be able to lift from base space
             * into the total bundle space
             * */
            virtual void lift(
                const ompl::base::State *xBase, 
                ompl::base::State *xBundle) const = 0;

            /* \brief Test implementation by repreatedly calling project/lift
             * */
            virtual bool test();

            /// Dimension of Base Space
            unsigned int getBaseDimension() const;
            /// Dimension of Bundle Space
            unsigned int getDimension() const;

            /// Type of Bundle Space Projection
            BundleSpaceComponentType getType() const;
            void setType(BundleSpaceComponentType &);

            std::string getTypeAsString() const;
            std::string getBundleTypeAsString() const;
            std::string getBaseTypeAsString() const;

            friend std::ostream &operator<<(
                std::ostream &out, 
                const BundleSpaceProjection &);

        protected:
            virtual void print(std::ostream &out) const;
            std::string stateTypeToString(base::StateSpacePtr) const;

            base::StateSpacePtr BundleSpace_{nullptr};
            base::StateSpacePtr BaseSpace_{nullptr};

            bool isDynamic_{false};

            BundleSpaceComponentType type_;
        };

    }
}
#endif
