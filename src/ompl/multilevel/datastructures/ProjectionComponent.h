/* Author: Andreas Orthey */

#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_PROJECTION_COMPONENT_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_PROJECTION_COMPONENT_
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/StateSpaceTypes.h>
#include "ProjectionComponentTypes.h"

namespace ompl
{
    namespace multilevel
    {
        class ProjectionComponent
        {
        public:
            ProjectionComponent(
                base::StateSpacePtr BundleSpace, 
                base::StateSpacePtr BaseSpace);

            virtual ~ProjectionComponent() = default;

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

            /// Dimension of Base Space
            unsigned int getBaseDimension() const;
            /// Dimension of Bundle Space
            unsigned int getDimension() const;
            /// Dimension of Bundle - Dimension of Base
            unsigned int getCoDimension() const;

            /// \brief Get bundle space
            base::StateSpacePtr getBundleSpace() const;
            /// \brief Get base space
            base::StateSpacePtr getBaseSpace() const;


            /// Type of Bundle Space Projection
            ProjectionComponentType getType() const;
            void setType(ProjectionComponentType &);

            std::string getTypeAsString() const;
            std::string getBundleTypeAsString() const;
            std::string getBaseTypeAsString() const;

            friend std::ostream &operator<<(
                std::ostream &out, 
                const ProjectionComponent &);
            virtual void print(std::ostream &out) const;
            std::string stateTypeToString(base::StateSpacePtr) const;
        private:


            base::StateSpacePtr bundleSpace_{nullptr};
            base::StateSpacePtr baseSpace_{nullptr};

            ProjectionComponentType type_;
        };

    }
}
#endif
