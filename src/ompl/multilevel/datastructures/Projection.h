/* Author: Andreas Orthey */

#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_PROJECTION_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_BUNDLE_PROJECTION_
#include <ompl/base/State.h>
#include <ompl/base/StateSpace.h>

namespace ompl
{
    namespace multilevel
    {
      OMPL_CLASS_FORWARD(ProjectionComponent);
      /* \brief A projection which consists of a set of component projections
       * */
        class Projection
        {
        public:
            Projection();
            Projection(std::vector<ProjectionComponentPtr> components);

            virtual ~Projection() = default;

            /* \brief All subclasses need to be able to project onto base space
             * */
            virtual void project(
                const ompl::base::State *xBundle, 
                ompl::base::State *xBase) const;

            /* \brief All subclasses need to be able to lift from base space
             * into the total bundle space
             * */
            virtual void lift(
                const ompl::base::State *xBase, 
                ompl::base::State *xBundle) const;

            unsigned int getCoDimension() const;
            unsigned int getDimension() const;

        protected:
            std::vector<ProjectionComponentPtr> components_;

        };
    }
}
#endif

