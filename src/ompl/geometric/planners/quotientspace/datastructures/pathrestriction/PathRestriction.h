#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_PATH_SECTION_
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_PATH_SECTION_
#include <ompl/geometric/planners/quotientspace/datastructures/BundleSpaceGraph.h>

namespace ompl
{
  
  namespace base
  {
      OMPL_CLASS_FORWARD(Path);
  }
  namespace geometric
  {
      OMPL_CLASS_FORWARD(BundleSpaceGraph);
      OMPL_CLASS_FORWARD(PathGeometric);

      class BundleSpacePathRestriction
      {
        public:
          using Configuration = ompl::geometric::BundleSpaceGraph::Configuration;
          BundleSpacePathRestriction() = delete;
          BundleSpacePathRestriction(BundleSpaceGraph*); 
          virtual ~BundleSpacePathRestriction();

          void setBasePath(base::PathPtr path);
          base::PathPtr getBasePath();

          std::vector<base::State*> interpolateManhattan(
                const base::State* xFiberStart,
                const base::State* xFiberGoal);

        protected:

          BundleSpaceGraph* bundleSpaceGraph_;

          base::PathPtr basePath_;
      };
  }
}

#endif
