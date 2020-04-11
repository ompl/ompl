#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_IMPORTANCE_IMPORTANCE_
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_IMPORTANCE_IMPORTANCE_
#include <ompl/geometric/planners/multilevel/datastructures/BundleSpaceGraph.h>
#include <memory>


namespace ompl
{
  namespace geometric
  {
    OMPL_CLASS_FORWARD(BundleSpaceGraph);


    class BundleSpaceImportance
    {
      public:

        using Configuration = BundleSpaceGraph::Configuration;
        BundleSpaceImportance() = delete;
        BundleSpaceImportance(BundleSpaceGraph* graph):
          bundleSpaceGraph_(graph){};

        virtual ~BundleSpaceImportance() = default;

        virtual double eval() = 0;

      protected:

        BundleSpaceGraph* bundleSpaceGraph_;
    };
  }
}


#endif

