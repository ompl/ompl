#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_PROPAGATORS_
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_PROPAGATORS_
#include <ompl/geometric/planners/multilevel/datastructures/BundleSpaceGraph.h>

namespace ompl
{

  namespace geometric
  {

    OMPL_CLASS_FORWARD(BundleSpaceGraph);

    class BundleSpacePropagator
    {

      public:

        using Configuration = BundleSpaceGraph::Configuration;
        BundleSpacePropagator() = delete;
        BundleSpacePropagator(BundleSpaceGraph*); 

        virtual ~BundleSpacePropagator();

        virtual bool steer( 
            const Configuration *from, 
            const Configuration *to, 
            Configuration *result) = 0;

      protected:

        BundleSpaceGraph* bundleSpaceGraph_;

    };
  }
}


#endif
