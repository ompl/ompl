#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_PROPAGATORS_GEOMETRIC__
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_PROPAGATORS_GEOMETRIC__
#include <ompl/geometric/planners/quotientspace/datastructures/propagators/BundleSpacePropagator.h>

namespace ompl
{

  namespace geometric
  {

    class BundleSpacePropagatorGeometric: public BundleSpacePropagator
    {
        using BaseT = BundleSpacePropagator;
      public:

        BundleSpacePropagatorGeometric() = delete;
        BundleSpacePropagatorGeometric(BundleSpaceGraph*); 

        virtual ~BundleSpacePropagatorGeometric() override;

        virtual bool steer( 
            const Configuration *from, 
            const Configuration *to, 
            Configuration *result) override;

    };
  }
}


#endif
