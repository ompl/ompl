#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_METRICS_BUNDLEMETRIC_
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_METRICS_BUNDLEMETRIC_
#include <ompl/geometric/planners/quotientspace/datastructures/BundleSpaceGraph.h>

namespace ompl
{
  namespace geometric
  {
    OMPL_CLASS_FORWARD(BundleSpaceGraph);

    class BundleSpaceMetric
    {
      public:
        using Configuration = ompl::geometric::BundleSpaceGraph::Configuration;
        BundleSpaceMetric() = delete;
        BundleSpaceMetric(BundleSpaceGraph*); 

        virtual ~BundleSpaceMetric();

        //############################################################################
        //Distance Functions
        //############################################################################

        virtual double distanceBundle(
            const Configuration *xStart, 
            const Configuration *xDest) = 0;
        virtual double distanceFiber(
            const Configuration *xStart, 
            const Configuration *xDest) = 0;
        virtual double distanceBase(
            const Configuration *xStart, 
            const Configuration *xDest) = 0;

        //############################################################################
        //Interpolate Functions
        //############################################################################

        virtual void interpolateBundle(const Configuration *q_from, const Configuration *q_to, const double step, Configuration* q_interp) = 0;

        void interpolateBundle(const Configuration *q_from, const Configuration *q_to, Configuration* q_interp);
        void interpolateBundle(const Configuration *q_from, Configuration *q_to);

      protected:

        BundleSpaceGraph* bundleSpaceGraph_;

        base::State *xFiberStartTmp_;
        base::State *xFiberDestTmp_;
        base::State *xBaseStartTmp_;
        base::State *xBaseDestTmp_;
    };
  }
}


#endif
