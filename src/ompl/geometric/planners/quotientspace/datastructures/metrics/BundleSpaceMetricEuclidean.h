#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_METRICS_BUNDLEMETRIC_EUCLIDEAN__
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_METRICS_BUNDLEMETRIC_EUCLIDEAN__
#include <ompl/geometric/planners/quotientspace/datastructures/metrics/BundleSpaceMetric.h>

namespace ompl
{
  namespace geometric
  {
    class BundleSpaceMetricEuclidean: public BundleSpaceMetric
    {
        using BaseT = BundleSpaceMetric;
      public:
        BundleSpaceMetricEuclidean() = delete;
        BundleSpaceMetricEuclidean(BundleSpaceGraph*); 
        virtual ~BundleSpaceMetricEuclidean() override = default;

        virtual double distanceBundle(
            const Configuration *xStart, 
            const Configuration *xDest) override;
        virtual double distanceFiber(
            const Configuration *xStart, 
            const Configuration *xDest) override;
        virtual double distanceBase(
            const Configuration *xStart, 
            const Configuration *xDest) override;

    };
  }
}

#endif
