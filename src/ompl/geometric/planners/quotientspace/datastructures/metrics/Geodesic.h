#ifndef OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_METRICS_BUNDLEMETRIC_GEODESIC__
#define OMPL_GEOMETRIC_PLANNERS_BUNDLESPACE_METRICS_BUNDLEMETRIC_GEODESIC__
#include <ompl/geometric/planners/quotientspace/datastructures/metrics/BundleSpaceMetric.h>

namespace ompl
{
  namespace geometric
  {
    class BundleSpaceMetricGeodesic: public BundleSpaceMetric
    {
        using BaseT = BundleSpaceMetric;
      public:
        BundleSpaceMetricGeodesic() = delete;
        BundleSpaceMetricGeodesic(BundleSpaceGraph*); 
        virtual ~BundleSpaceMetricGeodesic() override = default;

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
