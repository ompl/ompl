#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_METRICS_BUNDLEMETRIC_GEODESIC__
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_METRICS_BUNDLEMETRIC_GEODESIC__
#include <ompl/multilevel/datastructures/metrics/BundleSpaceMetric.h>

namespace ompl
{
    namespace multilevel
    {
        class BundleSpaceMetricGeodesic : public BundleSpaceMetric
        {
            using BaseT = BundleSpaceMetric;

        public:
            BundleSpaceMetricGeodesic() = delete;
            BundleSpaceMetricGeodesic(BundleSpaceGraph *);
            virtual ~BundleSpaceMetricGeodesic() override = default;

            virtual double distanceBundle(const Configuration *xStart, const Configuration *xDest) override;
            virtual double distanceFiber(const Configuration *xStart, const Configuration *xDest) override;
            virtual double distanceBase(const Configuration *xStart, const Configuration *xDest) override;

            virtual void interpolateBundle(const Configuration *q_from, const Configuration *q_to, const double step,
                                           Configuration *q_interp) override;
        };
    }
}

#endif
