#include <ompl/multilevel/datastructures/metrics/Geodesic.h>
using namespace ompl::multilevel;
using Configuration = ompl::multilevel::BundleSpaceGraph::Configuration;

BundleSpaceMetricGeodesic::BundleSpaceMetricGeodesic(BundleSpaceGraph *bundleSpaceGraph) : BaseT(bundleSpaceGraph)
{
}

double BundleSpaceMetricGeodesic::distanceBundle(const Configuration *xStart, const Configuration *xDest)
{
    return bundleSpaceGraph_->getBundle()->distance(xStart->state, xDest->state);
}

double BundleSpaceMetricGeodesic::distanceFiber(const Configuration *xStart, const Configuration *xDest)
{
    if (bundleSpaceGraph_->getFiberDimension() > 0)
    {
        bundleSpaceGraph_->projectFiber(xStart->state, xFiberStartTmp_);
        bundleSpaceGraph_->projectFiber(xDest->state, xFiberDestTmp_);
        double d = bundleSpaceGraph_->getFiber()->distance(xFiberStartTmp_, xFiberDestTmp_);
        return d;
    }
    else
    {
        return 0.0;
    }
}

double BundleSpaceMetricGeodesic::distanceBase(const Configuration *xStart, const Configuration *xDest)
{
    if (bundleSpaceGraph_->getBaseDimension() > 0)
    {
        bundleSpaceGraph_->projectBase(xStart->state, xBaseStartTmp_);
        bundleSpaceGraph_->projectBase(xDest->state, xBaseDestTmp_);
        double d = bundleSpaceGraph_->getBase()->distance(xBaseStartTmp_, xBaseDestTmp_);
        return d;
    }
    else
    {
        return 0.0;
    }
}

void BundleSpaceMetricGeodesic::interpolateBundle(const Configuration *q_from, const Configuration *q_to,
                                                  const double step, Configuration *q_interp)
{
    bundleSpaceGraph_->getBundle()->getStateSpace()->interpolate(q_from->state, q_to->state, step, q_interp->state);
}
