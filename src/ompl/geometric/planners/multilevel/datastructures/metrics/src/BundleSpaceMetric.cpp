#include <ompl/geometric/planners/multilevel/datastructures/metrics/BundleSpaceMetric.h>

ompl::geometric::BundleSpaceMetric::BundleSpaceMetric(BundleSpaceGraph *bundleSpaceGraph)
  : bundleSpaceGraph_(bundleSpaceGraph)
{
    if (bundleSpaceGraph_->getFiberDimension() > 0)
    {
        xFiberStartTmp_ = bundleSpaceGraph_->getFiber()->allocState();
        xFiberDestTmp_ = bundleSpaceGraph_->getFiber()->allocState();
    }
    if (bundleSpaceGraph_->getBaseDimension() > 0)
    {
        xBaseStartTmp_ = bundleSpaceGraph_->getBase()->allocState();
        xBaseDestTmp_ = bundleSpaceGraph_->getBase()->allocState();
    }
}
ompl::geometric::BundleSpaceMetric::~BundleSpaceMetric()
{
    if (bundleSpaceGraph_->getFiberDimension() > 0)
    {
        bundleSpaceGraph_->getFiber()->freeState(xFiberStartTmp_);
        bundleSpaceGraph_->getFiber()->freeState(xFiberDestTmp_);
    }
    if (bundleSpaceGraph_->getBaseDimension() > 0)
    {
        bundleSpaceGraph_->getBase()->freeState(xBaseStartTmp_);
        bundleSpaceGraph_->getBase()->freeState(xBaseDestTmp_);
    }
}

void ompl::geometric::BundleSpaceMetric::reset()
{
}

void ompl::geometric::BundleSpaceMetric::interpolateBundle(const Configuration *q_from, const Configuration *q_to,
                                                           Configuration *q_interp)
{
    interpolateBundle(q_from, q_to, 1.0, q_interp);
}

void ompl::geometric::BundleSpaceMetric::interpolateBundle(const Configuration *q_from, Configuration *q_to)
{
    interpolateBundle(q_from, q_to, q_to);
}
