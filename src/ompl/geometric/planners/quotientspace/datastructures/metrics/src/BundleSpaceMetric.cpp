#include <ompl/geometric/planners/quotientspace/datastructures/metrics/BundleSpaceMetric.h>

ompl::geometric::BundleSpaceMetric::BundleSpaceMetric(BundleSpaceGraph* bundleSpaceGraph):
  bundleSpaceGraph_(bundleSpaceGraph)
{
    if(bundleSpaceGraph_->getFiberDimension() > 0)
    {
        xFiberStartTmp_ = bundleSpaceGraph_->getFiber()->allocState();
        xFiberDestTmp_ = bundleSpaceGraph_->getFiber()->allocState();
    }
    if(bundleSpaceGraph_->getBaseDimension() > 0)
    {
        xBaseStartTmp_ = bundleSpaceGraph_->getBase()->allocState();
        xBaseDestTmp_ = bundleSpaceGraph_->getBase()->allocState();
    }
}
ompl::geometric::BundleSpaceMetric::~BundleSpaceMetric()
{
    if(bundleSpaceGraph_->getFiberDimension() > 0)
    {
        bundleSpaceGraph_->getFiber()->freeState(xFiberStartTmp_);
        bundleSpaceGraph_->getFiber()->freeState(xFiberDestTmp_);
    }
    if(bundleSpaceGraph_->getBaseDimension() > 0)
    {
        bundleSpaceGraph_->getBase()->freeState(xBaseStartTmp_);
        bundleSpaceGraph_->getBase()->freeState(xBaseDestTmp_);
    }
}
