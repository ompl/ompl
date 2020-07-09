#include <ompl/geometric/planners/multilevel/datastructures/propagators/Dynamic.h>

ompl::geometric::BundleSpacePropagatorDynamic::BundleSpacePropagatorDynamic(BundleSpaceGraph *bundleSpaceGraph)
  : BaseT(bundleSpaceGraph)
{
}

ompl::geometric::BundleSpacePropagatorDynamic::~BundleSpacePropagatorDynamic()
{
}

bool ompl::geometric::BundleSpacePropagatorDynamic::steer(const Configuration *from, const Configuration *to,
                                                            Configuration *result)
{
    bundleSpaceGraph_->interpolate(from, to, result);
    bool val = bundleSpaceGraph_->checkMotion(from, result);
    return val;
}
