#include <ompl/multilevel/datastructures/propagators/Dynamic.h>

ompl::multilevel::BundleSpacePropagatorDynamic::BundleSpacePropagatorDynamic(BundleSpaceGraph *bundleSpaceGraph)
  : BaseT(bundleSpaceGraph)
{
}

ompl::multilevel::BundleSpacePropagatorDynamic::~BundleSpacePropagatorDynamic()
{
}

bool ompl::multilevel::BundleSpacePropagatorDynamic::steer(const Configuration *from, const Configuration *to,
                                                           Configuration *result)
{
    bundleSpaceGraph_->interpolate(from, to, result);
    bool val = bundleSpaceGraph_->checkMotion(from, result);
    return val;
}
