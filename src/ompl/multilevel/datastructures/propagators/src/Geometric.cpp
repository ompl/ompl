#include <ompl/multilevel/datastructures/propagators/Geometric.h>

ompl::multilevel::BundleSpacePropagatorGeometric::BundleSpacePropagatorGeometric(BundleSpaceGraph *bundleSpaceGraph)
  : BaseT(bundleSpaceGraph)
{
}

ompl::multilevel::BundleSpacePropagatorGeometric::~BundleSpacePropagatorGeometric()
{
}

bool ompl::multilevel::BundleSpacePropagatorGeometric::steer(const Configuration *from, const Configuration *to,
                                                             Configuration *result)
{
    bundleSpaceGraph_->interpolate(from, to, result);
    bool val = bundleSpaceGraph_->checkMotion(from, result);
    return val;
}
