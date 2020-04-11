#include <ompl/geometric/planners/multilevel/datastructures/propagators/Geometric.h>

ompl::geometric::BundleSpacePropagatorGeometric::BundleSpacePropagatorGeometric(
    BundleSpaceGraph *bundleSpaceGraph):
  BaseT(bundleSpaceGraph)
{

}

ompl::geometric::BundleSpacePropagatorGeometric::~BundleSpacePropagatorGeometric()
{

}

bool ompl::geometric::BundleSpacePropagatorGeometric::steer( 
    const Configuration *from, 
    const Configuration *to, 
    Configuration *result) 
{
    bundleSpaceGraph_->interpolate(from, to, result);
    return bundleSpaceGraph_->checkMotion(from, result);
}
