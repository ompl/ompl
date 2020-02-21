#include "../BundleSpacePropagatorGeometric.h"

ompl::geometric::BundleSpacePropagatorGeometric::BundleSpacePropagatorGeometric(
    BundleSpaceGraph *bundleSpaceGraph):
  BaseT(bundleSpaceGraph)
{

}

ompl::geometric::BundleSpacePropagatorGeometric::~BundleSpacePropagatorGeometric()
{

}

bool ompl::geometric::BundleSpacePropagatorGeometric::propagate( 
    const Configuration *from, 
    const Configuration *to, 
    Configuration *result) const
{
    bundleSpaceGraph_->interpolate(from, to, result);
    return bundleSpaceGraph_->checkMotion(from, result);
}

