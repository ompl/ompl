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
    bool val = bundleSpaceGraph_->checkMotion(from, result);

    // std::cout << std::string(80, '-') << std::endl;
    // OMPL_WARN("Interpolate");
    // bundleSpaceGraph_->printConfiguration(from);
    // bundleSpaceGraph_->printConfiguration(to);
    // bundleSpaceGraph_->printConfiguration(result);
    // std::cout << std::string(80, '-') << std::endl;
    
    // std::cout << (val?"VALID":"INVALID") << std::endl;
    return val;
}
