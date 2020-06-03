#include <ompl/geometric/planners/multilevel/datastructures/importance/Uniform.h>

ompl::geometric::BundleSpaceImportanceUniform::BundleSpaceImportanceUniform(BundleSpaceGraph *graph) : BaseT(graph)
{
}

double ompl::geometric::BundleSpaceImportanceUniform::eval()
{
    double N = (double)bundleSpaceGraph_->getNumberOfVertices();
    return 1.0 / (N + 1);
}
