#include <ompl/multilevel/datastructures/importance/Uniform.h>

ompl::multilevel::BundleSpaceImportanceUniform::BundleSpaceImportanceUniform(BundleSpaceGraph *graph) : BaseT(graph)
{
}

double ompl::multilevel::BundleSpaceImportanceUniform::eval()
{
    double N = (double)bundleSpaceGraph_->getNumberOfVertices();
    return 1.0 / (N + 1);
}
