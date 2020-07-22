#include <ompl/multilevel/datastructures/importance/Exponential.h>

ompl::multilevel::BundleSpaceImportanceExponential::BundleSpaceImportanceExponential(BundleSpaceGraph *graph)
  : BaseT(graph)
{
}

double ompl::multilevel::BundleSpaceImportanceExponential::eval()
{
    const double p = 1.0 / bundleSpaceGraph_->getBundleDimension();
    double N = (double)bundleSpaceGraph_->getNumberOfVertices();
    double Nd = powf(N, p);
    return 1.0 / (Nd + 1.0);
}
