#include <ompl/geometric/planners/quotientspace/datastructures/importance/Exponential.h>


ompl::geometric::BundleSpaceImportanceExponential::BundleSpaceImportanceExponential(BundleSpaceGraph* graph):
  BaseT(graph)
{
}

double ompl::geometric::BundleSpaceImportanceExponential::eval()
{
    //The higher the level, the more measure the space has. Therefore, we
    //require more samples to cover the space with similar density. Uses
    //inspiration from Multi-level monte carlo techniques
    const double p = 1.0/bundleSpaceGraph_->getBundleDimension();
    double N = (double)bundleSpaceGraph_->getNumberOfVertices();
    double Nd = powf(N, p);
    return 1.0 / (Nd + 1.0);
    // const double base = 2;
    // const double normalizer = powf(base, bundleSpaceGraph_->getLevel());
    // double N = (double)bundleSpaceGraph_->getNumberOfVertices()/normalizer;
    // return 1.0 / (N + 1.0);
}

