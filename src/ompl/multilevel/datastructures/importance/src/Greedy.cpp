#include <ompl/multilevel/datastructures/importance/Greedy.h>

ompl::multilevel::BundleSpaceImportanceGreedy::BundleSpaceImportanceGreedy(BundleSpaceGraph *graph) : BaseT(graph)
{
}

double ompl::multilevel::BundleSpaceImportanceGreedy::getLevelConstant()
{
    const double k = bundleSpaceGraph_->getLevel() + 1;
    BundleSpace *cur = bundleSpaceGraph_;

    double K = k;
    while (cur->hasSolution() && cur->hasChild())
    {
        cur = cur->getChild();
        K++;
    }

    double f = (k > 1 ? powf(epsilon, K - k) - powf(epsilon, K - k + 1) : powf(epsilon, K - k));

    return f;
}

double ompl::multilevel::BundleSpaceImportanceGreedy::eval()
{
    const double f = getLevelConstant();
    double N = (double)bundleSpaceGraph_->getNumberOfVertices();
    return 1.0 / (N / f + 1);
}
