#include <ompl/geometric/planners/quotientspace/datastructures/graphsampler/GraphSampler.h>

ompl::geometric::BundleSpaceGraphSampler::BundleSpaceGraphSampler(BundleSpaceGraph* bundleSpaceGraph):
  bundleSpaceGraph_(bundleSpaceGraph)
{
}

void ompl::geometric::BundleSpaceGraphSampler::sample(base::State *xRandom)
{
    // RANDOM VERTEX SAMPLING
    const Vertex v = boost::random_vertex(bundleSpaceGraph_->getGraph(), rng_boost);
    bundleSpaceGraph_->getBundle()->getStateSpace()->copyState(xRandom, bundleSpaceGraph_->getGraph()[v]->state);
}
