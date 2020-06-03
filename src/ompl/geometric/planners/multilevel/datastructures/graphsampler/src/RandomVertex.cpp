#include <ompl/geometric/planners/multilevel/datastructures/graphsampler/RandomVertex.h>

ompl::geometric::BundleSpaceGraphSamplerRandomVertex::BundleSpaceGraphSamplerRandomVertex(
    BundleSpaceGraph *bundleSpaceGraph)
  : BaseT(bundleSpaceGraph)
{
}

void ompl::geometric::BundleSpaceGraphSamplerRandomVertex::sampleImplementation(base::State *xRandom)
{
    const Vertex v = boost::random_vertex(bundleSpaceGraph_->getGraph(), rng_boost);
    bundleSpaceGraph_->getBundle()->getStateSpace()->copyState(xRandom, bundleSpaceGraph_->getGraph()[v]->state);
}
