#include <ompl/multilevel/datastructures/graphsampler/RandomVertex.h>

ompl::multilevel::BundleSpaceGraphSamplerRandomVertex::BundleSpaceGraphSamplerRandomVertex(
    BundleSpaceGraph *bundleSpaceGraph)
  : BaseT(bundleSpaceGraph)
{
}

void ompl::multilevel::BundleSpaceGraphSamplerRandomVertex::sampleImplementation(base::State *xRandom)
{
    const Vertex v = boost::random_vertex(bundleSpaceGraph_->getGraph(), rng_boost);
    bundleSpaceGraph_->getBundle()->getStateSpace()->copyState(xRandom, bundleSpaceGraph_->getGraph()[v]->state);
}
