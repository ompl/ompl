#include <ompl/geometric/planners/multilevel/datastructures/graphsampler/RandomEdge.h>

ompl::geometric::BundleSpaceGraphSamplerRandomEdge::BundleSpaceGraphSamplerRandomEdge(BundleSpaceGraph* bundleSpaceGraph):
  BaseT(bundleSpaceGraph)
{
}

void ompl::geometric::BundleSpaceGraphSamplerRandomEdge::sampleImplementation(base::State *xRandom)
{
    BundleSpaceGraph::Graph graph = bundleSpaceGraph_->getGraph();

    if (num_edges(graph) == 0) return;

    BundleSpaceGraph::Edge e = boost::random_edge(graph, rng_boost);

    double s = rng_.uniform01();

    const Vertex v1 = boost::source(e, graph);
    const Vertex v2 = boost::target(e, graph);
    const base::State *from = graph[v1]->state;
    const base::State *to = graph[v2]->state;

    bundleSpaceGraph_->getBundle()->getStateSpace()->interpolate(from, to, s, xRandom);
}
