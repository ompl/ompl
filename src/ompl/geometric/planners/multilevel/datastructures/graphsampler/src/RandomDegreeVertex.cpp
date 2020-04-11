#include <ompl/geometric/planners/multilevel/datastructures/graphsampler/RandomDegreeVertex.h>
#include <ompl/datastructures/PDF.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

ompl::geometric::BundleSpaceGraphSamplerRandomDegreeVertex::BundleSpaceGraphSamplerRandomDegreeVertex(BundleSpaceGraph* bundleSpaceGraph):
  BaseT(bundleSpaceGraph)
{
}

void ompl::geometric::BundleSpaceGraphSamplerRandomDegreeVertex::sampleImplementation(base::State *xRandom)
{
    const BundleSpaceGraph::Graph &graph = bundleSpaceGraph_->getGraph();

    ompl::PDF<BundleSpaceGraph::Configuration*> pdf;

    foreach (BundleSpaceGraph::Vertex v, boost::vertices(graph))
    {
        pdf.add(graph[v], boost::degree(v, graph));
    }

    if (pdf.empty())
        return;
    
    BundleSpaceGraph::Configuration *q = pdf.sample(rng_.uniform01());
    bundleSpaceGraph_->getBundle()->getStateSpace()->copyState(xRandom, q->state);
}
