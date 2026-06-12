#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include <utility>

#include <ompl/multilevel/datastructures/BundleSpaceGraph.h>
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace om = ompl::multilevel;

void ompl::binding::multilevel::init_BundleSpaceGraph(nb::module_ &m)
{
    nb::class_<om::BundleSpaceGraph, om::BundleSpace>(m, "BundleSpaceGraph")
        .def("getNumberOfVertices", &om::BundleSpaceGraph::getNumberOfVertices)
        .def("getNumberOfEdges", &om::BundleSpaceGraph::getNumberOfEdges)
        .def("sampleFromDatastructure",
             [](om::BundleSpaceGraph &self, ob::State *xRandom) { self.sampleFromDatastructure(xRandom); },
             nb::arg("xRandom"))
        .def("sampleBundleGoalBias",
             [](om::BundleSpaceGraph &self, ob::State *xRandom) { self.sampleBundleGoalBias(xRandom); },
             nb::arg("xRandom"))
        .def("getSolution",
             [](om::BundleSpaceGraph &self)
             {
                 ob::PathPtr solution;
                 const bool found = self.getSolution(solution);
                 return std::make_pair(found, solution);
             })
        .def("getImportance", &om::BundleSpaceGraph::getImportance)
        .def("init", &om::BundleSpaceGraph::init)
        .def("setup", &om::BundleSpaceGraph::setup)
        .def("clear", &om::BundleSpaceGraph::clear)
        .def("clearVertices", &om::BundleSpaceGraph::clearVertices)
        .def("setMetric", &om::BundleSpaceGraph::setMetric, nb::arg("metric"))
        .def("setPropagator", &om::BundleSpaceGraph::setPropagator, nb::arg("propagator"))
        .def("setImportance", &om::BundleSpaceGraph::setImportance, nb::arg("importance"))
        .def("setGraphSampler", &om::BundleSpaceGraph::setGraphSampler, nb::arg("graphSampler"))
        .def("setFindSectionStrategy", &om::BundleSpaceGraph::setFindSectionStrategy, nb::arg("type"))
        .def("getGraphSampler", &om::BundleSpaceGraph::getGraphSampler)
        .def("setGoalBias", &om::BundleSpaceGraph::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &om::BundleSpaceGraph::getGoalBias)
        .def("setRange", &om::BundleSpaceGraph::setRange, nb::arg("distance"))
        .def("getRange", &om::BundleSpaceGraph::getRange)
        .def("writeToGraphviz", &om::BundleSpaceGraph::writeToGraphviz, nb::arg("filename"));
}
