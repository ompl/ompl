#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <iostream>
#include <sstream>
#include "ompl/base/PlannerData.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_PlannerData(nb::module_ &m)
{
    // PlannerDataVertex
    nb::class_<ob::PlannerDataVertex>(m, "PlannerDataVertex")
        .def(nb::init<const ob::State *, int>(), nb::arg("st"), nb::arg("tag") = 0)
        .def("getTag", &ob::PlannerDataVertex::getTag)
        .def("setTag", &ob::PlannerDataVertex::setTag, nb::arg("tag"))
        .def("getState", &ob::PlannerDataVertex::getState, nb::rv_policy::reference);

    // PlannerDataEdge
    nb::class_<ob::PlannerDataEdge>(m, "PlannerDataEdge").def(nb::init<>());

    // PlannerData
    nb::class_<ob::PlannerData>(m, "PlannerData")
        .def(nb::init<ob::SpaceInformationPtr>(), nb::arg("si"))
        // Vertex operations
        .def("addVertex", &ob::PlannerData::addVertex, nb::arg("st"))
        .def("addStartVertex", &ob::PlannerData::addStartVertex, nb::arg("v"))
        .def("addGoalVertex", &ob::PlannerData::addGoalVertex, nb::arg("v"))
        .def("removeVertex", nb::overload_cast<const ob::PlannerDataVertex &>(&ob::PlannerData::removeVertex),
             nb::arg("st"))
        .def("removeVertexByIndex", nb::overload_cast<unsigned int>(&ob::PlannerData::removeVertex), nb::arg("vIndex"))
        // Edge operations
        .def("addEdge",
             nb::overload_cast<unsigned int, unsigned int, const ob::PlannerDataEdge &, ob::Cost>(
                 &ob::PlannerData::addEdge),
             nb::arg("v1"), nb::arg("v2"), nb::arg("edge") = ob::PlannerDataEdge(), nb::arg("weight") = ob::Cost(1.0))
        .def("removeEdge", nb::overload_cast<unsigned int, unsigned int>(&ob::PlannerData::removeEdge), nb::arg("v1"),
             nb::arg("v2"))
        .def("clear", &ob::PlannerData::clear)
        .def("decoupleFromPlanner", &ob::PlannerData::decoupleFromPlanner)
        // Properties
        .def("numEdges", &ob::PlannerData::numEdges)
        .def("numVertices", &ob::PlannerData::numVertices)
        .def("numStartVertices", &ob::PlannerData::numStartVertices)
        .def("numGoalVertices", &ob::PlannerData::numGoalVertices)
        // Vertex lookup
        .def("vertexExists", &ob::PlannerData::vertexExists, nb::arg("v"))
        .def("getVertex", nb::overload_cast<unsigned int>(&ob::PlannerData::getVertex, nb::const_), nb::arg("index"),
             nb::rv_policy::reference)
        .def("getStartVertex", nb::overload_cast<unsigned int>(&ob::PlannerData::getStartVertex, nb::const_),
             nb::arg("i"), nb::rv_policy::reference)
        .def("getGoalVertex", nb::overload_cast<unsigned int>(&ob::PlannerData::getGoalVertex, nb::const_),
             nb::arg("i"), nb::rv_policy::reference)
        .def("getStartIndex", &ob::PlannerData::getStartIndex, nb::arg("i"))
        .def("getGoalIndex", &ob::PlannerData::getGoalIndex, nb::arg("i"))
        .def("isStartVertex", &ob::PlannerData::isStartVertex, nb::arg("index"))
        .def("isGoalVertex", &ob::PlannerData::isGoalVertex, nb::arg("index"))
        .def("vertexIndex", &ob::PlannerData::vertexIndex, nb::arg("v"))
        // Edge lookup
        .def("edgeExists", &ob::PlannerData::edgeExists, nb::arg("v1"), nb::arg("v2"))
        .def("getEdge", nb::overload_cast<unsigned int, unsigned int>(&ob::PlannerData::getEdge, nb::const_),
             nb::arg("v1"), nb::arg("v2"), nb::rv_policy::reference)
        .def(
            "getEdges",
            [](const ob::PlannerData &pd, unsigned int v)
            {
                std::vector<unsigned int> edgeList;
                pd.getEdges(v, edgeList);
                return edgeList;
            },
            nb::arg("v"))
        .def(
            "getIncomingEdges",
            [](const ob::PlannerData &pd, unsigned int v)
            {
                std::vector<unsigned int> edgeList;
                pd.getIncomingEdges(v, edgeList);
                return edgeList;
            },
            nb::arg("v"))
        .def("computeEdgeWeights", nb::overload_cast<>(&ob::PlannerData::computeEdgeWeights))
        // Output methods
        .def("printGraphviz",
             [](const ob::PlannerData &d)
             {
                 std::ostringstream oss;
                 d.printGraphviz(oss);
                 return oss.str();
             })
        .def("printGraphML",
             [](const ob::PlannerData &d)
             {
                 std::ostringstream oss;
                 d.printGraphML(oss);
                 return oss.str();
             })
        .def(
            "printPLY",
            [](const ob::PlannerData &d, bool asIs)
            {
                std::ostringstream oss;
                d.printPLY(oss, asIs);
                return oss.str();
            },
            nb::arg("asIs") = false)
        // Other
        .def("getSpaceInformation", &ob::PlannerData::getSpaceInformation)
        .def("hasControls", &ob::PlannerData::hasControls);
}
