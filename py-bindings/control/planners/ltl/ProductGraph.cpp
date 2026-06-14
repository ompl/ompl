#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/control/planners/ltl/ProductGraph.h"
#include "../../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersLtl_ProductGraph(nb::module_ &m)
{
    nb::class_<oc::ProductGraph::State>(m, "ProductGraphState")
        .def(nb::init<>())
        .def("isValid", &oc::ProductGraph::State::isValid)
        .def("getDecompRegion", &oc::ProductGraph::State::getDecompRegion)
        .def("getCosafeState", &oc::ProductGraph::State::getCosafeState)
        .def("getSafeState", &oc::ProductGraph::State::getSafeState);

    nb::class_<oc::ProductGraph>(m, "ProductGraph")
        .def(nb::init<const oc::PropositionalDecompositionPtr &, oc::AutomatonPtr, oc::AutomatonPtr>(),
             nb::arg("decomp"), nb::arg("cosafetyAut"), nb::arg("safetyAut"))
        .def(nb::init<const oc::PropositionalDecompositionPtr &, oc::AutomatonPtr>(), nb::arg("decomp"),
             nb::arg("cosafetyAut"))
        .def("getDecomp", &oc::ProductGraph::getDecomp, nb::rv_policy::reference_internal)
        .def("getCosafetyAutom", &oc::ProductGraph::getCosafetyAutom, nb::rv_policy::reference_internal)
        .def("getSafetyAutom", &oc::ProductGraph::getSafetyAutom, nb::rv_policy::reference_internal)
        .def("clear", &oc::ProductGraph::clear)
        .def("isSolution", &oc::ProductGraph::isSolution, nb::arg("s"))
        .def("getStartState", &oc::ProductGraph::getStartState, nb::rv_policy::reference_internal)
        .def("getRegionVolume", &oc::ProductGraph::getRegionVolume, nb::arg("s"))
        .def("getCosafeAutDistance", &oc::ProductGraph::getCosafeAutDistance, nb::arg("s"))
        .def("getSafeAutDistance", &oc::ProductGraph::getSafeAutDistance, nb::arg("s"))
        .def("getState", nb::overload_cast<const ob::State *>(&oc::ProductGraph::getState, nb::const_), nb::arg("cs"),
             nb::rv_policy::reference_internal)
        .def("getState", nb::overload_cast<const ob::State *, int, int>(&oc::ProductGraph::getState, nb::const_),
             nb::arg("cs"), nb::arg("cosafe"), nb::arg("safe"), nb::rv_policy::reference_internal)
        .def("getState",
             nb::overload_cast<const oc::ProductGraph::State *, int>(&oc::ProductGraph::getState, nb::const_),
             nb::arg("parent"), nb::arg("nextRegion"), nb::rv_policy::reference_internal)
        .def("getState",
             nb::overload_cast<const oc::ProductGraph::State *, const ob::State *>(&oc::ProductGraph::getState,
                                                                                   nb::const_),
             nb::arg("parent"), nb::arg("cs"), nb::rv_policy::reference_internal)
        .def("getState", nb::overload_cast<int, int, int>(&oc::ProductGraph::getState, nb::const_), nb::arg("region"),
             nb::arg("cosafe"), nb::arg("safe"), nb::rv_policy::reference_internal)
        .def(
            "buildGraph",
            [](oc::ProductGraph &pg, oc::ProductGraph::State *start, nb::object initialize)
            {
                if (initialize.is_none())
                    pg.buildGraph(start);
                else
                    pg.buildGraph(start, nb::cast<std::function<void(oc::ProductGraph::State *)>>(initialize));
            },
            nb::arg("start"), nb::arg("initialize") = nb::none());
}
