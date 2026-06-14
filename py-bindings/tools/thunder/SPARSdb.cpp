#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/tools/thunder/SPARSdb.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::tools::initThunder_SPARSdb(nb::module_ &m)
{
    nb::enum_<og::SPARSdb::GuardType>(m, "SPARSdbGuardType")
        .value("START", og::SPARSdb::START)
        .value("GOAL", og::SPARSdb::GOAL)
        .value("COVERAGE", og::SPARSdb::COVERAGE)
        .value("CONNECTIVITY", og::SPARSdb::CONNECTIVITY)
        .value("INTERFACE", og::SPARSdb::INTERFACE)
        .value("QUALITY", og::SPARSdb::QUALITY)
        .export_values();

    nb::class_<og::SPARSdb, ob::Planner>(m, "SPARSdb")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve",
             [](og::SPARSdb &self, nb::object what)
             {
                 if (nb::isinstance<ob::PlannerTerminationCondition>(what))
                     return self.solve(nb::cast<ob::PlannerTerminationCondition>(what));
                 if (nb::isinstance<double>(what))
                     return self.solve(ob::timedPlannerTerminationCondition(nb::cast<double>(what)));
                 throw nb::type_error(
                     "Invalid argument type for solve. Expected PlannerTerminationCondition or double.");
             })
        .def(
            "getPlannerData", [](const og::SPARSdb &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::SPARSdb::clear)
        .def("clearQuery", &og::SPARSdb::clearQuery)
        .def("setup", &og::SPARSdb::setup)
        .def("setStretchFactor", &og::SPARSdb::setStretchFactor, nb::arg("t"))
        .def("getStretchFactor", &og::SPARSdb::getStretchFactor)
        .def("setSparseDeltaFraction", &og::SPARSdb::setSparseDeltaFraction, nb::arg("D"))
        .def("getSparseDeltaFraction", &og::SPARSdb::getSparseDeltaFraction)
        .def("setDenseDeltaFraction", &og::SPARSdb::setDenseDeltaFraction, nb::arg("d"))
        .def("getDenseDeltaFraction", &og::SPARSdb::getDenseDeltaFraction)
        .def("setMaxFailures", &og::SPARSdb::setMaxFailures, nb::arg("m"))
        .def("getMaxFailures", &og::SPARSdb::getMaxFailures)
        .def("getNumVertices", &og::SPARSdb::getNumVertices)
        .def("getNumEdges", &og::SPARSdb::getNumEdges);
}
