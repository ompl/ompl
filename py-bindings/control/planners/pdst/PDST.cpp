#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include "ompl/control/planners/pdst/PDST.h"
#include "../../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersPdst_PDST(nb::module_ &m)
{
    nb::class_<oc::PDST, ob::Planner>(m, "PDST")
        .def(nb::init<const oc::SpaceInformationPtr &>(), nb::arg("si"))
        .def("setGoalBias", &oc::PDST::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &oc::PDST::getGoalBias)
        .def("setProjectionEvaluator",
             nb::overload_cast<const ob::ProjectionEvaluatorPtr &>(&oc::PDST::setProjectionEvaluator),
             nb::arg("projectionEvaluator"))
        .def("setProjectionEvaluator", nb::overload_cast<const std::string &>(&oc::PDST::setProjectionEvaluator),
             nb::arg("name"))
        .def("getProjectionEvaluator", &oc::PDST::getProjectionEvaluator);
}
