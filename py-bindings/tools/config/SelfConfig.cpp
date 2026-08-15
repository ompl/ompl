#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <sstream>

#include "ompl/tools/config/SelfConfig.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace ot = ompl::tools;

void ompl::binding::tools::initConfig_SelfConfig(nb::module_ &m)
{
    nb::class_<ot::SelfConfig>(m, "SelfConfig")
        .def(nb::init<const ob::SpaceInformationPtr &, const std::string &>(), nb::arg("si"),
             nb::arg("context") = std::string())
        .def("getProbabilityOfValidState", &ot::SelfConfig::getProbabilityOfValidState)
        .def("getAverageValidMotionLength", &ot::SelfConfig::getAverageValidMotionLength)
        .def("configureValidStateSamplingAttempts", &ot::SelfConfig::configureValidStateSamplingAttempts,
             nb::arg("attempts"))
        .def("configurePlannerRange", &ot::SelfConfig::configurePlannerRange, nb::arg("range"))
        .def("configureProjectionEvaluator", &ot::SelfConfig::configureProjectionEvaluator, nb::arg("proj"))
        .def("__repr__",
             [](const ot::SelfConfig &sc)
             {
                 std::ostringstream oss;
                 sc.print(oss);
                 return oss.str();
             })
        .def_static("getDefaultPlanner", &ot::SelfConfig::getDefaultPlanner, nb::arg("goal"));
}
