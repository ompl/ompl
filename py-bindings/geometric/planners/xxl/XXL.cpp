#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/geometric/planners/xxl/XXL.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersXxl_XXL(nb::module_ &m)
{
    nb::class_<og::XXL, ob::Planner>(m, "XXL")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def(nb::init<const ob::SpaceInformationPtr &, const og::XXLDecompositionPtr &>(), nb::arg("si"),
             nb::arg("decomp"))
        .def(
            "solve",
            [](og::XXL &self, nb::object what)
            {
                if (nb::isinstance<ob::PlannerTerminationCondition>(what))
                    return self.solve(nb::cast<ob::PlannerTerminationCondition>(what));
                if (nb::isinstance<double>(what))
                    return self.solve(ob::timedPlannerTerminationCondition(nb::cast<double>(what)));
                throw nb::type_error(
                    "Invalid argument type for solve. Expected PlannerTerminationCondition or double.");
            })
        .def(
            "getPlannerData", [](const og::XXL &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::XXL::clear)
        .def("setup", &og::XXL::setup)
        .def("setDecomposition", &og::XXL::setDecomposition, nb::arg("decomp"))
        .def("getRandWalkRate", &og::XXL::getRandWalkRate)
        .def("setRandWalkRate", &og::XXL::setRandWalkRate, nb::arg("rate"));
}
