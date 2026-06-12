#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/rrt/VFRRT.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_VFRRT(nb::module_ &m)
{
    nb::class_<og::VFRRT, og::RRT>(m, "VFRRT")
        .def(
            "__init__",
            [](og::VFRRT *self, const ob::SpaceInformationPtr &si, nb::callable vf, double exploration,
               double initial_lambda, unsigned int update_freq)
            {
                og::VFRRT::VectorField field =
                    [vf = std::move(vf)](const ob::State *state) mutable -> Eigen::VectorXd
                {
                    nb::gil_scoped_acquire gil;
                    return nb::cast<Eigen::VectorXd>(vf(state));
                };
                new (self) og::VFRRT(si, field, exploration, initial_lambda, update_freq);
            },
            nb::arg("si"), nb::arg("vf"), nb::arg("exploration"), nb::arg("initial_lambda"), nb::arg("update_freq"))
        .def("solve",
             [](og::VFRRT &self, nb::object what)
             {
                 if (nb::isinstance<ob::PlannerTerminationCondition>(what))
                 {
                     return self.solve(nb::cast<ob::PlannerTerminationCondition>(what));
                 }
                 else if (nb::isinstance<double>(what))
                 {
                     return self.solve(ob::timedPlannerTerminationCondition(nb::cast<double>(what)));
                 }
                 else
                 {
                     throw nb::type_error(
                         "Invalid argument type for solve. Expected PlannerTerminationCondition or double.");
                 }
             })
        .def(
            "getPlannerData", [](const og::VFRRT &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::VFRRT::clear)
        .def("setup", &og::VFRRT::setup)
        .def("determineMeanNorm", &og::VFRRT::determineMeanNorm);
}
