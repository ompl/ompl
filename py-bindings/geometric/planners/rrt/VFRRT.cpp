#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/rrt/VFRRT.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "PyGC.h"
#include "../../init.h"

namespace nb = nanobind;
namespace gc = ompl::binding::gc;
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace
{
    /// Holds the vector field in a member so the collector can see it; a callable captured by the
    /// std::function VFRRT stores is invisible to it, and a field that closes over the planner would leak.
    struct PyVFRRT : og::VFRRT
    {
        nb::object vf;

        PyVFRRT(const ob::SpaceInformationPtr &si, nb::object field, double exploration, double initial_lambda,
                unsigned int update_freq)
          : og::VFRRT(
                si, [this](const ob::State *state) { return nb::cast<Eigen::VectorXd>(vf(state)); }, exploration,
                initial_lambda, update_freq)
          , vf(std::move(field))
        {
        }

        // The stored vector field captures `this`.
        PyVFRRT(const PyVFRRT &) = delete;
        PyVFRRT &operator=(const PyVFRRT &) = delete;
    };
}  // namespace

void ompl::binding::geometric::initPlannersRrt_VFRRT(nb::module_ &m)
{
    nb::class_<PyVFRRT, og::RRT>(m, "VFRRT", nb::type_slots(gc::gcSlots<PyVFRRT, &PyVFRRT::vf>))
        .def(
            "__init__",
            [](PyVFRRT *self, const ob::SpaceInformationPtr &si, nb::callable vf, double exploration,
               double initial_lambda, unsigned int update_freq)
            { new (self) PyVFRRT(si, std::move(vf), exploration, initial_lambda, update_freq); },
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
