#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/trampoline.h>

#include "ompl/geometric/planners/rrt/TSRRT.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

class PyTaskSpaceConfig : public og::TaskSpaceConfig
{
public:
    NB_TRAMPOLINE(og::TaskSpaceConfig, 4);

    int getDimension() const override
    {
        NB_OVERRIDE_PURE(getDimension);
    }

    void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> ts_proj) const override
    {
        NB_OVERRIDE_PURE(project, state, ts_proj);
    }

    void sample(Eigen::Ref<Eigen::VectorXd> ts_proj) const override
    {
        NB_OVERRIDE_PURE(sample, ts_proj);
    }

    bool lift(const Eigen::Ref<Eigen::VectorXd> &ts_proj, const ob::State *seed, ob::State *state) const override
    {
        NB_OVERRIDE_PURE(lift, ts_proj, seed, state);
    }
};

void ompl::binding::geometric::initPlannersRrt_TSRRT(nb::module_ &m)
{
    nb::class_<og::TaskSpaceConfig, PyTaskSpaceConfig>(m, "TaskSpaceConfig")
        .def(nb::init<>())
        .def("getDimension", &og::TaskSpaceConfig::getDimension)
        .def("project", &og::TaskSpaceConfig::project, nb::arg("state"), nb::arg("ts_proj"))
        .def("sample", &og::TaskSpaceConfig::sample, nb::arg("ts_proj"))
        .def("lift", &og::TaskSpaceConfig::lift, nb::arg("ts_proj"), nb::arg("seed"), nb::arg("state"));

    nb::class_<og::TSRRT, ob::Planner>(m, "TSRRT")
        .def(nb::init<const ob::SpaceInformationPtr &, const og::TaskSpaceConfigPtr &>(), nb::arg("si"),
             nb::arg("task_space"))
        .def("solve",
             [](og::TSRRT &self, nb::object what)
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
            "getPlannerData", [](const og::TSRRT &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::TSRRT::clear)
        .def("setup", &og::TSRRT::setup)
        .def("setGoalBias", &og::TSRRT::setGoalBias, nb::arg("goalBias"))
        .def("getGoalBias", &og::TSRRT::getGoalBias)
        .def("setRange", &og::TSRRT::setRange, nb::arg("distance"))
        .def("getRange", &og::TSRRT::getRange);
}
