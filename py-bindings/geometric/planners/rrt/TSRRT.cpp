#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/trampoline.h>

#include "ompl/geometric/planners/rrt/TSRRT.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "PyGC.h"
#include "PlannerSolve.h"
#include "../../init.h"

namespace nb = nanobind;
namespace gc = ompl::binding::gc;
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

namespace
{
    struct TSRRTAccess : og::TSRRT
    {
        static og::TaskSpaceConfigPtr &taskSpace(og::TSRRT &planner)
        {
            return planner.*(&TSRRTAccess::task_space_);
        }
    };

    int tsrrt_tp_traverse(PyObject *self, visitproc visit, void *arg)
    {
        if (int rc = ompl::binding::gc::traverseParent<ob::Planner>(self, visit, arg))
            return rc;
        if (!nb::inst_ready(self) || !gc::pythonOwned(self))
            return 0;
        return gc::visitHeld(TSRRTAccess::taskSpace(*nb::inst_ptr<og::TSRRT>(self)), visit, arg);
    }

    int tsrrt_tp_clear(PyObject *self)
    {
        ompl::binding::gc::clearParent<ob::Planner>(self);
        if (nb::inst_ready(self) && gc::pythonOwned(self))
        {
            og::TaskSpaceConfigPtr &taskSpace = TSRRTAccess::taskSpace(*nb::inst_ptr<og::TSRRT>(self));
            if (gc::holdsPython(taskSpace))
                taskSpace.reset();
        }
        return 0;
    }

    PyType_Slot tsrrt_slots[] = {{Py_tp_traverse, (void *)tsrrt_tp_traverse},
                                 {Py_tp_clear, (void *)tsrrt_tp_clear},
                                 {0, nullptr}};
}  // namespace

void ompl::binding::geometric::initPlannersRrt_TSRRT(nb::module_ &m)
{
    nb::class_<og::TaskSpaceConfig, PyTaskSpaceConfig>(m, "TaskSpaceConfig")
        .def(nb::init<>())
        .def("getDimension", &og::TaskSpaceConfig::getDimension)
        .def("project", &og::TaskSpaceConfig::project, nb::arg("state"), nb::arg("ts_proj"))
        .def("sample", &og::TaskSpaceConfig::sample, nb::arg("ts_proj"))
        .def("lift", &og::TaskSpaceConfig::lift, nb::arg("ts_proj"), nb::arg("seed"), nb::arg("state"));

    nb::class_<og::TSRRT, ob::Planner>(m, "TSRRT", nb::type_slots(tsrrt_slots))
        .def(nb::init<const ob::SpaceInformationPtr &, const og::TaskSpaceConfigPtr &>(), nb::arg("si"),
             nb::arg("task_space"))
        .def("solve", ompl::binding::solveBinding<og::TSRRT>(), nb::arg("what"))
        .def("solve", ompl::binding::solveFnBinding<og::TSRRT>(), nb::arg("terminationConditionFn"),
             nb::arg("checkInterval") = 0.0)
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
