#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/rrt/VFRRT.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "PyGC.h"
#include "PlannerSolve.h"
#include "base/objectives/VectorField.h"
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
                si, [this](const ob::State *state)
                { return ompl::binding::callVectorField(vf, *si_->getStateSpace(), state); }, exploration,
                initial_lambda, update_freq)
          , vf(std::move(field))
        {
        }

        // The stored vector field captures `this`.
        PyVFRRT(const PyVFRRT &) = delete;
        PyVFRRT &operator=(const PyVFRRT &) = delete;
    };

    int vfrrt_tp_traverse(PyObject *self, visitproc visit, void *arg)
    {
        if (int rc = ompl::binding::gc::traverseParent<og::RRT>(self, visit, arg))
            return rc;
        if (!nb::inst_ready(self) || !gc::pythonOwned(self))
            return 0;
        const nb::object &vf = nb::inst_ptr<PyVFRRT>(self)->vf;
        if (vf.is_valid())
            Py_VISIT(vf.ptr());
        return 0;
    }

    int vfrrt_tp_clear(PyObject *self)
    {
        ompl::binding::gc::clearParent<og::RRT>(self);
        if (nb::inst_ready(self) && gc::pythonOwned(self))
            nb::inst_ptr<PyVFRRT>(self)->vf.reset();
        return 0;
    }

    PyType_Slot vfrrt_slots[] = {{Py_tp_traverse, (void *)vfrrt_tp_traverse},
                                 {Py_tp_clear, (void *)vfrrt_tp_clear},
                                 {0, nullptr}};
}  // namespace

void ompl::binding::geometric::initPlannersRrt_VFRRT(nb::module_ &m)
{
    nb::class_<PyVFRRT, og::RRT>(m, "VFRRT", nb::type_slots(vfrrt_slots))
        .def(
            "__init__",
            [](PyVFRRT *self, const ob::SpaceInformationPtr &si, nb::callable vf, double exploration,
               double initial_lambda, unsigned int update_freq)
            { new (self) PyVFRRT(si, std::move(vf), exploration, initial_lambda, update_freq); },
            nb::arg("si"), nb::arg("vf"), nb::arg("exploration"), nb::arg("initial_lambda"), nb::arg("update_freq"))
        .def("solve", ompl::binding::solveBinding<PyVFRRT>(), nb::arg("what"))
        .def("solve", ompl::binding::solveFnBinding<PyVFRRT>(), nb::arg("terminationConditionFn"),
             nb::arg("checkInterval") = 0.0)
        .def(
            "getPlannerData", [](const PyVFRRT &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::VFRRT::clear)
        .def("setup", &og::VFRRT::setup);
}
