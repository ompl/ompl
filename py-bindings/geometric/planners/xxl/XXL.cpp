#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/geometric/planners/xxl/XXL.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "PyGC.h"
#include "PlannerSolve.h"
#include "../../init.h"

namespace nb = nanobind;
namespace gc = ompl::binding::gc;
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace
{
    struct XXLAccess : og::XXL
    {
        static og::XXLDecompositionPtr &decomposition(og::XXL &planner)
        {
            return planner.*(&XXLAccess::decomposition_);
        }
    };

    int xxl_tp_traverse(PyObject *self, visitproc visit, void *arg)
    {
        if (int rc = ompl::binding::gc::traverseParent<ob::Planner>(self, visit, arg))
            return rc;
        if (!nb::inst_ready(self) || !gc::pythonOwned(self))
            return 0;
        return gc::visitHeld(XXLAccess::decomposition(*nb::inst_ptr<og::XXL>(self)), visit, arg);
    }

    int xxl_tp_clear(PyObject *self)
    {
        ompl::binding::gc::clearParent<ob::Planner>(self);
        if (nb::inst_ready(self) && gc::pythonOwned(self))
        {
            og::XXLDecompositionPtr &decomposition = XXLAccess::decomposition(*nb::inst_ptr<og::XXL>(self));
            if (gc::holdsPython(decomposition))
                decomposition.reset();
        }
        return 0;
    }

    PyType_Slot xxl_slots[] = {{Py_tp_traverse, (void *)xxl_tp_traverse},
                               {Py_tp_clear, (void *)xxl_tp_clear},
                               {0, nullptr}};
}  // namespace

void ompl::binding::geometric::initPlannersXxl_XXL(nb::module_ &m)
{
    // XXL::setup() aborts the process (a bare `throw;`) without a decomposition, so one is required up front.
    nb::class_<og::XXL, ob::Planner>(m, "XXL", nb::type_slots(xxl_slots))
        .def(nb::init<const ob::SpaceInformationPtr &, const og::XXLDecompositionPtr &>(), nb::arg("si"),
             nb::arg("decomp").none(false))
        .def("solve", ompl::binding::solveBinding<og::XXL>(), nb::arg("what"))
        .def("solve", ompl::binding::solveFnBinding<og::XXL>(), nb::arg("terminationConditionFn"),
             nb::arg("checkInterval") = 0.0)
        .def(
            "getPlannerData", [](const og::XXL &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::XXL::clear)
        .def("setup", &og::XXL::setup)
        .def("setDecomposition", &og::XXL::setDecomposition, nb::arg("decomp").none(false))
        .def("getRandWalkRate", &og::XXL::getRandWalkRate)
        .def("setRandWalkRate", &og::XXL::setRandWalkRate, nb::arg("rate"));
}
