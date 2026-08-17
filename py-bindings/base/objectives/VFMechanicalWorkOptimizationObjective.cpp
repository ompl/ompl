#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/base/objectives/VFMechanicalWorkOptimizationObjective.h"
#include "ompl/base/objectives/MechanicalWorkOptimizationObjective.h"
#include "ompl/geometric/planners/rrt/VFRRT.h"
#include "PyGC.h"
#include "../init.h"

namespace nb = nanobind;
namespace gc = ompl::binding::gc;
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace
{
    /// Holds the vector field in a member so the collector can see it; see PyGC.h.
    struct PyVFMechanicalWork : ob::VFMechanicalWorkOptimizationObjective
    {
        nb::object vf;

        PyVFMechanicalWork(const ob::SpaceInformationPtr &si, nb::object field)
          : ob::VFMechanicalWorkOptimizationObjective(si, [this](const ob::State *state)
                                                      { return nb::cast<Eigen::VectorXd>(vf(state)); })
          , vf(std::move(field))
        {
        }

        // The stored vector field captures `this`.
        PyVFMechanicalWork(const PyVFMechanicalWork &) = delete;
        PyVFMechanicalWork &operator=(const PyVFMechanicalWork &) = delete;
    };
}  // namespace

void ompl::binding::base::initObjectives_VFMechanicalWorkOptimizationObjective(nb::module_ &m)
{
    nb::class_<PyVFMechanicalWork, ob::MechanicalWorkOptimizationObjective>(
        m, "VFMechanicalWorkOptimizationObjective",
        nb::type_slots(gc::gcSlots<PyVFMechanicalWork, &PyVFMechanicalWork::vf>))
        .def(
            "__init__", [](PyVFMechanicalWork *self, const ob::SpaceInformationPtr &si, nb::callable vf)
            { new (self) PyVFMechanicalWork(si, std::move(vf)); }, nb::arg("si"), nb::arg("vf"))
        .def("isSatisfied", &ob::VFMechanicalWorkOptimizationObjective::isSatisfied, nb::arg("c"))
        .def("motionCost", &ob::VFMechanicalWorkOptimizationObjective::motionCost, nb::arg("s1"), nb::arg("s2"))
        .def("isSymmetric", &ob::VFMechanicalWorkOptimizationObjective::isSymmetric);
}
