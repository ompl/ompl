#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/objectives/VFUpstreamCriterionOptimizationObjective.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/geometric/planners/rrt/VFRRT.h"
#include "PyGC.h"
#include "base/objectives/VectorField.h"
#include "../init.h"

namespace nb = nanobind;
namespace gc = ompl::binding::gc;
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace
{
    /// Holds the vector field in a member so the collector can see it; see PyGC.h.
    struct PyVFUpstreamCriterion : ob::VFUpstreamCriterionOptimizationObjective
    {
        nb::object vf;

        PyVFUpstreamCriterion(const ob::SpaceInformationPtr &si, nb::object field)
          : ob::VFUpstreamCriterionOptimizationObjective(si, [this](const ob::State *state) { return evaluate(state); })
          , vf(std::move(field))
        {
        }

        // The stored vector field captures `this`.
        PyVFUpstreamCriterion(const PyVFUpstreamCriterion &) = delete;
        PyVFUpstreamCriterion &operator=(const PyVFUpstreamCriterion &) = delete;

        Eigen::VectorXd evaluate(const ob::State *state) const
        {
            return ompl::binding::callVectorField(vf, *si_->getStateSpace(), state);
        }
    };
}  // namespace

void ompl::binding::base::initObjectives_VFUpstreamCriterionOptimizationObjective(nb::module_ &m)
{
    nb::class_<PyVFUpstreamCriterion, ob::OptimizationObjective>(
        m, "VFUpstreamCriterionOptimizationObjective",
        nb::type_slots(gc::gcSlotsOver<ob::OptimizationObjective, PyVFUpstreamCriterion, &PyVFUpstreamCriterion::vf>))
        .def(
            "__init__", [](PyVFUpstreamCriterion *self, const ob::SpaceInformationPtr &si, nb::callable vf)
            { new (self) PyVFUpstreamCriterion(si, std::move(vf)); }, nb::arg("si"), nb::arg("vf"))
        .def("isSatisfied", &ob::VFUpstreamCriterionOptimizationObjective::isSatisfied, nb::arg("c"))
        .def("stateCost", &ob::VFUpstreamCriterionOptimizationObjective::stateCost, nb::arg("s"))
        .def("motionCost", &ob::VFUpstreamCriterionOptimizationObjective::motionCost, nb::arg("s1"), nb::arg("s2"))
        .def("isSymmetric", &ob::VFUpstreamCriterionOptimizationObjective::isSymmetric);
}
