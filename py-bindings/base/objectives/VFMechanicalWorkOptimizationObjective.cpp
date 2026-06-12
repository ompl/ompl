#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/base/objectives/VFMechanicalWorkOptimizationObjective.h"
#include "ompl/base/objectives/MechanicalWorkOptimizationObjective.h"
#include "ompl/geometric/planners/rrt/VFRRT.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace
{
    og::VFRRT::VectorField wrapVectorField(og::VFRRT::VectorField vf)
    {
        return [vf = std::move(vf)](const ob::State *state) -> Eigen::VectorXd
        {
            nb::gil_scoped_acquire gil;
            return vf(state);
        };
    }
}  // namespace

void ompl::binding::base::initObjectives_VFMechanicalWorkOptimizationObjective(nb::module_ &m)
{
    nb::class_<ob::VFMechanicalWorkOptimizationObjective, ob::MechanicalWorkOptimizationObjective>(
        m, "VFMechanicalWorkOptimizationObjective")
        .def(
            "__init__",
            [](ob::VFMechanicalWorkOptimizationObjective *self, const ob::SpaceInformationPtr &si,
               og::VFRRT::VectorField vf)
            { new (self) ob::VFMechanicalWorkOptimizationObjective(si, wrapVectorField(std::move(vf))); },
            nb::arg("si"), nb::arg("vf"))
        .def("isSatisfied", &ob::VFMechanicalWorkOptimizationObjective::isSatisfied, nb::arg("c"))
        .def("motionCost", &ob::VFMechanicalWorkOptimizationObjective::motionCost, nb::arg("s1"), nb::arg("s2"))
        .def("isSymmetric", &ob::VFMechanicalWorkOptimizationObjective::isSymmetric);
}
