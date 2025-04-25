#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/Constraint.h"
#include "init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_Constraint(nb::module_& m)
{
    // TODO [ob::Constraint][IMPLEMENT]
    // TODO [ob::Constraint][TRAMPOLINE]
    nb::class_<ob::Constraint>(m, "Constraint");

    // TODO [ob::ConstraintIntersection][IMPLEMENT]
    nb::class_<ob::ConstraintIntersection, ob::Constraint>(m, "ConstraintIntersection");

    // TODO [ob::ConstraintUnion][IMPLEMENT]
    nb::class_<ob::ConstraintObjective, ob::OptimizationObjective>(m, "ConstraintObjective");
}
