#include <nanobind/nanobind.h>
#include "ompl/control/ControlSpaceTypes.h"
#include "init.hh"

namespace nb = nanobind;
namespace oc = ompl::control;

void ompl::binding::control::init_ControlSpaceTypes(nb::module_& m)
{
    nb::enum_<oc::ControlSpaceType>(m, "ControlSpaceType", "Enumeration of control space types in OMPL")
        .value("CONTROL_SPACE_UNKNOWN", oc::CONTROL_SPACE_UNKNOWN,
               "Unknown or unspecified control space type.")
        .value("CONTROL_SPACE_REAL_VECTOR", oc::CONTROL_SPACE_REAL_VECTOR,
               "A real-vector control space.")
        .value("CONTROL_SPACE_DISCRETE", oc::CONTROL_SPACE_DISCRETE,
               "A discrete control space.")
        .value("CONTROL_SPACE_TYPE_COUNT", oc::CONTROL_SPACE_TYPE_COUNT,
               "Number of distinct control space types in the enumeration.")
        .export_values();
}
