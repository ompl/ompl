#include <nanobind/nanobind.h>

#include "ompl/multilevel/datastructures/pathrestriction/FindSectionTypes.h"
#include "init.h"

namespace nb = nanobind;

void ompl::binding::multilevel::init_FindSectionType(nb::module_ &m)
{
    nb::enum_<ompl::multilevel::FindSectionType>(m, "FindSectionType")
        .value("NONE", ompl::multilevel::FindSectionType::NONE)
        .value("SIDE_STEP", ompl::multilevel::FindSectionType::SIDE_STEP)
        .value("PATTERN_DANCE", ompl::multilevel::FindSectionType::PATTERN_DANCE);
}
