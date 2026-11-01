#include <nanobind/nanobind.h>

#include "ompl/tools/config/MagicConstants.h"
#include "../init.h"

namespace nb = nanobind;

void ompl::binding::tools::initConfig_MagicConstants(nb::module_ &m)
{
    nb::module_ magic = m.def_submodule("magic");

    magic.attr("PROJECTION_DIMENSION_SPLITS") = ompl::magic::PROJECTION_DIMENSION_SPLITS;
    magic.attr("PROJECTION_EXTENTS_SAMPLES") = ompl::magic::PROJECTION_EXTENTS_SAMPLES;
    magic.attr("PROJECTION_EXPAND_FACTOR") = ompl::magic::PROJECTION_EXPAND_FACTOR;
    magic.attr("MAX_MOTION_LENGTH_AS_SPACE_EXTENT_FRACTION") = ompl::magic::MAX_MOTION_LENGTH_AS_SPACE_EXTENT_FRACTION;
    magic.attr("COST_MAX_MOTION_LENGTH_AS_SPACE_EXTENT_FRACTION") =
        ompl::magic::COST_MAX_MOTION_LENGTH_AS_SPACE_EXTENT_FRACTION;
    magic.attr("STD_DEV_AS_SPACE_EXTENT_FRACTION") = ompl::magic::STD_DEV_AS_SPACE_EXTENT_FRACTION;
    magic.attr("MAX_VALID_SAMPLE_ATTEMPTS") = ompl::magic::MAX_VALID_SAMPLE_ATTEMPTS;
    magic.attr("FIND_VALID_STATE_ATTEMPTS_WITHOUT_TERMINATION_CHECK") =
        ompl::magic::FIND_VALID_STATE_ATTEMPTS_WITHOUT_TERMINATION_CHECK;
    magic.attr("TEST_STATE_COUNT") = ompl::magic::TEST_STATE_COUNT;
    magic.attr("NEAREST_K_RECALL_SOLUTIONS") = ompl::magic::NEAREST_K_RECALL_SOLUTIONS;
}
