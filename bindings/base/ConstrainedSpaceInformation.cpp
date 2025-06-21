#include <nanobind/nanobind.h>
#include "ompl/base/ConstrainedSpaceInformation.h"
#include "init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_ConstrainedSpaceInformation(nb::module_& m)
{
    // TODO [ob::ConstrainedMotionValidator][IMPLEMENT]
    nb::class_<ob::ConstrainedValidStateSampler, ob::ValidStateSampler>(m, "ConstrainedValidStateSampler")
        ;

    // TODO [ob::ConstrainedSpaceInformation][IMPLEMENT]
    nb::class_<ob::ConstrainedSpaceInformation, ob::SpaceInformation>(m, "ConstrainedSpaceInformation")
        ;

    // TODO [ob::TangentBundleSpaceInformation][IMPLEMENT]
    nb::class_<ob::TangentBundleSpaceInformation, ob::ConstrainedSpaceInformation>(m, "TangentBundleSpaceInformation")
        ;
}
