#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/informedtrees/eitstar/ForwardQueue.h"
#include "../../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersInformedtreesEitstar_ForwardQueue(nb::module_& m)
{
    // TODO [og::eitstar::ForwardQueue][IMPLEMENT]
    nb::class_<og::eitstar::ForwardQueue>(m, "ForwardQueue")
        ;
}
