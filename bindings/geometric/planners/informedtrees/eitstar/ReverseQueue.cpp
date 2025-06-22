#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/informedtrees/eitstar/ReverseQueue.h"
#include "../../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersInformedtreesEitstar_ReverseQueue(nb::module_& m)
{
    // TODO [og::eitstar::ReverseQueue][IMPLEMENT]
    nb::class_<og::eitstar::ReverseQueue>(m, "ReverseQueue")
        ;
}
