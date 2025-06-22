#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/informedtrees/bitstar/IdGenerator.h"
#include "../../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersInformedtreesBitstar_IdGenerator(nb::module_& m)
{
    // TODO [og::BITstar::IdGenerator][IMPLEMENT]
    nb::class_<og::BITstar::IdGenerator>(m, "IdGenerator")
        ;
}
