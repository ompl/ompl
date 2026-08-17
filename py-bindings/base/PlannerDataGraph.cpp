#include <nanobind/nanobind.h>

#include "ompl/base/PlannerDataGraph.h"
#include "ompl/base/PlannerData.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_PlannerDataGraph(nb::module_ &m)
{
    nb::class_<ob::PlannerData::Graph>(m, "PlannerDataGraph").def(nb::init<>());
}
