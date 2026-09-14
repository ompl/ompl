#include <nanobind/nanobind.h>

#include <ompl/multilevel/datastructures/metrics/BundleSpaceMetric.h>
#include "../init.h"

namespace nb = nanobind;
namespace om = ompl::multilevel;

void ompl::binding::multilevel::initMetrics_BundleSpaceMetric(nb::module_ &m)
{
    nb::class_<om::BundleSpaceMetric>(m, "BundleSpaceMetric").def("clear", &om::BundleSpaceMetric::clear);
}
