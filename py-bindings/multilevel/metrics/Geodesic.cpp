#include <nanobind/nanobind.h>

#include <ompl/multilevel/datastructures/metrics/Geodesic.h>
#include "../init.h"

namespace nb = nanobind;
namespace om = ompl::multilevel;

void ompl::binding::multilevel::initMetrics_Geodesic(nb::module_ &m)
{
    nb::class_<om::BundleSpaceMetricGeodesic, om::BundleSpaceMetric>(m, "Geodesic")
        .def(nb::init<om::BundleSpaceGraph *>(), nb::arg("graph"));
}
