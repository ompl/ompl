#include <nanobind/nanobind.h>

#include <ompl/multilevel/datastructures/graphsampler/RandomDegreeVertex.h>
#include "../init.h"

namespace nb = nanobind;
namespace om = ompl::multilevel;

void ompl::binding::multilevel::initGraphsampler_RandomDegreeVertex(nb::module_ &m)
{
    nb::class_<om::BundleSpaceGraphSamplerRandomDegreeVertex, om::BundleSpaceGraphSampler>(m, "RandomDegreeVertex")
        .def(nb::init<om::BundleSpaceGraph *>(), nb::arg("graph"));
}
