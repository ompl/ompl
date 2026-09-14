#include <nanobind/nanobind.h>

#include "init.h"

namespace nb = nanobind;

void ompl::binding::multilevel::init_BundleSpaceSequence(nb::module_ &m)
{
    // BundleSpaceSequence<T> is exposed through concrete planner classes (QRRT, QRRTStar, QMP, QMPStar).
    m.attr("BundleSpaceSequence") = m.attr("QRRT");
}
