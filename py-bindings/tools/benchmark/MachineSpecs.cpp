#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include "ompl/tools/benchmark/MachineSpecs.h"
#include "../init.h"

namespace nb = nanobind;

void ompl::binding::tools::initBenchmark_MachineSpecs(nb::module_ &m)
{
    m.def("getProcessMemoryUsage", &ompl::machine::getProcessMemoryUsage);
    m.def("getHostname", &ompl::machine::getHostname);
    m.def("getCPUInfo", &ompl::machine::getCPUInfo);
}
