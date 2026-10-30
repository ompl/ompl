#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>

#include <vector>

#include "ompl/control/planners/syclop/Syclop.h"
#include "PyGC.h"
#include "../../init.h"

namespace nb = nanobind;
namespace gc = ompl::binding::gc;
namespace oc = ompl::control;
namespace ob = ompl::base;

namespace
{
    // The vector caster copies the out-parameter, so Python returns the lead instead.
    struct LeadCompute
    {
        gc::SharedRef fn;

        void operator()(int startRegion, int goalRegion, std::vector<int> &lead) const
        {
            lead = gc::call<std::vector<int>>(fn, startRegion, goalRegion);
        }
    };
}  // namespace

void ompl::binding::control::initPlannersSyclop_Syclop(nb::module_ &m)
{
    // Owned rather than reported: Syclop keeps these in private members, so a cycle through them leaks.
    nb::class_<oc::Syclop, ob::Planner>(m, "Syclop")
        .def(
            "setLeadComputeFn", [](oc::Syclop &p, nb::callable fn)
            { p.setLeadComputeFn(LeadCompute{gc::SharedRef(std::move(fn))}); }, nb::arg("compute"))
        .def(
            "addEdgeCostFactor", [](oc::Syclop &p, nb::callable fn)
            { p.addEdgeCostFactor(gc::function<oc::Syclop::EdgeCostFactorFn>(std::move(fn))); }, nb::arg("factor"))
        .def("clearEdgeCostFactors", &oc::Syclop::clearEdgeCostFactors)
        .def("getNumFreeVolumeSamples", &oc::Syclop::getNumFreeVolumeSamples)
        .def("setNumFreeVolumeSamples", &oc::Syclop::setNumFreeVolumeSamples, nb::arg("numSamples"))
        .def("getProbShortestPathLead", &oc::Syclop::getProbShortestPathLead)
        .def("setProbShortestPathLead", &oc::Syclop::setProbShortestPathLead, nb::arg("probability"))
        .def("getProbAddingToAvailableRegions", &oc::Syclop::getProbAddingToAvailableRegions)
        .def("setProbAddingToAvailableRegions", &oc::Syclop::setProbAddingToAvailableRegions, nb::arg("probability"))
        .def("getNumRegionExpansions", &oc::Syclop::getNumRegionExpansions)
        .def("setNumRegionExpansions", &oc::Syclop::setNumRegionExpansions, nb::arg("regionExpansions"))
        .def("getNumTreeExpansions", &oc::Syclop::getNumTreeExpansions)
        .def("setNumTreeExpansions", &oc::Syclop::setNumTreeExpansions, nb::arg("treeExpansions"))
        .def("getProbAbandonLeadEarly", &oc::Syclop::getProbAbandonLeadEarly)
        .def("setProbAbandonLeadEarly", &oc::Syclop::setProbAbandonLeadEarly, nb::arg("probability"));
}
