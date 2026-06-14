#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "ompl/base/Planner.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/multilevel/datastructures/Projection.h"
#include "ompl/multilevel/datastructures/pathrestriction/FindSectionTypes.h"

namespace nb = nanobind;
namespace ob = ompl::base;

namespace ompl::binding::multilevel::detail
{
    inline auto plannerSolveOverload()
    {
        return [](ob::Planner &self, nb::object what) -> ob::PlannerStatus
        {
            if (nb::isinstance<ob::PlannerTerminationCondition>(what))
                return self.solve(nb::cast<ob::PlannerTerminationCondition>(what));
            if (nb::isinstance<double>(what))
                return self.solve(ob::timedPlannerTerminationCondition(nb::cast<double>(what)));
            throw nb::type_error("Invalid argument type for solve. Expected PlannerTerminationCondition or double.");
        };
    }

    template <typename PlannerT>
    void bindBundleSpaceSequencePlanner(nb::module_ &m, const char *pythonName)
    {
        nb::class_<PlannerT, ob::Planner>(m, pythonName)
            .def(nb::new_([](ob::SpaceInformationPtr si, const std::string &type) { return new PlannerT(si, type); }),
                 nb::arg("si"), nb::arg("type") = "BundleSpacePlannerNonMultilevel")
            .def(nb::new_([](std::vector<ob::SpaceInformationPtr> siVec, const std::string &type)
                          { return new PlannerT(siVec, type); }),
                 nb::arg("si_vec"), nb::arg("type") = "BundleSpacePlanner")
            .def(nb::new_([](std::vector<ob::SpaceInformationPtr> siVec,
                             std::vector<ompl::multilevel::ProjectionPtr> &projVec, const std::string &type)
                          { return new PlannerT(siVec, projVec, type); }),
                 nb::arg("si_vec"), nb::arg("proj_vec"), nb::arg("type") = "BundleSpacePlannerCustomProjection")
            .def("solve", plannerSolveOverload())
            .def(
                "getPlannerData", [](const PlannerT &self, ob::PlannerData &data) { self.getPlannerData(data); },
                nb::arg("data"))
            .def("clear", &PlannerT::clear)
            .def("setup", &PlannerT::setup)
            .def("setProblemDefinition", &PlannerT::setProblemDefinition, nb::arg("pdef"))
            .def("setStopLevel", &PlannerT::setStopLevel, nb::arg("level"))
            .def("setFindSectionStrategy", &PlannerT::setFindSectionStrategy, nb::arg("type"))
            .def("declareBundleSpaces", &PlannerT::declareBundleSpaces, nb::arg("guessProjection") = true);
    }
}  // namespace ompl::binding::multilevel::detail
