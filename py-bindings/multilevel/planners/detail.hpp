#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "ompl/base/Planner.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/multilevel/datastructures/BundleSpaceGraph.h"
#include "ompl/multilevel/datastructures/PlannerMultiLevel.h"
#include "ompl/multilevel/datastructures/Projection.h"
#include "ompl/multilevel/datastructures/pathrestriction/FindSectionTypes.h"
#include "PlannerSolve.h"

namespace nb = nanobind;
namespace ob = ompl::base;

namespace ompl::binding::multilevel::detail
{
    template <typename PlannerT>
    struct BundleSpacesAccess : PlannerT
    {
        using PlannerT::bundleSpaces_;
    };

    /// The levels as views that keep the planner alive; the planner deletes them only in its destructor.
    template <typename PlannerT>
    nb::list bundleSpaces(nb::handle self)
    {
        const PlannerT &planner = nb::cast<const PlannerT &>(self);
        nb::list levels;
        for (auto *level : planner.*(&BundleSpacesAccess<PlannerT>::bundleSpaces_))
            levels.append(nb::cast(static_cast<ompl::multilevel::BundleSpaceGraph *>(level),
                                   nb::rv_policy::reference_internal, self));
        return levels;
    }

    inline void checkLevels(const std::vector<ob::SpaceInformationPtr> &siVec)
    {
        if (siVec.empty())
            throw nb::value_error("si_vec must name at least one level");
    }

    // The constructors already declare the levels; declareBundleSpaces() would add them a second time.
    template <typename PlannerT>
    void bindBundleSpaceSequencePlanner(nb::module_ &m, const char *pythonName)
    {
        nb::class_<PlannerT, ompl::multilevel::PlannerMultiLevel>(m, pythonName)
            .def(nb::new_([](ob::SpaceInformationPtr si, const std::string &type) { return new PlannerT(si, type); }),
                 nb::arg("si"), nb::arg("type") = "BundleSpacePlannerNonMultilevel")
            .def(nb::new_(
                     [](std::vector<ob::SpaceInformationPtr> siVec, const std::string &type)
                     {
                         checkLevels(siVec);
                         return new PlannerT(siVec, type);
                     }),
                 nb::arg("si_vec"), nb::arg("type") = "BundleSpacePlanner")
            .def(nb::new_(
                     [](std::vector<ob::SpaceInformationPtr> siVec,
                        std::vector<ompl::multilevel::ProjectionPtr> &projVec, const std::string &type)
                     {
                         checkLevels(siVec);
                         if (projVec.size() + 1 != siVec.size())
                             throw nb::value_error("proj_vec needs one projection between each pair of levels");
                         return new PlannerT(siVec, projVec, type);
                     }),
                 nb::arg("si_vec"), nb::arg("proj_vec"), nb::arg("type") = "BundleSpacePlannerCustomProjection")
            .def("solve", ompl::binding::solveBinding<PlannerT>(), nb::arg("what"))
            .def("solve", ompl::binding::solveFnBinding<PlannerT>(), nb::arg("terminationConditionFn"),
                 nb::arg("checkInterval") = 0.0)
            .def(
                "getPlannerData", [](const PlannerT &self, ob::PlannerData &data) { self.getPlannerData(data); },
                nb::arg("data"))
            .def("clear", &PlannerT::clear)
            .def("setup", &PlannerT::setup)
            .def("setProblemDefinition", &PlannerT::setProblemDefinition, nb::arg("pdef"))
            .def("setStopLevel", &PlannerT::setStopLevel, nb::arg("level"))
            .def("setFindSectionStrategy", &PlannerT::setFindSectionStrategy, nb::arg("type"))
            .def("getBundleSpaces", &bundleSpaces<PlannerT>);
    }
}  // namespace ompl::binding::multilevel::detail
