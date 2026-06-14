#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include <ompl/multilevel/datastructures/BundleSpace.h>
#include <ompl/multilevel/datastructures/Projection.h>
#include <ompl/base/OptimizationObjective.h>
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace om = ompl::multilevel;

void ompl::binding::multilevel::init_BundleSpace(nb::module_ &m)
{
    nb::class_<om::BundleSpace, ob::Planner>(m, "BundleSpace")
        .def("getBundle", &om::BundleSpace::getBundle, nb::rv_policy::reference_internal)
        .def("getBase", &om::BundleSpace::getBase, nb::rv_policy::reference_internal)
        .def("getProjection", &om::BundleSpace::getProjection)
        .def("makeProjection", &om::BundleSpace::makeProjection)
        .def("setProjection", &om::BundleSpace::setProjection, nb::arg("projection"))
        .def("setProblemDefinition", &om::BundleSpace::setProblemDefinition, nb::arg("pdef"))
        .def("setMetric", &om::BundleSpace::setMetric, nb::arg("metric"))
        .def("setPropagator", &om::BundleSpace::setPropagator, nb::arg("propagator"))
        .def("sampleFromDatastructure",
             [](om::BundleSpace &self, ob::State *xBase) { self.sampleFromDatastructure(xBase); }, nb::arg("xRandom"))
        .def("sampleBundle",
             [](om::BundleSpace &self, ob::State *xRandom) { self.sampleBundle(xRandom); }, nb::arg("xRandom"))
        .def("sampleBundleValid", &om::BundleSpace::sampleBundleValid, nb::arg("xRandom"))
        .def("hasSolution", &om::BundleSpace::hasSolution)
        .def("isInfeasible", &om::BundleSpace::isInfeasible)
        .def("hasConverged", &om::BundleSpace::hasConverged)
        .def("clear", &om::BundleSpace::clear)
        .def("setup", &om::BundleSpace::setup)
        .def("getImportance", &om::BundleSpace::getImportance)
        .def("allocIdentityStateBundle", &om::BundleSpace::allocIdentityStateBundle, nb::rv_policy::reference)
        .def("allocIdentityStateBase", &om::BundleSpace::allocIdentityStateBase, nb::rv_policy::reference)
        .def("resetCounter", &om::BundleSpace::resetCounter)
        .def("getBaseDimension", &om::BundleSpace::getBaseDimension)
        .def("getBundleDimension", &om::BundleSpace::getBundleDimension)
        .def("getCoDimension", &om::BundleSpace::getCoDimension)
        .def("getBundleSamplerPtr", &om::BundleSpace::getBundleSamplerPtr)
        .def("getBaseSamplerPtr", &om::BundleSpace::getBaseSamplerPtr)
        .def("getChild", &om::BundleSpace::getChild, nb::rv_policy::reference_internal)
        .def("setChild", &om::BundleSpace::setChild, nb::arg("child"))
        .def("getParent", &om::BundleSpace::getParent, nb::rv_policy::reference_internal)
        .def("setParent", &om::BundleSpace::setParent, nb::arg("parent"))
        .def("hasParent", &om::BundleSpace::hasParent)
        .def("hasBaseSpace", &om::BundleSpace::hasBaseSpace)
        .def("getLevel", &om::BundleSpace::getLevel)
        .def("setLevel", &om::BundleSpace::setLevel, nb::arg("level"))
        .def("project",
             [](const om::BundleSpace &self, const ob::State *xBundle, ob::State *xBase)
             { self.project(xBundle, xBase); },
             nb::arg("xBundle"), nb::arg("xBase"))
        .def("lift",
             [](const om::BundleSpace &self, const ob::State *xBase, ob::State *xBundle)
             { self.lift(xBase, xBundle); },
             nb::arg("xBase"), nb::arg("xBundle"))
        .def("getOptimizationObjectivePtr", &om::BundleSpace::getOptimizationObjectivePtr)
        .def("isDynamic", &om::BundleSpace::isDynamic)
        .def("getGoalPtr", &om::BundleSpace::getGoalPtr, nb::rv_policy::reference_internal);
}
