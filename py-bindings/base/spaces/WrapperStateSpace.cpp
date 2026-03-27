#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/eigen/dense.h>
#include "ompl/base/spaces/WrapperStateSpace.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/base/StateSpace.h"


#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpaces_WrapperStateSpace(nb::module_ &m)
{
     // TODO [ob::WrapperStateSpace][TEST]
     nb::class_<ob::WrapperStateSampler, ob::StateSampler>(m, "WrapperStateSampler")
          .def(nb::init<const ob::StateSpace *, ob::StateSamplerPtr>(), nb::arg("space"), nb::arg("sampler"),
               "Construct a wrapper sampler that delegates to a given sampler")
          .def("sampleUniform", &ob::WrapperStateSampler::sampleUniform, nb::arg("state"),
               "Uniformly sample the wrapped space")
          .def("sampleUniformNear", &ob::WrapperStateSampler::sampleUniformNear, nb::arg("state"), nb::arg("near"),
               nb::arg("distance"), "Uniformly sample near a state, delegated to the inner sampler")
          .def("sampleGaussian", &ob::WrapperStateSampler::sampleGaussian, nb::arg("state"), nb::arg("mean"),
               nb::arg("stdDev"), "Gaussian sample, delegated to the inner sampler");

     //
     // WrapperProjectionEvaluator
     //
     // TODO [ob::WrapperProjectionEvaluator][TEST]
     nb::class_<ob::WrapperProjectionEvaluator, ob::ProjectionEvaluator /* base */>(m, "WrapperProjectionEvaluator")
          .def(nb::init<const ob::WrapperStateSpace *>(), nb::arg("space"),
               "Wrap the default projection evaluator of an existing state space")
          .def("setup", &ob::WrapperProjectionEvaluator::setup, "Trigger any deferred initialization")
          .def("getDimension", &ob::WrapperProjectionEvaluator::getDimension, "Return the projection dimension")
          .def(
               "project",
               [](const ob::WrapperProjectionEvaluator &self, const ob::State *state, Eigen::Ref<Eigen::VectorXd> out)
               { self.project(state, out); }, nb::arg("state"), nb::arg("projection"),
               "Project a wrapped state into the low‑dimensional space");

     // TODO [ob::WrapperStateSpace::StateType][TEST]
     nb::class_<ompl::base::WrapperStateSpace::StateType, ompl::base::State> stateType(m, "WrapperStateType");
     //
     // WrapperStateSpace
     //
     // TODO [ob::WrapperStateSpace][TEST]
     nb::class_<ob::WrapperStateSpace, ob::StateSpace>(m, "WrapperStateSpace")
          .def(nb::init<const ob::StateSpacePtr &>(), nb::arg("space"),
               "Construct a wrapper around any existing StateSpace")
          .def("isCompound", &ob::WrapperStateSpace::isCompound)
          .def("isDiscrete", &ob::WrapperStateSpace::isDiscrete)
          .def("isHybrid", &ob::WrapperStateSpace::isHybrid)
          .def("isMetricSpace", &ob::WrapperStateSpace::isMetricSpace)
          .def("hasSymmetricDistance", &ob::WrapperStateSpace::hasSymmetricDistance)
          .def("hasSymmetricInterpolate", &ob::WrapperStateSpace::hasSymmetricInterpolate)
          .def("getName", &ob::WrapperStateSpace::getName)
          .def("setName", &ob::WrapperStateSpace::setName, nb::arg("name"))
          .def("getType", &ob::WrapperStateSpace::getType)
          .def("includes", nb::overload_cast<const ob::StateSpace *>(&ob::WrapperStateSpace::includes, nb::const_), nb::arg("other"))
          .def("covers", nb::overload_cast<const ob::StateSpace *>(&ob::WrapperStateSpace::covers, nb::const_), nb::arg("other"))
          .def("params", nb::overload_cast<>(&ob::WrapperStateSpace::params), nb::rv_policy::reference_internal)
          .def("getLongestValidSegmentFraction", &ob::WrapperStateSpace::getLongestValidSegmentFraction)
          .def("setLongestValidSegmentFraction", &ob::WrapperStateSpace::setLongestValidSegmentFraction,
               nb::arg("fraction"))
          .def("validSegmentCount", &ob::WrapperStateSpace::validSegmentCount, nb::arg("state1"), nb::arg("state2"))
          .def("setValidSegmentCountFactor", &ob::WrapperStateSpace::setValidSegmentCountFactor, nb::arg("factor"))
          .def("getValidSegmentCountFactor", &ob::WrapperStateSpace::getValidSegmentCountFactor)
          .def("getLongestValidSegmentLength", &ob::WrapperStateSpace::getLongestValidSegmentLength)
          .def("computeSignature", &ob::WrapperStateSpace::computeSignature, nb::arg("signature"))
          .def("getDimension", &ob::WrapperStateSpace::getDimension)
          .def("getMaximumExtent", &ob::WrapperStateSpace::getMaximumExtent)
          .def("getMeasure", &ob::WrapperStateSpace::getMeasure)
          .def("enforceBounds", &ob::WrapperStateSpace::enforceBounds, nb::arg("state"))
          .def("satisfiesBounds", &ob::WrapperStateSpace::satisfiesBounds, nb::arg("state"))
          .def("copyState", &ob::WrapperStateSpace::copyState, nb::arg("dest"), nb::arg("src"))
          .def("distance", &ob::WrapperStateSpace::distance, nb::arg("state1"), nb::arg("state2"))
          .def("getSerializationLength", &ob::WrapperStateSpace::getSerializationLength)
          .def("serialize", &ob::WrapperStateSpace::serialize, nb::arg("buffer"), nb::arg("state"))
          .def("deserialize", &ob::WrapperStateSpace::deserialize, nb::arg("state"), nb::arg("buffer"))
          .def("equalStates", &ob::WrapperStateSpace::equalStates, nb::arg("s1"), nb::arg("s2"))
          .def("interpolate", &ob::WrapperStateSpace::interpolate, nb::arg("from"), nb::arg("to"), nb::arg("t"),
               nb::arg("state"))
          .def("allocDefaultStateSampler", &ob::WrapperStateSpace::allocDefaultStateSampler)
          .def("allocState", &ob::WrapperStateSpace::allocState)
          .def("freeState", &ob::WrapperStateSpace::freeState, nb::arg("state"))
          .def("getValueAddressAtIndex", nb::overload_cast<ob::State*, unsigned int>(&ob::WrapperStateSpace::getValueAddressAtIndex, nb::const_), nb::arg("state"),
               nb::arg("index"))
          .def("getValueAddressAtLocation", nb::overload_cast<ob::State*, const ob::StateSpace::ValueLocation&>(&ob::WrapperStateSpace::getValueAddressAtLocation, nb::const_), nb::arg("state"),
               nb::arg("loc"))
          .def("getValueAddressAtName", nb::overload_cast<ob::State*, const std::string&>(&ob::WrapperStateSpace::getValueAddressAtName, nb::const_), nb::arg("state"), nb::arg("name"))
          .def("copyToReals", &ob::WrapperStateSpace::copyToReals, nb::arg("reals"), nb::arg("source"))
          .def("copyFromReals", &ob::WrapperStateSpace::copyFromReals, nb::arg("dest"), nb::arg("reals"))
          .def("registerProjections", &ob::WrapperStateSpace::registerProjections)
          .def("printState", &ob::WrapperStateSpace::printState, nb::arg("state"), nb::arg("out"))
          .def("printSettings", &ob::WrapperStateSpace::printSettings, nb::arg("out"))
          .def("printProjections", &ob::WrapperStateSpace::printProjections, nb::arg("out"))
          .def("sanityChecks", nb::overload_cast<double, double, unsigned int>(&ob::WrapperStateSpace::sanityChecks, nb::const_),
               nb::arg("zero"), nb::arg("eps"), nb::arg("flags"))
          .def("allocSubspaceStateSampler", &ob::WrapperStateSpace::allocSubspaceStateSampler, nb::arg("subspace"))
          .def("computeLocations", &ob::WrapperStateSpace::computeLocations)
          .def("setup", &ob::WrapperStateSpace::setup)
          .def("getSpace", &ob::WrapperStateSpace::getSpace, nb::rv_policy::reference_internal);
}
