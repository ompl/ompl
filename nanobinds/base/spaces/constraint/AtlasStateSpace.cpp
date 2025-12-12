#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <sstream>
#include "ompl/base/spaces/constraint/AtlasStateSpace.h"
#include "ompl/base/spaces/constraint/AtlasChart.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSpacesConstraint_AtlasStateSpace(nb::module_ &m)
{
    m.attr("ATLAS_STATE_SPACE_SAMPLES") = nb::cast(ompl::magic::ATLAS_STATE_SPACE_SAMPLES);
    m.attr("ATLAS_STATE_SPACE_EPSILON") = nb::cast(0.05);
    m.attr("ATLAS_STATE_SPACE_RHO_MULTIPLIER") = nb::cast(5);
    m.attr("ATLAS_STATE_SPACE_ALPHA") = nb::cast(boost::math::constants::pi<double>() / 8.0);
    m.attr("ATLAS_STATE_SPACE_EXPLORATION") = nb::cast(0.75);
    m.attr("ATLAS_STATE_SPACE_MAX_CHARTS_PER_EXTENSION") = nb::cast(200);
    m.attr("ATLAS_STATE_SPACE_BACKOFF") = nb::cast(0.75);

    // TODO [ob::AtlasStateSampler][TEST]
    nb::class_<ob::AtlasStateSampler, ob::StateSampler>(m, "AtlasStateSampler")
        .def(nb::init<const ob::AtlasStateSpace *>(), nb::arg("space"))
        .def("sampleUniform", &ob::AtlasStateSampler::sampleUniform, nb::arg("state"))
        .def("sampleUniformNear", &ob::AtlasStateSampler::sampleUniformNear, nb::arg("state"), nb::arg("near"),
             nb::arg("distance"))
        .def("sampleGaussian", &ob::AtlasStateSampler::sampleGaussian, nb::arg("state"), nb::arg("mean"),
             nb::arg("stdDev"));

     // TODO [ob::AtlasStateSampler][TEST]
    nb::class_<ompl::base::AtlasStateSpace::StateType, ompl::base::ConstrainedStateSpace::StateType> stateType(m, "Atla"
                                                                                                                  "sSta"
                                                                                                                  "teTy"
                                                                                                                  "pe");
                                                                                                                  
    // TODO [ob::AtlasStateSpace][TEST]
    nb::class_<ob::AtlasStateSpace, ob::ConstrainedStateSpace>(m, "AtlasStateSpace")
        .def(nb::init<const ob::StateSpacePtr &, const ob::ConstraintPtr &, bool>(), nb::arg("ambientSpace"),
             nb::arg("constraint"), nb::arg("separate") = true)
        .def("clear", &ob::AtlasStateSpace::clear)
        .def("allocDefaultStateSampler", &ob::AtlasStateSpace::allocDefaultStateSampler)
        .def("allocStateSampler", &ob::AtlasStateSpace::allocStateSampler)
        // setters
        .def("setEpsilon", &ob::AtlasStateSpace::setEpsilon, nb::arg("epsilon"))
        .def("setRho", &ob::AtlasStateSpace::setRho, nb::arg("rho"))
        .def("setAlpha", &ob::AtlasStateSpace::setAlpha, nb::arg("alpha"))
        .def("setExploration", &ob::AtlasStateSpace::setExploration, nb::arg("exploration"))
        .def("setMaxChartsPerExtension", &ob::AtlasStateSpace::setMaxChartsPerExtension, nb::arg("charts"))
        .def("setSeparated", &ob::AtlasStateSpace::setSeparated, nb::arg("separate"))
        .def("setBackoff", &ob::AtlasStateSpace::setBackoff, nb::arg("backoff"))
        // getters
        .def("getEpsilon", &ob::AtlasStateSpace::getEpsilon)
        .def("getRho", &ob::AtlasStateSpace::getRho)
        .def("getAlpha", &ob::AtlasStateSpace::getAlpha)
        .def("getExploration", &ob::AtlasStateSpace::getExploration)
        .def("getRho_s", &ob::AtlasStateSpace::getRho_s)
        .def("getMaxChartsPerExtension", &ob::AtlasStateSpace::getMaxChartsPerExtension)
        .def("isSeparated", &ob::AtlasStateSpace::isSeparated)
        .def("getChartCount", &ob::AtlasStateSpace::getChartCount)
        .def("getBackoff", &ob::AtlasStateSpace::getBackoff)
        // discrete geodesic
        .def(
            "discreteGeodesic",
            [](const ob::AtlasStateSpace &space, const ob::State *from, const ob::State *to, bool interpolate)
            {
                std::vector<ob::State *> geod;
                bool ok = space.discreteGeodesic(from, to, interpolate, &geod);
                return std::make_pair(ok, geod);
            },
            nb::arg("from"), nb::arg("to"), nb::arg("interpolate") = false)
        .def("anchorChart", &ob::AtlasStateSpace::anchorChart, nb::arg("state"), nb::rv_policy::reference_internal)
        .def("estimateFrontierPercent", &ob::AtlasStateSpace::estimateFrontierPercent)
        .def("printPLY", [](const ob::AtlasStateSpace &space) { 
            // space.printPLY(std::cout); 
            std::ostringstream oss;
            space.printPLY(oss);
            return oss.str();
        });
    }
