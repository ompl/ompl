#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <cstring>
#include <sstream>
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
void ompl::binding::base::initSpaces_RealVectorStateSpace(nb::module_ &m)
{
    nb::class_<ob::RealVectorStateSampler, ob::StateSampler>(m, "RealVectorStateSampler")
        .def(nb::init<const ob::StateSpace *>())
        .def("sampleUniform", &ob::RealVectorStateSampler::sampleUniform)
        .def("sampleUniformNear", &ob::RealVectorStateSampler::sampleUniformNear)
        .def("sampleGaussian", &ob::RealVectorStateSampler::sampleGaussian);

    nb::class_<ob::RealVectorStateSpace::StateType, ob::State> stateType(m, "RealVectorStateType");
    stateType
        // Single element access (no bounds checking)
        .def("__getitem__",
             [](const ob::RealVectorStateSpace::StateType *s, unsigned int i) { 
                 return s->values[i]; 
             }, nb::rv_policy::reference_internal)
        .def("__setitem__",
             [](ob::RealVectorStateSpace::StateType *s, unsigned int i, double v) { 
                 s->values[i] = v; 
             })
        // Slice access (optimized, no bounds checking)
        // Note: User is responsible for ensuring indices are within valid range
        .def("__getitem__",
             [](const ob::RealVectorStateSpace::StateType *s,
                const nb::slice &slice) {

                 // Reject open-ended slices: arr[:], arr[a:], arr[::k]
                 if (slice.attr("stop").is_none()) {
                     throw nb::type_error(
                         "Open-ended slices are not supported (unknown state dim)");
                 }

                 // Extract stop (required)
                 ssize_t stop = nb::cast<ssize_t>(slice.attr("stop"));

                 // Extract start (default 0)
                 ssize_t start = 0;
                 if (!slice.attr("start").is_none())
                     start = nb::cast<ssize_t>(slice.attr("start"));

                 // Extract step (default 1)
                 ssize_t step = 1;
                 if (!slice.attr("step").is_none())
                     step = nb::cast<ssize_t>(slice.attr("step"));

                 // Reject negative or zero step
                 if (step <= 0) {
                     throw nb::type_error("Slice step must be a positive integer");
                 }

                 // Reject negative indices (need length to resolve them)
                 if (start < 0 || stop < 0) {
                     throw nb::type_error(
                         "Negative slice indices require a known dim count");
                 }

                 // Empty slice
                 if (stop <= start) {
                     return std::vector<double>{};
                 }

                 // Compute slice length safely
                 size_t slicelength =
                     static_cast<size_t>((stop - start + step - 1) / step);

                 std::vector<double> result(slicelength);

                 if (step == 1) {
                     // Contiguous fast path
                     std::memcpy(result.data(),
                                 &s->values[start],
                                 slicelength * sizeof(double));
                 } else {
                     // Strided copy
                     for (size_t i = 0; i < slicelength; ++i) {
                         result[i] = s->values[start + i * step];
                     }
                 }

                 return result;
             })
        .def("__setitem__",
             [](ob::RealVectorStateSpace::StateType *s,
                const nb::slice &slice,
                const std::vector<double> &values) {

                 // Reject open-ended slices: arr[:], arr[a:], arr[::k]
                 if (slice.attr("stop").is_none()) {
                     throw nb::type_error(
                         "Open-ended slices are not supported (unknown state dim)");
                 }

                 // Extract stop (required)
                 ssize_t stop = nb::cast<ssize_t>(slice.attr("stop"));

                 // Extract start (default 0)
                 ssize_t start = 0;
                 if (!slice.attr("start").is_none())
                     start = nb::cast<ssize_t>(slice.attr("start"));

                 // Extract step (default 1)
                 ssize_t step = 1;
                 if (!slice.attr("step").is_none())
                     step = nb::cast<ssize_t>(slice.attr("step"));

                 // Reject negative or zero step
                 if (step <= 0) {
                     throw nb::type_error("Slice step must be a positive integer");
                 }

                 // Reject negative indices (need length to resolve them)
                 if (start < 0 || stop < 0) {
                     throw nb::type_error(
                         "Negative slice indices require a known dim count");
                 }

                 // Compute slice length safely
                 size_t slicelength = 0;
                 if (stop > start) {
                    slicelength = static_cast<size_t>((stop - start + step - 1) / step);
                 }
                
                 if (values.size() != slicelength) {
                     throw nb::value_error(
                         "Size of assignment does not match slice size");
                 }

                 if (step == 1) {
                     // Contiguous memory - use memcpy for maximum efficiency
                     std::memcpy(&s->values[start],
                                 values.data(),
                                 slicelength * sizeof(double));
                 } else {
                     // Non-contiguous - manual loop
                     for (size_t i = 0; i < slicelength; ++i) {
                         s->values[start + i * step] = values[i];
                     }
                 }
             }, nb::rv_policy::reference_internal,
             "Slice assignment to state values.\n\n"
             "WARNING: Open-ended slices like state[:] or state[1:] do NOT work because\n"
             "StateType does not know its dimension. You must use explicit bounds:\n"
             "  state[0:dim] = [...]  # OK\n"
             "  state[:]     = [...]  # WRONG\n");

    // Bind RealVectorStateSpace
    nb::class_<ob::RealVectorStateSpace, ob::StateSpace>(m, "RealVectorStateSpace")
        .def(nb::init<unsigned int>())
        .def(nb::init<>())
        .def("getDimension", &ob::RealVectorStateSpace::getDimension)
        .def("addDimension", nb::overload_cast<double, double>(&ob::RealVectorStateSpace::addDimension))
        .def("addDimension",
             nb::overload_cast<const std::string &, double, double>(&ob::RealVectorStateSpace::addDimension))
        .def("setBounds",
             nb::overload_cast<const ob::RealVectorBounds &>(&ob::RealVectorStateSpace::setBounds))
        .def("setBounds", nb::overload_cast<double, double>(&ob::RealVectorStateSpace::setBounds))
        .def("getBounds", &ob::RealVectorStateSpace::getBounds)
        .def("getDimensionName", &ob::RealVectorStateSpace::getDimensionName)
        .def("getDimensionIndex", &ob::RealVectorStateSpace::getDimensionIndex)
        .def("setDimensionName", &ob::RealVectorStateSpace::setDimensionName)
        .def("getMaximumExtent", &ob::RealVectorStateSpace::getMaximumExtent)
        .def("getMeasure", &ob::RealVectorStateSpace::getMeasure)
        .def("enforceBounds", &ob::RealVectorStateSpace::enforceBounds)
        .def("satisfiesBounds", &ob::RealVectorStateSpace::satisfiesBounds)
        .def("copyState", &ob::RealVectorStateSpace::copyState)
        .def("getSerializationLength", &ob::RealVectorStateSpace::getSerializationLength)
        .def("serialize", &ob::RealVectorStateSpace::serialize)
        .def("deserialize", &ob::RealVectorStateSpace::deserialize)
        .def("distance", &ob::RealVectorStateSpace::distance)
        .def("equalStates", &ob::RealVectorStateSpace::equalStates)
        .def("interpolate", &ob::RealVectorStateSpace::interpolate)
        .def("allocDefaultStateSampler", &ob::RealVectorStateSpace::allocDefaultStateSampler)
        .def("allocState", &ob::RealVectorStateSpace::allocState)
        .def("getValueAddressAtIndex", &ob::RealVectorStateSpace::getValueAddressAtIndex)
        .def(
            "printState", [](const ob::RealVectorStateSpace &space, const ob::State *state)
            { space.printState(state, std::cout); })
        .def(
            "printSettings", [](const ob::RealVectorStateSpace &space) { space.printSettings(std::cout); })
        .def("registerProjections", &ob::RealVectorStateSpace::registerProjections)
        .def("setup", &ob::RealVectorStateSpace::setup);
}