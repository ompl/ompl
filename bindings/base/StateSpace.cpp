#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/StateSpace.h"
#include "init.hh"

namespace nb = nanobind;

void ompl::binding::base::init_StateSpace(nb::module_& m)
{
    nb::class_<ompl::base::StateSpace>(m, "StateSpace");
    
    nb::class_<ompl::base::CompoundStateSpace, ompl::base::StateSpace>(m, "CompoundStateSpace", 
    "A state space representing the composition of multiple state spaces")
    .def(nb::init<>(), "Construct an empty compound state space")
    .def(nb::init<const std::vector<ompl::base::StateSpacePtr>&, const std::vector<double>&>(),
         "Construct a compound state space from components and weights")
    
    // Management of subspaces
    .def("addSubspace", &ompl::base::CompoundStateSpace::addSubspace,
         "Add a state space as part of the compound state space with specified weight")
    .def("getSubspaceCount", &ompl::base::CompoundStateSpace::getSubspaceCount,
         "Get the number of state spaces in the compound state space")
    .def("getSubspace", nb::overload_cast<unsigned int>(&ompl::base::CompoundStateSpace::getSubspace, nb::const_),
         "Get a specific subspace by index")
    .def("getSubspace", nb::overload_cast<const std::string&>(&ompl::base::CompoundStateSpace::getSubspace, nb::const_),
         "Get a specific subspace by name")
    .def("hasSubspace", &ompl::base::CompoundStateSpace::hasSubspace,
         "Check if a specific subspace exists")
    .def("getSubspaceWeight", nb::overload_cast<unsigned int>(&ompl::base::CompoundStateSpace::getSubspaceWeight, nb::const_),
         "Get the weight of a subspace by index")
    .def("getSubspaceWeight", nb::overload_cast<const std::string&>(&ompl::base::CompoundStateSpace::getSubspaceWeight, nb::const_),
         "Get the weight of a subspace by name")
    .def("setSubspaceWeight", nb::overload_cast<unsigned int, double>(&ompl::base::CompoundStateSpace::setSubspaceWeight),
         "Set the weight of a subspace by index")
    .def("setSubspaceWeight", nb::overload_cast<const std::string&, double>(&ompl::base::CompoundStateSpace::setSubspaceWeight),
         "Set the weight of a subspace by name")
    .def("getSubspaces", &ompl::base::CompoundStateSpace::getSubspaces,
         "Get the list of component state spaces")
    .def("getSubspaceWeights", &ompl::base::CompoundStateSpace::getSubspaceWeights,
         "Get the list of component weights")
    .def("isLocked", &ompl::base::CompoundStateSpace::isLocked,
         "Return true if the state space is locked (no more components can be added)")
    .def("lock", &ompl::base::CompoundStateSpace::lock,
         "Lock this state space preventing addition of more components")

    // State space functionality
    .def("getDimension", &ompl::base::CompoundStateSpace::getDimension,
         "Get the dimension of the space")
    .def("getMaximumExtent", &ompl::base::CompoundStateSpace::getMaximumExtent,
         "Get the maximum extent of the space")
    .def("getMeasure", &ompl::base::CompoundStateSpace::getMeasure,
         "Get the measure of the space")
    .def("enforceBounds", &ompl::base::CompoundStateSpace::enforceBounds,
         "Bring a state within space bounds")
    .def("satisfiesBounds", &ompl::base::CompoundStateSpace::satisfiesBounds,
         "Check if a state is within space bounds")
    .def("distance", &ompl::base::CompoundStateSpace::distance,
         "Compute distance between states");
}
