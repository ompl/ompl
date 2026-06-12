#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <iostream>
#include <sstream>

#include "ompl/control/planners/ltl/Automaton.h"
#include "../../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;

void ompl::binding::control::initPlannersLtl_Automaton(nb::module_ &m)
{
    nb::class_<oc::Automaton>(m, "Automaton")
        .def(nb::init<unsigned int, unsigned int>(), nb::arg("numProps"), nb::arg("numStates") = 0)
        .def("addState", &oc::Automaton::addState, nb::arg("accepting") = false)
        .def("setAccepting", &oc::Automaton::setAccepting, nb::arg("s"), nb::arg("a"))
        .def("isAccepting", &oc::Automaton::isAccepting, nb::arg("s"))
        .def("setStartState", &oc::Automaton::setStartState, nb::arg("s"))
        .def("getStartState", &oc::Automaton::getStartState)
        .def("addTransition", &oc::Automaton::addTransition, nb::arg("src"), nb::arg("w"), nb::arg("dest"))
        .def("run", &oc::Automaton::run, nb::arg("trace"))
        .def("step", &oc::Automaton::step, nb::arg("state"), nb::arg("w"))
        .def("numStates", &oc::Automaton::numStates)
        .def("numTransitions", &oc::Automaton::numTransitions)
        .def("numProps", &oc::Automaton::numProps)
        .def("distFromAccepting", &oc::Automaton::distFromAccepting, nb::arg("s"))
        .def("print", [](const oc::Automaton &a) { a.print(std::cout); })
        .def_static("AcceptingAutomaton", &oc::Automaton::AcceptingAutomaton, nb::arg("numProps"))
        .def_static("CoverageAutomaton",
                    nb::overload_cast<unsigned int, const std::vector<unsigned int> &>(
                        &oc::Automaton::CoverageAutomaton),
                    nb::arg("numProps"), nb::arg("covProps"))
        .def_static("CoverageAutomaton",
                    nb::overload_cast<unsigned int>(&oc::Automaton::CoverageAutomaton),
                    nb::arg("numProps"))
        .def_static("SequenceAutomaton",
                    nb::overload_cast<unsigned int, const std::vector<unsigned int> &>(
                        &oc::Automaton::SequenceAutomaton),
                    nb::arg("numProps"), nb::arg("seqProps"))
        .def_static("SequenceAutomaton",
                    nb::overload_cast<unsigned int>(&oc::Automaton::SequenceAutomaton),
                    nb::arg("numProps"))
        .def_static("DisjunctionAutomaton",
                    nb::overload_cast<unsigned int, const std::vector<unsigned int> &>(
                        &oc::Automaton::DisjunctionAutomaton),
                    nb::arg("numProps"), nb::arg("disjProps"))
        .def_static("DisjunctionAutomaton",
                    nb::overload_cast<unsigned int>(&oc::Automaton::DisjunctionAutomaton),
                    nb::arg("numProps"))
        .def_static("AvoidanceAutomaton", &oc::Automaton::AvoidanceAutomaton, nb::arg("numProps"),
                    nb::arg("avoidProps"));
}
