#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/control/Control.h"
#include "init.hh"

namespace nb = nanobind;
namespace oc = ompl::control;

void ompl::binding::control::init_Control(nb::module_& m)
{
    nb::class_<ompl::control::Control>(m, "Control");
    nb::class_<ompl::control::CompoundControl, ompl::control::Control>(m, "CompoundControl")
        .def(nb::init<>())
        .def("__getitem__", [](oc::CompoundControl &cc, unsigned int idx) -> oc::Control* {
                return cc[idx]; 
            }, nb::rv_policy::reference_internal,
            "Return the pointer to the i-th sub-control in this compound control.");
        // TODO: Expose the components member variable
}
