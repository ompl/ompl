#include <nanobind/nanobind.h>
#include "ompl/base/State.h"
#include "init.h"

namespace nb = nanobind;

void ompl::binding::base::init_State(nb::module_ &m)
{
    nb::class_<ompl::base::State>(m, "State");
    // No constructor or __setitem__: the components belong to the space that allocated them.
    nb::class_<ompl::base::CompoundState, ompl::base::State>(m, "CompoundState")
        .def(
            "__getitem__",
            [](ompl::base::CompoundState &self, unsigned int i)
            {
                if (self.components == nullptr)
                    throw nb::index_error("CompoundState has no components");
                return self[i];
            },
            nb::arg("index"), nb::rv_policy::reference_internal)
        // Without a length the sequence protocol would index past the last component.
        .def("__iter__", [](const ompl::base::CompoundState &)
             { throw nb::type_error("CompoundState is not iterable; index it up to its space's getSubspaceCount()"); });
}
