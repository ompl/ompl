#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h>

#include <cstddef>
#include <stdexcept>
#include <string>

#include "ompl/base/State.h"
#include "ompl/base/StateSpace.h"

namespace ompl::binding
{
    namespace nb = nanobind;

    // Planners call the field without the GIL, and OMPL indexes its result without a length check.
    inline Eigen::VectorXd callVectorField(const nb::object &vf, const ompl::base::StateSpace &space,
                                           const ompl::base::State *state)
    {
        nb::gil_scoped_acquire gil;
        if (!vf.is_valid())
            throw std::runtime_error("the vector field is no longer set");
        auto f = nb::cast<Eigen::VectorXd>(vf(state));
        const std::size_t expected = space.getValueLocations().size();
        if (static_cast<std::size_t>(f.size()) != expected)
            throw nb::value_error(("the vector field returned " + std::to_string(f.size()) +
                                   " value(s), but state space '" + space.getName() + "' has " +
                                   std::to_string(expected) + " (call setup() first if that is 0)")
                                      .c_str());
        return f;
    }
}  // namespace ompl::binding
