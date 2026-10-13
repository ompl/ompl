#pragma once

#include <cstddef>
#include <stdexcept>
#include <string>
#include <vector>

#include "ompl/base/StateSpace.h"

namespace ompl::binding::base
{
    /** \brief Validate the \e reals argument of a StateSpace::copyFromReals() call before handing it to C++. */
    inline void checkRealsSize(const ompl::base::StateSpace &space, const std::vector<double> &reals,
                               const char *function)
    {
        const std::size_t expected = space.getValueLocations().size();
        if (reals.size() == expected)
            return;

        std::string message = std::string(function) + "(): state space '" + space.getName() + "' expects " +
                              std::to_string(expected) + " real value(s), but " + std::to_string(reals.size()) +
                              " were given.";
        if (expected == 0)
            message += " This space has no known value locations; call setup() (or computeLocations()) on it first.";
        throw std::invalid_argument(message);
    }
}  // namespace ompl::binding::base
