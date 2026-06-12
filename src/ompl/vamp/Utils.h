#pragma once

#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/WrapperStateSpace.h>

#include <array>
#include <cstddef>

namespace ompl::vamp
{
    namespace ob = ompl::base;

    //==========================================================================
    // OMPL Conversion Utilities
    //==========================================================================

    /// Unwrap a state for either a plain RealVectorStateSpace or a
    /// ConstrainedStateSpace (which inherits from WrapperStateSpace and
    /// holds the wrapped real-vector state internally). Lets the same
    /// validity/motion checker run in both unconstrained and constrained
    /// planning modes.
    inline auto extract_real_state(const ob::State *state) -> const ob::RealVectorStateSpace::StateType *
    {
        if (const auto *wrapper = dynamic_cast<const ob::WrapperStateSpace::StateType *>(state))
        {
            return wrapper->getState()->as<ob::RealVectorStateSpace::StateType>();
        }
        return state->as<ob::RealVectorStateSpace::StateType>();
    }

    /// Convert an OMPL state to a VAMP Configuration (full-body case).
    template <typename Robot>
    inline auto ompl_to_vamp(const ob::State *state) -> typename Robot::Configuration
    {
        using Configuration = typename Robot::Configuration;

        alignas(Configuration::S::Alignment) std::array<typename Configuration::S::ScalarT, Configuration::num_scalars>
            aligned_buffer{};

        const auto *as = extract_real_state(state);
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            aligned_buffer[i] = static_cast<float>(as->values[i]);
        }

        return Configuration(aligned_buffer.data());
    }

    /// Convert a VAMP Configuration to an OMPL state
    template <typename Robot>
    inline void vamp_to_ompl(const typename Robot::Configuration &config, ob::State *state)
    {
        auto *as = state->as<ob::RealVectorStateSpace::StateType>();
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            as->values[i] = static_cast<double>(config[{i, 0}]);
        }
    }

}  // namespace ompl::vamp
