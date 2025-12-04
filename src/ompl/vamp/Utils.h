#pragma once

#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ompl::vamp
{
    namespace ob = ompl::base;

    //==========================================================================
    // OMPL Conversion Utilities
    //==========================================================================

    /// Convert an OMPL state to a VAMP Configuration
    template <typename Robot>
    inline auto ompl_to_vamp(const ob::State *state) -> typename Robot::Configuration
    {
        using Configuration = typename Robot::Configuration;

        alignas(Configuration::S::Alignment)
            std::array<typename Configuration::S::ScalarT, Configuration::num_scalars>
                aligned_buffer{};

        const auto *as = state->as<ob::RealVectorStateSpace::StateType>();
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