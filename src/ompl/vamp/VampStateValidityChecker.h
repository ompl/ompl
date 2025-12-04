#pragma once

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/vamp/Validate.hh>
#include <ompl/vamp/collision/environment.hh>

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

    /// Get OMPL RealVectorBounds from Robot scaling parameters
    template <typename Robot>
    inline auto get_robot_bounds() -> ob::RealVectorBounds
    {
        using Configuration = typename Robot::Configuration;

        std::array<float, Robot::dimension> zeros{};
        std::array<float, Robot::dimension> ones{};
        std::fill(ones.begin(), ones.end(), 1.0f);

        auto zero_v = Configuration(zeros);
        auto one_v = Configuration(ones);

        Robot::scale_configuration(zero_v);
        Robot::scale_configuration(one_v);

        ob::RealVectorBounds bounds(Robot::dimension);
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            bounds.setLow(i, zero_v[{i, 0}]);
            bounds.setHigh(i, one_v[{i, 0}]);
        }

        return bounds;
    }

    //==========================================================================
    // VAMP State Validity Checker for OMPL
    //==========================================================================

    template <typename Robot, std::size_t rake = FloatVectorWidth>
    class VampStateValidityChecker : public ob::StateValidityChecker
    {
    public:
        using Environment = collision::Environment<FloatVector<rake>>;

        VampStateValidityChecker(ob::SpaceInformation *si, const Environment &env)
          : ob::StateValidityChecker(si), env_(env)
        {
        }

        VampStateValidityChecker(const ob::SpaceInformationPtr &si, const Environment &env)
          : ob::StateValidityChecker(si), env_(env)
        {
        }

        auto isValid(const ob::State *state) const -> bool override
        {
            auto configuration = ompl_to_vamp<Robot>(state);
            return validate_motion<Robot, rake, 1>(configuration, configuration, env_);
        }

    private:
        const Environment &env_;
    };

}  // namespace ompl::vamp
