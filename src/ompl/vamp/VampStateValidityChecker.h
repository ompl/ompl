#pragma once

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/vamp/Utils.h>

#include <vamp/planning/validate.hh>
#include <vamp/collision/environment.hh>

namespace ompl::vamp
{
    namespace ob = ompl::base;

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

    template <typename Robot, std::size_t rake = ::vamp::FloatVectorWidth>
    class VampStateValidityChecker : public ob::StateValidityChecker
    {
    public:
        using Environment = ::vamp::collision::Environment<::vamp::FloatVector<rake>>;

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
            return ::vamp::planning::validate_motion<Robot, rake, 1>(configuration, configuration, env_);
        }

    private:
        const Environment &env_;
    };

}  // namespace ompl::vamp
