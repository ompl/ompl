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
