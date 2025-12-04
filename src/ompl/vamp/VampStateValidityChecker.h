#pragma once

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/vamp/Utils.h>
#include <ompl/vamp/Validate.hh>
#include <ompl/vamp/collision/environment.hh>

namespace ompl::vamp
{
    namespace ob = ompl::base;

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

