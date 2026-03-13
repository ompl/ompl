#pragma once

#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/util/Exception.h>

#include <ompl/vamp/Utils.h>

#include <vamp/planning/validate.hh>
#include <vamp/collision/environment.hh>

namespace ompl::vamp
{
    namespace ob = ompl::base;

    //==========================================================================
    // VAMP Motion Validator for OMPL
    //==========================================================================

    template <typename Robot, std::size_t rake = ::vamp::FloatVectorWidth>
    class VampMotionValidator : public ob::MotionValidator
    {
    public:
        using Environment = ::vamp::collision::Environment<::vamp::FloatVector<rake>>;

        VampMotionValidator(ob::SpaceInformation *si, const Environment &env)
          : ob::MotionValidator(si), env_(env)
        {
        }

        VampMotionValidator(const ob::SpaceInformationPtr &si, const Environment &env)
          : ob::MotionValidator(si), env_(env)
        {
        }

        auto checkMotion(const ob::State *s1, const ob::State *s2) const -> bool override
        {
            return ::vamp::planning::validate_motion<Robot, rake, Robot::resolution>(
                ompl_to_vamp<Robot>(s1),
                ompl_to_vamp<Robot>(s2),
                env_);
        }

        auto checkMotion(
            const ob::State *s1,
            const ob::State *s2,
            std::pair<ob::State *, double> &last_valid) const -> bool override
        {
            // throw ompl::Exception("VampMotionValidator::checkMotion with lastValid not implemented");
            // TODO: VAMP does not support returning the last valid state, but it may become available in the future
            last_valid.first = nullptr;
            last_valid.second = 0.0;
            return ::vamp::planning::validate_motion<Robot, rake, Robot::resolution>(
                ompl_to_vamp<Robot>(s1),
                ompl_to_vamp<Robot>(s2),
                env_);
        }

    private:
        const Environment &env_;
    };

}  // namespace ompl::vamp
