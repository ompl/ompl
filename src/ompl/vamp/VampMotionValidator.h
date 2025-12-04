#pragma once

#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/util/Exception.h>
#include <ompl/vamp/Utils.h>
#include <ompl/vamp/Validate.hh>
#include <ompl/vamp/collision/environment.hh>

namespace ompl::vamp
{
    namespace ob = ompl::base;

    template <typename Robot, std::size_t rake = FloatVectorWidth>
    class VampMotionValidator : public ob::MotionValidator
    {
    public:
        using Environment = collision::Environment<FloatVector<rake>>;

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
            return validate_motion<Robot, rake, Robot::resolution>(
                ompl_to_vamp<Robot>(s1),
                ompl_to_vamp<Robot>(s2),
                env_);
        }

        auto checkMotion(
            const ob::State *,
            const ob::State *,
            std::pair<ob::State *, double> &) const -> bool override
        {
            throw ompl::Exception("VampMotionValidator::checkMotion with lastValid not implemented");
        }

    private:
        const Environment &env_;
    };

}  // namespace ompl::vamp

