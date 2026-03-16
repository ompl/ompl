#pragma once

#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ompl::vamp
{
    namespace ob = ompl::base;

    /// Get OMPL RealVectorBounds from Robot scaling parameters
    template <typename Robot>
    inline auto getRobotBounds() -> ob::RealVectorBounds
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
        for (auto i = 0U; i < Robot::dimension; ++i)
        {
            bounds.setLow(i, zero_v[{0, i}]);
            bounds.setHigh(i, one_v[{0, i}]);
        }

        return bounds;
    }

    //==========================================================================
    // VAMP State Space for OMPL
    //==========================================================================

    template <typename Robot> 
    class VampStateSpace : public ob::RealVectorStateSpace
    {
    public:
        VampStateSpace() : ob::RealVectorStateSpace(Robot::dimension)
        {
            this->setBounds(getRobotBounds<Robot>());
        }
    };

}  // namespace ompl::vamp
