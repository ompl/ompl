
#ifndef OMPL_BASE_OBJECTIVES_OBSTACLE_CONSTRAINT_
#define OMPL_BASE_OBJECTIVES_OBSTACLE_CONSTRAINT_

#include "ompl/base/objectives/ConvexifiableConstraint.h"
#include "ompl/trajopt/modeling_utils.hpp"
#include "ompl/trajopt/typedefs.hpp"

namespace ompl
{
    namespace base
    {
        struct TrajObstacleConstraint : public sco::ConstraintFromFunc
        {
            TrajObstacleConstraint(
                StateValidityCheckerPtr sv,
                StateSpacePtr ss,
                const sco::VarVector& vars,
                double safe_dist);
        };

        class ObstacleConstraint : public ConvexifiableConstraint
        {
        public:
            ObstacleConstraint(const SpaceInformationPtr &si, double safeDist);

            Cost stateCost(const State *s) const override;
            Cost motionCost(const State *s1, const State *s2) const override;

        protected:
            sco::ConstraintPtr toConstraint(sco::OptProbPtr problem) override;

            double safeDist_;
        };
    }
}


#endif
