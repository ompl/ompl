/* Authors: John Schulman and Bryce Willey */

#include "ompl/base/objectives/ObstacleConstraint.h"
#include "ompl/trajopt/num_diff.hpp"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/trajopt/sco_common.hpp"
#include "ompl/geometric/planners/trajopt/OmplOptProb.h"

namespace ompl {
    namespace base {
        struct TrajObstacleCalc : public sco::VectorOfVector {
            StateValidityCheckerPtr sv_;
            StateSpacePtr ss_;
            double safeDist_;
            TrajObstacleCalc(StateValidityCheckerPtr sv, StateSpacePtr ss, double safeDist) :
                sv_(sv), ss_(ss), safeDist_(safeDist)
            {}
            Eigen::VectorXd operator()(const Eigen::VectorXd& x) const {
                // TODO: stop assuming StateType and hardcoding nsteps and stuff.
                double clearanceSum = 0.0;
                for (int i = 0; i < 10; i++) {
                    State *state = ss_->allocState();
                    SE2StateSpace::StateType *se2State = state->as<SE2StateSpace::StateType>();
                    se2State->setX(x(i * 3));
                    se2State->setY(x(i * 3 + 1));
                    se2State->setYaw(x(i * 3 + 2));
                    double clearance = sv_->clearance(se2State);
                    printf("\tState: (%f, %f), yaw = %f, Clearance: %f\n", x(i * 3), x(i * 3 + 1), x(i * 3 + 2), clearance);
                    //printf("Size: %lu, State: (%f, %f), yaw = %f, Clearance: %f\n", x.size(), x(0), x(1), x(2), clearance);
                    clearanceSum += sco::pospart(safeDist_ - clearance);
                }
                // Use ConstraintFromFunc for now, just return a vector of 1.
                // TODO: figure out why exactly Constraints have to be Vec -> VectorXd
                Eigen::VectorXd toReturn(1);
                printf("ClearanceSum: %f\n", clearanceSum);
                return toReturn * clearanceSum;
            }
        };
    }
}

ompl::base::TrajObstacleConstraint::TrajObstacleConstraint(
        ompl::base::StateValidityCheckerPtr sv,
        ompl::base::StateSpacePtr ss,
        const sco::VarVector& vars,
        double safeDist) :
        ConstraintFromFunc(
                sco::VectorOfVectorPtr(new TrajObstacleCalc(sv, ss, safeDist)),
                vars,
                Eigen::VectorXd::Ones(1),
                sco::ConstraintType::INEQ,
                "ObstacleConstraint") {}

ompl::base::ObstacleConstraint::ObstacleConstraint(const ompl::base::SpaceInformationPtr &si, double safeDist) :
    ConvexifiableConstraint(si), safeDist_(safeDist) {}

ompl::base::Cost ompl::base::ObstacleConstraint::stateCost(const State *s) const {
    double clearance = si_->getStateValidityChecker()->clearance(s);
    printf("state: %s, clearance: %f\n", "...", clearance);
    return Cost(sco::pospart(safeDist_ - clearance));
}

ompl::base::Cost ompl::base::ObstacleConstraint::motionCost(const State *s1, const State *s2) const {
    // TODO: make this continious. TrajOpt isn't at the moment, but they detail how to (convex hulls.)
    double s1HingePenalty = stateCost(s1).value();
    double s2HingePenalty = stateCost(s2).value();
    return Cost(std::max(s1HingePenalty, s2HingePenalty));
}

sco::ConstraintPtr ompl::base::ObstacleConstraint::toConstraint(sco::OptProbPtr problem) {
    sco::ConstraintPtr constraint(new TrajObstacleConstraint(
            si_->getStateValidityChecker(),
            si_->getStateSpace(),
            std::static_pointer_cast<ompl::geometric::OmplOptProb>(problem)->getVars(),
            safeDist_));
    return constraint;
}
