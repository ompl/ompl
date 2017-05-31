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
                // TODO: stop assuming StateType.
                State *state = ss_->allocState();
                SE2StateSpace::StateType *se2State = state->as<SE2StateSpace::StateType>();
                se2State->setX(x(0));
                se2State->setY(x(1));
                se2State->setYaw(x(2));
                double clearance = sv_->clearance(se2State);
                // Use ConstraintFromFunc for now, just return a vector of 1.
                // TODO: figure out why exactly Constraints have to be Vec -> VectorXd
                Eigen::VectorXd toReturn(1);
                return toReturn * sco::pospart(safeDist_ - clearance);
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
    return Cost(sco::pospart(safeDist_ - si_->getStateValidityChecker()->clearance(s)));
}

ompl::base::Cost ompl::base::ObstacleConstraint::motionCost(const State *s1, const State *s2) const {
    // TODO: make this continious. TrajOpt isn't at the moment, but they detail how to (convex hulls.)
    double s1HingePenalty = sco::pospart(safeDist_ - si_->getStateValidityChecker()->clearance(s1));
    double s2HingePenalty = sco::pospart(safeDist_ - si_->getStateValidityChecker()->clearance(s2));
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
