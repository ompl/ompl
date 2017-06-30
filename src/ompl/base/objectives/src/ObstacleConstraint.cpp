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
                int size = x.size();
                int dof = ss_->getDimension();
                int nSteps = size / dof;
                // Use ConstraintFromFunc for now, just return a vector of 1.
                // TODO: figure out why exactly Constraints have to be Vec -> VectorXd
                Eigen::VectorXd toReturn(nSteps);
                for (int t = 0; t < nSteps; t++) {
                    State *state = ss_->allocState();
                    std::vector<double> stateVec(dof);
                    for (int i = 0; i < dof; i++) {
                        stateVec[i] = x(t * dof + i);
                    }
                    ss_->copyFromReals(state, stateVec);
                    double clearance = sv_->clearance(state);
                    toReturn[t] = sco::pospart(safeDist_ - clearance);
                }
                std::cout << toReturn.transpose() << std::endl << std::endl;
                return toReturn;
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
                Eigen::VectorXd::Ones(vars.size() / ss->getDimension()) * 1.0, // ss->getDimension(),
                sco::ConstraintType::INEQ,
                "ObsConstraint") {}

ompl::base::ObstacleConstraint::ObstacleConstraint(const ompl::base::SpaceInformationPtr &si, double safeDist) :
    ConvexifiableConstraint(si), safeDist_(safeDist) {}

ompl::base::Cost ompl::base::ObstacleConstraint::stateCost(const State *s) const {
    double clearance = si_->getStateValidityChecker()->clearance(s);
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
