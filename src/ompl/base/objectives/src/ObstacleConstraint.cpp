/* Authors: John Schulman and Bryce Willey */

#include "ompl/base/objectives/ObstacleConstraint.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/trajopt/num_diff.h"
#include "ompl/trajopt/expr_ops.h"
#include "ompl/trajopt/sco_common.h"
#include "ompl/geometric/planners/trajopt/OmplOptProb.h"

/************** Numerical Collision Constraint Implementation. ****************/

Eigen::VectorXd ompl::base::ObstacleDistanceFunction::operator()(const Eigen::VectorXd& x) const
{
    int size = x.size();
    int dof = ss_->getDimension();
    int nSteps = size / dof;
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

ompl::base::NumericalCollisionTrajOptConstraint::NumericalCollisionTrajOptConstraint(
        ompl::base::StateValidityCheckerPtr sv,
        ompl::base::StateSpacePtr ss,
        const sco::VarVector& vars,
        double safeDist) :
        ConstraintFromFunc(
                sco::VectorOfVectorPtr(new ObstacleDistanceFunction(sv, ss, safeDist)),
                vars,
                Eigen::VectorXd::Ones(vars.size() / ss->getDimension()) * 1.0, // ss->getDimension(),
                sco::ConstraintType::INEQ,
                "ObsConstraint") {}

/************ Jacobian Collision Constraint Implementation. **************/

ompl::base::JacobianCollisionTrajOptConstraint::JacobianCollisionTrajOptConstraint(
        double safeDist,
        WorkspaceCollisionFn collision,
        StateSpacePtr ss,
        JacobianFn J,
        sco::VarVector vars) :
    eval_(new JacobianDiscreteCollisionEvaluator(collision, ss, J, vars)),
    safeDist_(safeDist)
{
    name_ = "JacobianCollision";
}

sco::ConvexConstraintsPtr ompl::base::JacobianCollisionTrajOptConstraint::convex(const std::vector<double>& x, sco::Model *model)
{
    sco::ConvexConstraintsPtr out(new sco::ConvexConstraints(model));
    std::vector<sco::AffExpr> exprs = eval_->calcDistanceExpressions(x);
    for (size_t i = 0; i < exprs.size(); i++) {
        sco::AffExpr viol = sco::exprSub(sco::AffExpr(safeDist_), exprs[i]);
        out->addIneqCnt(viol); // coeffs?
    }
    return out;
}

std::vector<double> ompl::base::JacobianCollisionTrajOptConstraint::value(const std::vector<double>& x)
{
    std::vector<double> dists = eval_->calcDistances(x);
    std::vector<double> out(dists.size());
    for (size_t i = 0; i < dists.size(); i++) {
        out[i] = sco::pospart(safeDist_ - dists[i]);
    }
    return out;
}

/************ OMPL Obstacle Constraint Implementation. ****************/

ompl::base::ObstacleConstraint::ObstacleConstraint(const ompl::base::SpaceInformationPtr &si, double safeDist) :
    ConvexifiableConstraint(si), safeDist_(safeDist) {}

ompl::base::ObstacleConstraint::ObstacleConstraint(
        const ompl::base::SpaceInformationPtr &si,
        double safeDist,
        WorkspaceCollisionFn collision,
        JacobianFn J) :
    ConvexifiableConstraint(si), collision_(collision), J_(J),
    useJacobians_(true), safeDist_(safeDist)
{}

ompl::base::Cost ompl::base::ObstacleConstraint::stateCost(const State *s) const
{
    double clearance = si_->getStateValidityChecker()->clearance(s);
    return Cost(sco::pospart(safeDist_ - clearance));
}

ompl::base::Cost ompl::base::ObstacleConstraint::motionCost(const State *s1, const State *s2) const
{
    // TODO: make this continious. TrajOpt isn't at the moment, but they detail how to (convex hulls.)
    double s1HingePenalty = stateCost(s1).value();
    double s2HingePenalty = stateCost(s2).value();
    return Cost(std::max(s1HingePenalty, s2HingePenalty));
}

std::vector<sco::ConstraintPtr> ompl::base::ObstacleConstraint::toConstraint(sco::OptProbPtr problem)
{
    std::vector<sco::ConstraintPtr> constraints;
    int size = problem->getNumVars();
    int dof = si_->getStateDimension();
    int nSteps = size / dof;
    printf("Using jacobians: %s\n", (useJacobians_) ? "true" : "false");
    printf("SafeDist: %f\n", safeDist_);
    for (int i = 0; i < nSteps; i++) {
        if (useJacobians_) {
            sco::ConstraintPtr constraint(new JacobianCollisionTrajOptConstraint(
                safeDist_, collision_, si_->getStateSpace(), J_,
                std::static_pointer_cast<ompl::geometric::OmplOptProb>(problem)->GetVarRow(i)
            ));
            constraints.push_back(constraint);
        } else {
            sco::ConstraintPtr constraint(new NumericalCollisionTrajOptConstraint(
                    si_->getStateValidityChecker(),
                    si_->getStateSpace(),
                    std::static_pointer_cast<ompl::geometric::OmplOptProb>(problem)->GetVarRow(i),
                    safeDist_));
            constraints.push_back(constraint);
        }
    }
    return constraints;
}
