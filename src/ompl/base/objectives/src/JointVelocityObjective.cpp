/* Authors: John Schulman, Bryce Willey */
#include <cmath>
#include <Eigen/Core>

#include "ompl/base/objectives/JointVelocityObjective.h"
#include "ompl/trajopt/expr_ops.hpp"
#include "ompl/trajopt/utils.hpp"
#include "ompl/geometric/planners/trajopt/OmplOptProb.h"

/**
 * Takes the difference of each row from the previous row.
 */
static Eigen::MatrixXd diffPrevRow(const Eigen::MatrixXd& in) {
    // Subtracts rows 0:(end-1) from 1:end
    return in.middleRows(1, in.rows()-1) - in.middleRows(0, in.rows()-1);
}

ompl::base::JointVelCost::JointVelCost(const trajopt::VarArray& vars)
{
    for (int i = 0; i < vars.rows() - 1; i++)
    {
        for (int j = 0; j < vars.cols(); j++)
        {
            sco::AffExpr vel;
            sco::exprInc(vel, sco::exprMult(vars(i, j), -1));
            sco::exprInc(vel, vars(i+1, j));
            sco::exprInc(expr_, sco::exprSquare(vel));
        }
    }
}

sco::ConvexObjectivePtr ompl::base::JointVelCost::convex(const std::vector<double>& x, sco::Model* model)
{
    sco::ConvexObjectivePtr out(new sco::ConvexObjective(model));
    out->addQuadExpr(expr_);
    return out;
}

double ompl::base::JointVelCost::value(const std::vector<double>& xvec)
{
    Eigen::MatrixXd traj = trajopt::getTraj(xvec, vars_);
    return (diffPrevRow(traj).array().square().matrix()).sum();
}

/*****************************************
 * OMPL Side of Joint Velocity Objectives.
 ****************************************/

ompl::base::JointVelocityObjective::JointVelocityObjective(const ompl::base::SpaceInformationPtr &si) :
    ConvexifiableObjective(si)
{}

ompl::base::Cost ompl::base::JointVelocityObjective::stateCost(const State *s) const
{
    return identityCost();
}

ompl::base::Cost ompl::base::JointVelocityObjective::motionCost(const State *s1, const State *s2) const
{
    return Cost(pow(si_->distance(s1, s2), 2)); // TODO: add a coefficient when we know how to separate
    // joints.
}

sco::CostPtr ompl::base::JointVelocityObjective::toCost(sco::OptProbPtr problem)
{
    sco::CostPtr cptr(new JointVelCost((std::static_pointer_cast<ompl::geometric::OmplOptProb>(problem)->GetVars())));
    return cptr;
}
