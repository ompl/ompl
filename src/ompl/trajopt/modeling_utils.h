#pragma once
#include "modeling.h"
#include "num_diff.h"
#include "sco_common.h"
/**
 * @brief Build problem from user-defined functions
 * Utilities for creating Cost and Constraint objects from functions
 * using numerical derivatives or user-defined analytic derivatives.
 */

namespace sco
{
    enum PenaltyType
    {
        SQUARED,
        ABS,
        HINGE
    };

    using Eigen::VectorXd;
    using Eigen::MatrixXd;

    /**
     * x is the big solution vector of the whole problem. vars are variables that
     * index into the vector x. This function extracts (from x) the values of the
     * variables in vars into an Eigen Vector.
     */
    Eigen::VectorXd getVec(const vector<double> &x, const VarVector &vars);
    /**
     * Same idea as above, but different output type
     */
    std::vector<double> getDblVec(const vector<double> &x, const VarVector &vars);

    AffExpr affFromValGrad(double y, const VectorXd &x, const VectorXd &dydx, const VarVector &vars);

    class CostFromFunc : public Cost
    {
    public:
        /// supply function, obtain derivative and hessian numerically
        CostFromFunc(ScalarOfVectorPtr f, const VarVector &vars, const string &name, bool full_hessian = false);
        double value(const vector<double> &x);
        ConvexObjectivePtr convex(const vector<double> &x, Model *model);
        VarVector getVars()
        {
            return vars_;
        }

    protected:
        ScalarOfVectorPtr f_;
        VarVector vars_;
        bool full_hessian_;
        double epsilon_;
    };

    class CostFromErrFunc : public Cost
    {
    public:
        /// supply error function, obtain derivative numerically
        CostFromErrFunc(VectorOfVectorPtr f, const VarVector &vars, const VectorXd &coeffs, PenaltyType pen_type,
                        const string &name);
        /// supply error function and gradient
        CostFromErrFunc(VectorOfVectorPtr f, MatrixOfVectorPtr dfdx, const VarVector &vars, const VectorXd &coeffs,
                        PenaltyType pen_type, const string &name);
        double value(const vector<double> &x);
        ConvexObjectivePtr convex(const vector<double> &x, Model *model);
        VarVector getVars()
        {
            return vars_;
        }

    protected:
        VectorOfVectorPtr f_;
        MatrixOfVectorPtr dfdx_;
        VarVector vars_;
        VectorXd coeffs_;
        PenaltyType pen_type_;
        double epsilon_;
    };

    class ConstraintFromFunc : public Constraint
    {
    public:
        /// supply error function, obtain derivative numerically
        ConstraintFromFunc(VectorOfVectorPtr f, const VarVector &vars, const VectorXd &coeffs, ConstraintType type,
                           const std::string &name);
        /// supply error function and gradient
        ConstraintFromFunc(VectorOfVectorPtr f, MatrixOfVectorPtr dfdx, const VarVector &vars, const VectorXd &coeffs,
                           ConstraintType type, const std::string &name);
        vector<double> value(const vector<double> &x);
        ConvexConstraintsPtr convex(const vector<double> &x, Model *model);
        ConstraintType type()
        {
            return type_;
        }
        VarVector getVars()
        {
            return vars_;
        }

    protected:
        VectorOfVectorPtr f_;
        MatrixOfVectorPtr dfdx_;
        VarVector vars_;
        VectorXd coeffs_;
        ConstraintType type_;
        double epsilon_;
        VectorXd scaling_;
    };
}
