#include <Eigen/Eigenvalues>
#include <iostream>

#include "ompl/trajopt/eigen_conversions.h"
#include "ompl/trajopt/expr_ops.h"
#include "ompl/trajopt/modeling.h"
#include "ompl/trajopt/modeling_utils.h"
using namespace std;

namespace sco
{
    const double DEFAULT_EPSILON = 1e-5;

    VectorXd getVec(const vector<double> &x, const VarVector &vars)
    {
        VectorXd out(vars.size());
        for (unsigned i = 0; i < vars.size(); ++i)
            out[i] = x[vars[i].var_rep->index];
        return out;
    }

    DblVec getDblVec(const vector<double> &x, const VarVector &vars)
    {
        DblVec out(vars.size());
        for (unsigned i = 0; i < vars.size(); ++i)
            out[i] = x[vars[i].var_rep->index];
        return out;
    }

    AffExpr affFromValGrad(double y, const VectorXd &x, const VectorXd &dydx, const VarVector &vars)
    {
        AffExpr aff;
        aff.constant = y - dydx.dot(x);
        aff.coeffs = toDblVec(dydx);
        aff.vars = vars;
        aff = cleanupAff(aff);
        return aff;
    }

    CostFromFunc::CostFromFunc(ScalarOfVectorPtr f, const VarVector &vars, const string &name, bool full_hessian)
      : Cost(name), f_(f), vars_(vars), full_hessian_(full_hessian), epsilon_(DEFAULT_EPSILON)
    {
    }

    double CostFromFunc::value(const vector<double> &xin)
    {
        VectorXd x = getVec(xin, vars_);
        return f_->call(x);
    }

    ConvexObjectivePtr CostFromFunc::convex(const vector<double> &xin, Model *model)
    {
        VectorXd x = getVec(xin, vars_);

        ConvexObjectivePtr out(new ConvexObjective(model));
        if (!full_hessian_)
        {
            double val;
            VectorXd grad, hess;
            calcGradAndDiagHess(*f_, x, epsilon_, val, grad, hess);
            hess = hess.cwiseMax(VectorXd::Zero(hess.size()));
            QuadExpr &quad = out->quad_;
            quad.affexpr.constant = val - grad.dot(x) + .5 * x.dot(hess.cwiseProduct(x));
            quad.affexpr.vars = vars_;
            quad.affexpr.coeffs = toDblVec(grad - hess.cwiseProduct(x));
            quad.vars1 = vars_;
            quad.vars2 = vars_;
            quad.coeffs = toDblVec(hess * .5);
        }
        else
        {
            double val;
            VectorXd grad;
            MatrixXd hess;
            calcGradHess(f_, x, epsilon_, val, grad, hess);

            MatrixXd pos_hess = MatrixXd::Zero(x.size(), x.size());
            Eigen::SelfAdjointEigenSolver<MatrixXd> es(hess);
            VectorXd eigvals = es.eigenvalues();
            MatrixXd eigvecs = es.eigenvectors();
            for (size_t i = 0, end = x.size(); i != end; ++i)
            {  // tricky --- eigen size() is signed
                if (eigvals(i) > 0)
                    pos_hess += eigvals(i) * eigvecs.col(i) * eigvecs.col(i).transpose();
            }

            QuadExpr &quad = out->quad_;
            quad.affexpr.constant = val - grad.dot(x) + .5 * x.dot(pos_hess * x);
            quad.affexpr.vars = vars_;
            quad.affexpr.coeffs = toDblVec(grad - pos_hess * x);

            int nquadterms = (x.size() * (x.size() - 1)) / 2;
            quad.coeffs.reserve(nquadterms);
            quad.vars1.reserve(nquadterms);
            quad.vars2.reserve(nquadterms);
            for (size_t i = 0, end = x.size(); i != end; ++i)
            {  // tricky --- eigen size() is signed
                quad.vars1.push_back(vars_[i]);
                quad.vars2.push_back(vars_[i]);
                quad.coeffs.push_back(pos_hess(i, i) / 2);
                for (size_t j = i + 1; j != end; ++j)
                {  // tricky --- eigen size() is signed
                    quad.vars1.push_back(vars_[i]);
                    quad.vars2.push_back(vars_[j]);
                    quad.coeffs.push_back(pos_hess(i, j));
                }
            }
        }

        return out;
    }

    CostFromErrFunc::CostFromErrFunc(VectorOfVectorPtr f, const VarVector &vars, const VectorXd &coeffs,
                                     PenaltyType pen_type, const std::string &name)
      : Cost(name), f_(f), vars_(vars), coeffs_(coeffs), pen_type_(pen_type), epsilon_(DEFAULT_EPSILON)
    {
    }
    CostFromErrFunc::CostFromErrFunc(VectorOfVectorPtr f, MatrixOfVectorPtr dfdx, const VarVector &vars,
                                     const VectorXd &coeffs, PenaltyType pen_type, const std::string &name)
      : Cost(name), f_(f), dfdx_(dfdx), vars_(vars), coeffs_(coeffs), pen_type_(pen_type), epsilon_(DEFAULT_EPSILON)
    {
    }
    double CostFromErrFunc::value(const vector<double> &xin)
    {
        VectorXd x = getVec(xin, vars_);
        VectorXd err = f_->call(x);
        if (coeffs_.size() > 0)
            err.array() *= coeffs_.array();
        switch (pen_type_)
        {
            case SQUARED:
                return err.array().square().sum();
            case ABS:
                return err.array().abs().sum();
            case HINGE:
                return err.cwiseMax(VectorXd::Zero(err.size())).sum();
            default:
                assert(0 && "unreachable");
        }

        return 0;  // avoid compiler warning
    }
    ConvexObjectivePtr CostFromErrFunc::convex(const vector<double> &xin, Model *model)
    {
        VectorXd x = getVec(xin, vars_);
        MatrixXd jac = (dfdx_) ? dfdx_->call(x) : calcForwardNumJac(*f_, x, epsilon_);
        ConvexObjectivePtr out(new ConvexObjective(model));
        VectorXd y = f_->call(x);
        for (int i = 0; i < jac.rows(); ++i)
        {
            AffExpr aff = affFromValGrad(y[i], x, jac.row(i), vars_);
            if (coeffs_.size() > 0)
            {
                exprScale(aff, coeffs_[i]);
                if (coeffs_[i] == 0)
                    continue;
            }
            switch (pen_type_)
            {
                case SQUARED:
                    out->addQuadExpr(exprSquare(aff));
                    break;
                case ABS:
                    out->addAbs(aff, 1);
                    break;
                case HINGE:
                    out->addHinge(aff, 1);
                    break;
                default:
                    assert(0 && "unreachable");
            }
        }
        return out;
    }

    ConstraintFromFunc::ConstraintFromFunc(VectorOfVectorPtr f, const VarVector &vars, const VectorXd &coeffs,
                                           ConstraintType type, const std::string &name)
      : Constraint(name), f_(f), vars_(vars), coeffs_(coeffs), type_(type), epsilon_(DEFAULT_EPSILON)
    {
    }

    ConstraintFromFunc::ConstraintFromFunc(VectorOfVectorPtr f, MatrixOfVectorPtr dfdx, const VarVector &vars,
                                           const VectorXd &coeffs, ConstraintType type, const std::string &name)
      : Constraint(name), f_(f), dfdx_(dfdx), vars_(vars), coeffs_(coeffs), type_(type), epsilon_(DEFAULT_EPSILON)
    {
    }

    vector<double> ConstraintFromFunc::value(const vector<double> &xin)
    {
        VectorXd x = getVec(xin, vars_);
        VectorXd err = f_->call(x);
        if (coeffs_.size() > 0)
            err.array() *= coeffs_.array();

        return toDblVec(err);
    }

    ConvexConstraintsPtr ConstraintFromFunc::convex(const vector<double> &xin, Model *model)
    {
        VectorXd x = getVec(xin, vars_);
        MatrixXd jac = (dfdx_) ? dfdx_->call(x) : calcForwardNumJac(*f_, x, epsilon_);
        ConvexConstraintsPtr out(new ConvexConstraints(model));
        VectorXd y = f_->call(x);
        for (int i = 0; i < jac.rows(); ++i)
        {
            AffExpr aff = affFromValGrad(y[i], x, jac.row(i), vars_);
            if (coeffs_.size() > 0)
            {
                if (coeffs_[i] == 0)
                    continue;
                exprScale(aff, coeffs_[i]);
            }
            if (type() == INEQ)
                out->addIneqCnt(aff);
            else
                out->addEqCnt(aff);
        }
        return out;
    }
}
