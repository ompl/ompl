#include "solver_interface.hpp"
#include <Eigen/Core>

namespace sco {

using Eigen::MatrixXd;
using Eigen::VectorXd;

#if 0
typedef vector<AffExpr> ExprVector;
Matrix3d leftCrossProdMat(const Vector3d& x);
Matrix3d rightCrossProdMat(const Vector3d& x);

ExprVector exprMatMult(const MatrixXd& A, const VarVector& x);
ExprVector exprMatMult(const MatrixXd& A, const ExprVector& x);

ExprVector exprCross(const VectorXd& x, const VarVector& y);
ExprVector exprCross(const VarVector& x, const VectorXd& y);
ExprVector exprCross(const VectorXd& x, const ExprVector& y);
ExprVector exprCross(const ExprVector& x, const VectorXd& y);
#endif

AffExpr varDot(const VectorXd& x, const VarVector& v);
AffExpr exprDot(const VectorXd& x, const AffExprVector& v);

#if 0
QuadExpr varNorm2(const VarVector& v);
QuadExpr exprNorm2(const ExprVector& v);
#endif
}
