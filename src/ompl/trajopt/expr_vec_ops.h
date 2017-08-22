#include "solver_interface.h"
#include <Eigen/Core>

namespace sco {

using Eigen::MatrixXd;
using Eigen::VectorXd;

AffExpr varDot(const VectorXd& x, const VarVector& v);
AffExpr exprDot(const VectorXd& x, const AffExprVector& v);

}
