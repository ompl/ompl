#include "ompl/trajopt/expr_vec_ops.hpp"

sco::AffExpr sco::varDot(const VectorXd& x, const VarVector& v) {
  sco::AffExpr out;
  out.constant = 0;
  out.vars = v;
  out.coeffs = vector<double>(x.data(), x.data()+x.size());
  return out;
}
