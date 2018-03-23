#include <cmath>
#include "ompl/trajopt/expr_ops.h"
#include "ompl/trajopt/sco_common.h"

namespace sco {
QuadExpr exprSquare(const Var& a) {
  QuadExpr out;
  out.coeffs.push_back(1);
  out.vars1.push_back(a);
  out.vars2.push_back(a);
  return out;
}

QuadExpr exprSquare(const AffExpr& affexpr) {
  QuadExpr out;
  size_t naff = affexpr.coeffs.size();
  size_t nquad = (naff*(naff+1))/2;

  out.affexpr.constant = sq(affexpr.constant);

  out.affexpr.vars = affexpr.vars;
  out.affexpr.coeffs.resize(naff);
  for (size_t i=0; i < naff; ++i) out.affexpr.coeffs[i] = 2*affexpr.constant*affexpr.coeffs[i];

  out.coeffs.reserve(nquad);
  out.vars1.reserve(nquad);
  out.vars2.reserve(nquad);
  for (size_t i=0; i < naff; ++i) {
    out.vars1.push_back(affexpr.vars[i]);
    out.vars2.push_back(affexpr.vars[i]);
    out.coeffs.push_back(sq(affexpr.coeffs[i]));
    for (size_t j=i+1; j < naff; ++j) {
      out.vars1.push_back(affexpr.vars[i]);
      out.vars2.push_back(affexpr.vars[j]);
      out.coeffs.push_back(2 * affexpr.coeffs[i] * affexpr.coeffs[j]);
    }
  }
  return out;
}


AffExpr cleanupAff(const AffExpr& a) {
  AffExpr out;
  for (size_t i=0; i < a.size(); ++i) {
    if (fabs(a.coeffs[i]) > 1e-7) {
      out.coeffs.push_back(a.coeffs[i]);
      out.vars.push_back(a.vars[i]);
    }
  }
  out.constant = a.constant;
  return out;
}

QuadExpr cleanupQuad(const QuadExpr& q) {
  QuadExpr out;
  out.affexpr = cleanupAff(q.affexpr);
  for (size_t i=0; i < q.size(); ++i) {
    if (fabs(q.coeffs[i]) > 1e-8) {
      out.coeffs.push_back(q.coeffs[i]);
      out.vars1.push_back(q.vars1[i]);
      out.vars2.push_back(q.vars2[i]);
    }
  }
  return out;
}

}
