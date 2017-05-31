#include "ompl/trajopt/modeling.hpp"
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <cstdio>
#include "ompl/trajopt/expr_ops.hpp"
#include "ompl/trajopt/sco_common.hpp"
#include "ompl/trajopt/macros.h"
#include <iostream>
#include <sstream>

using namespace std;

namespace sco {

void ConvexObjective::addAffExpr(const AffExpr& affexpr) {
  exprInc(quad_, affexpr);
}
void ConvexObjective::addQuadExpr(const QuadExpr& quadexpr) {
  exprInc(quad_, quadexpr);
}
void ConvexObjective::addHinge(const AffExpr& affexpr, double coeff) {
  Var hinge = model_->addVar("hinge", 0, INFINITY);
  vars_.push_back(hinge);
  ineqs_.push_back(affexpr);
  exprDec(ineqs_.back(), hinge);
  AffExpr hinge_cost = exprMult(AffExpr(hinge), coeff);
  exprInc(quad_, hinge_cost);
}
void ConvexObjective::addAbs(const AffExpr& affexpr, double coeff) {
  Var neg = model_->addVar("neg", 0, INFINITY);
  Var pos = model_->addVar("pos", 0, INFINITY);
  vars_.push_back(neg);
  vars_.push_back(pos);
  AffExpr neg_plus_pos;
  neg_plus_pos.coeffs = vector<double>(2, coeff);
  neg_plus_pos.vars.push_back(neg);
  neg_plus_pos.vars.push_back(pos);
  exprInc(quad_, neg_plus_pos);
  AffExpr affeq = affexpr;
  affeq.vars.push_back(neg);
  affeq.vars.push_back(pos);
  affeq.coeffs.push_back(1);
  affeq.coeffs.push_back(-1);
  eqs_.push_back(affeq);
}
void ConvexObjective::addHinges(const AffExprVector& ev) {
  for (size_t i=0; i < ev.size(); ++i) addHinge(ev[i],1);
}
void ConvexObjective::addL1Norm(const AffExprVector& ev) {
  for (size_t i=0; i < ev.size(); ++i) addAbs(ev[i],1);
}
void ConvexObjective::addL2Norm(const AffExprVector& ev) {
  for (size_t i=0; i < ev.size(); ++i) exprInc(quad_, exprSquare(ev[i]));
}
void ConvexObjective::addMax(const AffExprVector& ev) {
  Var m = model_->addVar("max", -INFINITY, INFINITY);
  for (size_t i=0; i < ev.size(); ++i) {
    ineqs_.push_back(ev[i]);
    exprDec(ineqs_.back(), m);
  }
}

void ConvexObjective::addConstraintsToModel() {
  cnts_.reserve(eqs_.size() + ineqs_.size());
  BOOST_FOREACH(const AffExpr& aff, eqs_) {
    cnts_.push_back(model_->addEqCnt(aff, ""));
  }
  BOOST_FOREACH(const AffExpr& aff, ineqs_) {
    cnts_.push_back(model_->addIneqCnt(aff, ""));
  }
}

void ConvexObjective::removeFromModel() {
  model_->removeCnts(cnts_);
  model_->removeVars(vars_);
  model_ = NULL;
}
ConvexObjective::~ConvexObjective() {
  if (inModel()) removeFromModel();
}

void ConvexConstraints::addEqCnt(const AffExpr& aff) {
  eqs_.push_back(aff);
}

void ConvexConstraints::addIneqCnt(const AffExpr& aff) {
  ineqs_.push_back(aff);
}

void ConvexConstraints::addConstraintsToModel() {
  cnts_.reserve(eqs_.size() + ineqs_.size());
  BOOST_FOREACH(const AffExpr& aff, eqs_) {
    cnts_.push_back(model_->addEqCnt(aff, ""));
  }
  BOOST_FOREACH(const AffExpr& aff, ineqs_) {
    cnts_.push_back(model_->addIneqCnt(aff, ""));
  }
}

void ConvexConstraints::removeFromModel() {
  model_->removeCnts(cnts_);
  model_ = NULL;
}

vector<double> ConvexConstraints::violations(const vector<double>& x) {
  DblVec out;
  out.reserve(eqs_.size() + ineqs_.size());
  BOOST_FOREACH(const AffExpr& aff, eqs_) out.push_back(fabs(aff.value(x.data())));
  BOOST_FOREACH(const AffExpr& aff, ineqs_) out.push_back(pospart(aff.value(x.data())));
  return out;
}
double ConvexConstraints::violation(const vector<double>& x) {
  return vecSum(violations(x));
}

ConvexConstraints::~ConvexConstraints() {
  if (inModel()) removeFromModel();
}

double ConvexObjective::value(const vector<double>& x)  {
  return quad_.value(x);
}


vector<double> Constraint::violations(const DblVec& x) {
  DblVec val = value(x);
  DblVec out(val.size());

  if (type() == EQ) {
    for (size_t i=0; i < val.size(); ++i) out[i] = fabs(val[i]);
  }
  else { // type() == INEQ
    for (size_t i=0; i < val.size(); ++i) out[i] = pospart(val[i]);
  }

  return out;
}

double Constraint::violation(const DblVec& x) {
  return vecSum(violations(x));
}

OptProb::OptProb() : model_(createModel()) {}

VarVector OptProb::createVariables(const vector<string>& var_names) {
  return createVariables(var_names, DblVec(var_names.size(), -INFINITY), DblVec(var_names.size(), INFINITY));
}
VarVector OptProb::createVariables(const vector<string>& var_names, const DblVec& lb, const DblVec& ub) {
  size_t n_add = var_names.size(), n_cur = vars_.size();
  assert(lb.size() == n_add);
  assert(ub.size() == n_add);
  vars_.reserve(n_cur + n_add);
  lower_bounds_.reserve(n_cur + n_add);
  upper_bounds_.reserve(n_cur + n_add);
  for (size_t i=0; i < var_names.size(); ++i) {
    vars_.push_back(model_->addVar(var_names[i], lb[i], ub[i]));
    lower_bounds_.push_back(lb[i]);
    upper_bounds_.push_back(ub[i]);
  }
  model_->update();
  return VarVector(vars_.end()-n_add, vars_.end());
}
void OptProb::setLowerBounds(const vector<double>& lb) {
  assert(lb.size() == vars_.size());
  lower_bounds_ = lb;
}
void OptProb::setUpperBounds(const vector<double>& ub) {
  assert(ub.size() == vars_.size());
  upper_bounds_ = ub;
}
void OptProb::setLowerBounds(const vector<double>& lb, const vector<Var>& vars) {
  setVec(lower_bounds_, vars, lb);
}
void OptProb::setUpperBounds(const vector<double>& ub, const vector<Var>& vars) {
  setVec(upper_bounds_, vars, ub);
}

void OptProb::addCost(CostPtr cost) {
  costs_.push_back(cost);
}
void OptProb::addConstraint(ConstraintPtr cnt) {
  if (cnt->type() == EQ) addEqConstraint(cnt);
  else addIneqConstraint(cnt);
}
void OptProb::addEqConstraint(ConstraintPtr cnt) {
  assert (cnt->type() == EQ);
  eqcnts_.push_back(cnt);
}
void OptProb::addIneqConstraint(ConstraintPtr cnt) {
  assert (cnt->type() == INEQ);
  ineqcnts_.push_back(cnt);
}
vector<ConstraintPtr> OptProb::getConstraints() const {
  vector<ConstraintPtr> out;
  out.reserve(eqcnts_.size() + ineqcnts_.size());
  out.insert(out.end(), eqcnts_.begin(), eqcnts_.end());
  out.insert(out.end(), ineqcnts_.begin(), ineqcnts_.end());
  return out;
}
void OptProb::addLinearConstraint(const AffExpr& expr, ConstraintType type) {
  if (type == EQ) model_->addEqCnt(expr, "");
  else model_->addIneqCnt(expr, "");
}

vector<double> OptProb::getCentralFeasiblePoint(const vector<double>& x) {
  assert(x.size() == lower_bounds_.size());
  DblVec center(x.size());
  for (size_t i=0; i < x.size(); ++i) center[i] = (lower_bounds_[i] + upper_bounds_[i])/2;
  return getClosestFeasiblePoint(center);
}
vector<double> OptProb::getClosestFeasiblePoint(const vector<double>& x) {
  assert(vars_.size() == x.size());
  QuadExpr obj;
  for (size_t i=0; i < x.size(); ++i) {
    exprInc(obj, exprSquare(exprSub(AffExpr(vars_[i]),x[i])));
  }
  model_->setVarBounds(vars_, lower_bounds_, upper_bounds_);
  model_->setObjective(obj);
  CvxOptStatus status = model_->optimize();
  if(status != CVX_SOLVED) {
    model_->writeToFile("/tmp/fail.lp");
    PRINT_AND_THROW("couldn't find a feasible point. there's probably a problem with variable bounds (e.g. joint limits). wrote to /tmp/fail.lp");
  }
  return model_->getVarValues(vars_);
}

}
