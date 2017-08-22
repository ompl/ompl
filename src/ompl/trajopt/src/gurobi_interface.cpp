
#include <boost/foreach.h>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "ompl/trajopt/solver_interface.h"
#include "ompl/trajopt/gurobi_interface.h"
#include "ompl/trajopt/logging.h"
extern "C" {
#include "ompl/trajopt/gurobi_c.h"
}
#include "ompl/trajopt/sco_common.h"
#include "ompl/trajopt/macros.h"
#include "ompl/trajopt/stl_to_string.h"

using namespace std;

namespace sco {


extern void simplify2(vector<int>& inds, vector<double>& vals);
extern vector<int> vars2inds(const vector<Var>& vars);
extern vector<int> cnts2inds(const vector<Cnt>& cnts);

GRBenv* gEnv;


#if 0
void simplify(vector<int>& inds, vector<double>& vals) {
  // first find the largest element of inds
  // make an array of that size
  //

  assert(inds.size() == vals.size());
  int min_ind = *std::min_element(inds.begin(), inds.end());
  int max_ind = *std::max_element(inds.begin(), inds.end());
  int orig_size = inds.size();
  int work_size = max_ind - min_ind + 1;
  vector<double> work_vals(work_size, 0);

  for (int i=0; i < orig_size; ++i) {
    work_vals[inds[i] - min_ind] += vals[i];
  }


  int i_new = 0;
  for (int i=0; i < work_size; ++i) {
    if (work_vals[i] != 0) {
      vals[i_new] = work_vals[i];
      inds[i_new] = i + min_ind;
      ++i_new;
    }
  }

  inds.resize(i_new);
  vals.resize(i_new);

}
#endif

#define ENSURE_SUCCESS(expr) do {\
    bool error = expr;\
    if (error) {\
      printf("GRB error: %s while evaluating %s at %s:%i\n", GRBgeterrormsg(gEnv), #expr, __FILE__,__LINE__);\
      abort();\
    }\
} while(0)

ModelPtr createGurobiModel() {
  ModelPtr out(new GurobiModel());
  return out;
}

GurobiModel::GurobiModel() {
  if (!gEnv) {
    GRBloadenv(&gEnv, NULL);
    if (util::GetLogLevel() < util::LevelDebug) {
      ENSURE_SUCCESS(GRBsetintparam(gEnv, "OutputFlag",0));
    }
  }
  GRBnewmodel(gEnv, &m_model,"problem",0, NULL, NULL, NULL,NULL, NULL);
}

Var GurobiModel::addVar(const string& name) {
  ENSURE_SUCCESS(GRBaddvar(m_model, 0, NULL, NULL, 0, -GRB_INFINITY, GRB_INFINITY, GRB_CONTINUOUS, const_cast<char*>(name.c_str())));
  m_vars.push_back(new VarRep(m_vars.size(), name, this));
  return m_vars.back();
}

Var GurobiModel::addVar(const string& name, double lb, double ub) {
  ENSURE_SUCCESS(GRBaddvar(m_model, 0, NULL, NULL, 0, lb, ub, GRB_CONTINUOUS, const_cast<char*>(name.c_str())));
  m_vars.push_back(new VarRep(m_vars.size(), name, this));
  return m_vars.back();
}


Cnt GurobiModel::addEqCnt(const AffExpr& expr, const string& name) {
  LOG_TRACE("adding eq constraint: %s = 0", CSTR(expr));
  vector<int> inds = vars2inds(expr.vars);
  vector<double> vals = expr.coeffs;
  simplify2(inds, vals);
  ENSURE_SUCCESS(GRBaddconstr(m_model, inds.size(),
       const_cast<int*>(inds.data()), const_cast<double*>(vals.data()), GRB_EQUAL, -expr.constant, const_cast<char*>(name.c_str())));
  m_cnts.push_back(new CntRep(m_cnts.size(), this));
  return m_cnts.back();
}
Cnt GurobiModel::addIneqCnt(const AffExpr& expr, const string& name) {
  LOG_TRACE("adding ineq: %s <= 0", CSTR(expr));
  vector<int> inds = vars2inds(expr.vars);
  vector<double> vals = expr.coeffs;
  simplify2(inds, vals);
  ENSURE_SUCCESS(GRBaddconstr(m_model, inds.size(),
      inds.data(), vals.data(), GRB_LESS_EQUAL, -expr.constant, const_cast<char*>(name.c_str())));
  m_cnts.push_back(new CntRep(m_cnts.size(), this));
  return m_cnts.back();
}
Cnt GurobiModel::addIneqCnt(const QuadExpr& qexpr, const string& name) {
  int numlnz = qexpr.affexpr.size();
  vector<int> linds = vars2inds(qexpr.affexpr.vars);
  vector<double> lvals = qexpr.affexpr.coeffs;
  vector<int> inds1 = vars2inds(qexpr.vars1);
  vector<int> inds2 = vars2inds(qexpr.vars2);
  ENSURE_SUCCESS(GRBaddqconstr(m_model, numlnz, linds.data(), lvals.data(), qexpr.size(),
    inds1.data(), inds2.data(), const_cast<double*>(qexpr.coeffs.data()),
    GRB_LESS_EQUAL, -qexpr.affexpr.constant, const_cast<char*>(name.c_str())));
  return Cnt();
}

void resetIndices(VarVector& vars) {
  for (size_t i=0; i < vars.size(); ++i) vars[i].var_rep[i].index = i;
}
void resetIndices(vector<Cnt>& cnts) {
  for (size_t i=0; i < cnts.size(); ++i) cnts[i].cnt_rep[i].index = i;
}

void GurobiModel::removeVars(const vector<Var>& vars) {
  vector<int>inds = vars2inds(vars);
  ENSURE_SUCCESS(GRBdelvars(m_model, inds.size(), inds.data()));
  for (int i=0; i < vars.size(); ++i) vars[i].var_rep->removed = true;
}

void GurobiModel::removeCnts(const vector<Cnt>& cnts) {
  vector<int>inds = cnts2inds(cnts);
  ENSURE_SUCCESS(GRBdelconstrs(m_model, inds.size(), inds.data()));
  for (int i=0; i < cnts.size(); ++i) cnts[i].cnt_rep->removed = true;
}

#if 0
void GurobiModel::setVarBounds(const Var& var, double lower, double upper) {
  assert(var.var_rep->creator == this);
  ENSURE_SUCCESS(GRBsetdblattrelement(m_model, GRB_DBL_ATTR_LB, var.var_rep->index, lower));
  ENSURE_SUCCESS(GRBsetdblattrelement(m_model, GRB_DBL_ATTR_UB, var.var_rep->index, upper));
}
#endif


void GurobiModel::setVarBounds(const VarVector& vars, const vector<double>& lower, const vector<double>& upper) {
  assert(vars.size() == lower.size() && vars.size() == upper.size());
  vector<int> inds = vars2inds(vars);
  ENSURE_SUCCESS(GRBsetdblattrlist(m_model, GRB_DBL_ATTR_LB, inds.size(), inds.data(), const_cast<double*>(lower.data())));
  ENSURE_SUCCESS(GRBsetdblattrlist(m_model, GRB_DBL_ATTR_UB, inds.size(), inds.data(), const_cast<double*>(upper.data())));
}

#if 0
double GurobiModel::getVarValue(const Var& var) const {
  assert(var.var_rep->creator == this);
  double out;
  ENSURE_SUCCESS(GRBgetdblattrelement(m_model, GRB_DBL_ATTR_X, var.var_rep->index, &out));
  return out;
}
#endif

vector<double> GurobiModel::getVarValues(const vector<Var>& vars) const {
  assert((vars.size() == 0) || (vars[0].var_rep->creator == this));
  vector<int> inds = vars2inds(vars);
  vector<double> out(inds.size());
  ENSURE_SUCCESS(GRBgetdblattrlist(m_model, GRB_DBL_ATTR_X, inds.size(), inds.data(), out.data()));
  return out;
}

CvxOptStatus GurobiModel::optimize(){
  ENSURE_SUCCESS(GRBoptimize(m_model));
  int status;
  GRBgetintattr(m_model, GRB_INT_ATTR_STATUS, &status);
  if (status == GRB_OPTIMAL) {
    double objval; GRBgetdblattr(m_model, GRB_DBL_ATTR_OBJVAL, &objval);
    LOG_DEBUG("solver objective value: %.3e", objval);
    return CVX_SOLVED;
  }
  else if (status == GRB_INFEASIBLE) {
    GRBcomputeIIS(m_model);
    return CVX_INFEASIBLE;
  }
  else return CVX_FAILED;
}
CvxOptStatus GurobiModel::optimizeFeasRelax(){
  double lbpen=GRB_INFINITY, ubpen = GRB_INFINITY,  rhspen=1;
  ENSURE_SUCCESS(GRBfeasrelax(m_model, 0/*sum of viol*/, 0/*just minimize cost of viol*/, &lbpen, &ubpen, &rhspen, NULL));
  return optimize();
}

void GurobiModel::setObjective(const AffExpr& expr) {
  GRBdelq(m_model);

  int nvars;
  GRBgetintattr(m_model, GRB_INT_ATTR_NUMVARS, &nvars);
  assert(nvars == m_vars.size());

  vector<double> obj(nvars, 0);
  for (size_t i=0; i < expr.size(); ++i) {
    obj[expr.vars[i].var_rep->index] += expr.coeffs[i];
  }
  ENSURE_SUCCESS(GRBsetdblattrarray(m_model, "Obj", 0, nvars, obj.data()));
  GRBsetdblattr(m_model, "ObjCon", expr.constant);
}

void GurobiModel::setObjective(const QuadExpr& quad_expr) {
  setObjective(quad_expr.affexpr);
  vector<int> inds1 = vars2inds(quad_expr.vars1);
  vector<int> inds2 = vars2inds(quad_expr.vars2);
  GRBaddqpterms(m_model, quad_expr.coeffs.size(), const_cast<int*>(inds1.data()),
      const_cast<int*>(inds2.data()), const_cast<double*>(quad_expr.coeffs.data()));
}

void GurobiModel::writeToFile(const string& fname) {
  ENSURE_SUCCESS(GRBwrite(m_model, fname.c_str()));
}


void GurobiModel::update() {
  ENSURE_SUCCESS(GRBupdatemodel(m_model));

  {
  int inew = 0;
  BOOST_FOREACH(const Var& var, m_vars) {
    if (!var.var_rep->removed) {
      m_vars[inew] = var;
      var.var_rep->index = inew;
      ++inew;
    }
    else delete var.var_rep;
  }
  m_vars.resize(inew);
  }
  {
  int inew = 0;
  BOOST_FOREACH(const Cnt& cnt, m_cnts) {
    if (!cnt.cnt_rep->removed) {
      m_cnts[inew] = cnt;
      cnt.cnt_rep->index = inew;
      ++inew;
    }
    else delete cnt.cnt_rep;
  }
  m_cnts.resize(inew);
  }
}

VarVector GurobiModel::getVars() const {
  return m_vars;
}

GurobiModel::~GurobiModel() {
  ENSURE_SUCCESS(GRBfreemodel(m_model));
}

}
