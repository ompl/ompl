#include "ompl/trajopt/solver_interface.hpp"
#include <iostream>
#include "ompl/trajopt/macros.h"
#include <boost/format.hpp>
#include <sstream>
#include <map>
#include <boost/foreach.hpp>
using namespace std;

namespace sco {

vector<int> vars2inds(const vector<Var>& vars) {
  vector<int> inds(vars.size());
  for (size_t i=0; i < inds.size(); ++i) inds[i] = vars[i].var_rep->index;
  return inds;
}
vector<int> cnts2inds(const vector<Cnt>& cnts) {
  vector<int> inds(cnts.size());
  for (size_t i=0; i < inds.size(); ++i) inds[i] = cnts[i].cnt_rep->index;
  return inds;
}

void simplify2(vector<int>& inds, vector<double>& vals) {
  typedef std::map<int, double> Int2Double;
  Int2Double ind2val;
  for (unsigned i=0; i < inds.size(); ++i) {
    if (vals[i] != 0) ind2val[inds[i]] += vals[i];
  }
  inds.resize(ind2val.size());
  vals.resize(ind2val.size());
  int i_new = 0;
  BOOST_FOREACH(Int2Double::value_type& iv, ind2val) {
    inds[i_new] = iv.first;
    vals[i_new] = iv.second;
    ++i_new;
  }
}


double AffExpr::value(const double* x) const {
  double out = constant;
  for (size_t i=0; i < size(); ++i) {
    out += coeffs[i] * vars[i].value(x);
  }
  return out;
}
double AffExpr::value(const vector<double>& x) const {
  double out = constant;
  for (size_t i=0; i < size(); ++i) {
    out += coeffs[i] * vars[i].value(x);
  }
  return out;
}
double QuadExpr::value(const vector<double>& x) const {
  double out = affexpr.value(x);
  for (size_t i=0; i < size(); ++i) {
    out += coeffs[i] * vars1[i].value(x) * vars2[i].value(x);
  }
  return out;
}
double QuadExpr::value(const double* x) const {
  double out = affexpr.value(x);
  for (size_t i=0; i < size(); ++i) {
    out += coeffs[i] * vars1[i].value(x) * vars2[i].value(x);
  }
  return out;
}


Var Model::addVar(const string& name, double lb, double ub) {
  Var v = addVar(name);
  setVarBounds(v, lb, ub);
  return v;
}
void Model::removeVar(const Var& var) {
  vector<Var> vars(1,var);
  removeVars(vars);
}
void Model::removeCnt(const Cnt& cnt) {
  vector<Cnt> cnts(1, cnt);
  removeCnts(cnts);
}

double Model::getVarValue(const Var& var) const {
  VarVector vars(1,var);
  return getVarValues(vars)[0];
}

void Model::setVarBounds(const Var& var, double lower, double upper) {
  vector<double> lowers(1,lower), uppers(1, upper);
  vector<Var> vars(1,var);
  setVarBounds(vars, lowers, uppers);
}


ostream& operator<<(ostream& o, const Var& v) {
  if (v.var_rep != NULL)
    o << v.var_rep->name;
  else
    o << "nullvar";
  return o;
}
ostream& operator<<(ostream& o, const Cnt& c) {
  o << c.cnt_rep->expr << ((c.cnt_rep->type == EQ) ? " == 0" : " <= 0");
  return o;
}
ostream& operator<<(ostream& o, const AffExpr& e) {
  o << e.constant;
  for (size_t i=0; i < e.size(); ++i) {
    o << " + " << e.coeffs[i] << "*" << e.vars[i];
  }
  return o;
}
ostream& operator<<(ostream& o, const QuadExpr& e) {
  o << e.affexpr;
  for (size_t i=0; i < e.size(); ++i) {
    o << " + " << e.coeffs[i] << "*" << e.vars1[i] << "*" << e.vars2[i];
  }
  return o;
}



ModelPtr createModel() {

#ifdef HAVE_GUROBI
  extern ModelPtr createGurobiModel();
#endif
#ifdef HAVE_BPMPD
  extern ModelPtr createBPMPDModel();
#endif

  enum ConvexSolver {
    GUROBI,
    BPMPD,
    INVALID
  };


  char* solver_env = getenv("TRAJOPT_CONVEX_SOLVER");


  ConvexSolver solver = INVALID;

  if (solver_env) {
    if (string(solver_env) == "GUROBI") solver = GUROBI;
    else if (string(solver_env) == "BPMPD") solver = BPMPD;
    else PRINT_AND_THROW( boost::format("invalid solver \"%s\"specified by TRAJOPT_CONVEX_SOLVER")%solver_env);
#ifndef HAVE_GUROBI
    if (solver == GUROBI) PRINT_AND_THROW("you didn't build with GUROBI support");
#endif
#ifndef HAVE_BPMPD
    if (solver == BPMPD) PRINT_AND_THROW("you don't have BPMPD support on this platform");
#endif

  }
  else {
#ifdef HAVE_GUROBI
  solver = GUROBI;
#else
  solver = BPMPD;
#endif
  }

#ifdef HAVE_GUROBI
  if (solver == GUROBI) return createGurobiModel();
#endif
#ifdef HAVE_BPMPD
  if (solver == BPMPD) return createBPMPDModel();
#endif
  PRINT_AND_THROW("Failed to create solver");
  return ModelPtr();

}



}
