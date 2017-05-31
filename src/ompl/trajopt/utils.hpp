#pragma once
#include "ompl/trajopt/typedefs.hpp"

namespace trajopt {

/**
Extract trajectory array from solution vector x using indices in array vars
*/
TrajArray TRAJOPT_API getTraj(const DblVec& x, const VarArray& vars);
TrajArray TRAJOPT_API getTraj(const DblVec& x, const AffArray& arr);


inline DblVec trajToDblVec(const TrajArray& x) {
  return DblVec(x.data(), x.data()+x.rows()*x.cols());
}

inline VectorXd concat(const VectorXd& a, const VectorXd& b) {
  VectorXd out(a.size()+b.size());
  out.topRows(a.size()) = a;
  out.middleRows(a.size(), b.size()) = b;
  return out;
}

template <typename T>
vector<T> concat(const vector<T>& a, const vector<T>& b) {
  vector<T> out;
  vector<int> x;
  out.insert(out.end(), a.begin(), a.end());
  out.insert(out.end(), b.begin(), b.end());
  return out;
}

template <typename T>
vector<T> singleton(const T& x) {
  return vector<T>(1,x);
}

void TRAJOPT_API AddVarArrays(sco::OptProb& prob, int rows, const vector<int>& cols, const vector<std::string>& name_prefix, const vector<VarArray*>& newvars);

void TRAJOPT_API AddVarArray(sco::OptProb& prob, int rows, int cols, const std::string& name_prefix, VarArray& newvars);

}
