#pragma once

/*
 * Model a non-convex optimization problem by defining Cost and Constraint objects
 * which know how to generate a convex approximation
 */

#include <vector>
#include <boost/shared_ptr.hpp>
#include "ompl/trajopt/sco_fwd.hpp"
#include "ompl/trajopt/solver_interface.hpp"

namespace sco {

using std::vector;

/**
Stores convex terms in a objective
For non-quadratic terms like hinge(x) and abs(x), it needs to add auxilliary variables and linear constraints to the model
Note: When this object is deleted, the constraints and variables it added to the model are removed
 */
class ConvexObjective {
public:
  /** \brief Constructor: binds the model to this objective so that this objective can be removed
      from the model when deleted. */
  ConvexObjective(Model* model) : model_(model) {}
  void addAffExpr(const AffExpr&);
  void addQuadExpr(const QuadExpr&);
  void addHinge(const AffExpr&, double coeff);
  void addAbs(const AffExpr&, double coeff);
  void addHinges(const AffExprVector&);
  void addL1Norm(const AffExprVector&);
  void addL2Norm(const AffExprVector&);
  void addMax(const AffExprVector&);

  bool inModel() {
    return model_ != NULL;
  }
  void addConstraintsToModel();
  void removeFromModel();
  double value(const vector<double>& x);

  ~ConvexObjective();


  Model* model_;
  QuadExpr quad_;
  vector<Var> vars_;
  vector<AffExpr> eqs_;
  vector<AffExpr> ineqs_;
  vector<Cnt> cnts_;
private:
  ConvexObjective()  {}
  ConvexObjective(ConvexObjective&)  {}
};

/**
Stores convex inequality constraints and affine equality constraints.
Actually only affine inequality constraints are currently implemented.
*/
class ConvexConstraints {
public:
  ConvexConstraints(Model* model) : model_(model) {}
  /** Expression that should == 0 */
  void addEqCnt(const AffExpr&);
  /** Expression that should <= 0 */
  void addIneqCnt(const AffExpr&);
  void setModel(Model* model) {
    assert(!inModel());
    model_ = model;
  }
  bool inModel() {
    return model_ != NULL;
  }
  void addConstraintsToModel();
  void removeFromModel();

  vector<double> violations(const vector<double>& x);
  double violation(const vector<double>& x);

  ~ConvexConstraints();
  vector<AffExpr> eqs_;
  vector<AffExpr> ineqs_;
private:
   Model* model_;
   vector<Cnt> cnts_;
   ConvexConstraints() : model_(NULL) {}
   ConvexConstraints(ConvexConstraints&) {}
};

/**
Non-convex cost function, which knows how to calculate its convex approximation (convexify() method)
*/
class Cost {
public:
  /** Evaluate at solution vector x*/
  virtual double value(const vector<double>&) = 0;
  /** Convexify at solution vector x (x is only used for numerical costs functions) */
  virtual ConvexObjectivePtr convex(const vector<double>& x, Model* model) = 0;
  /** Get problem variables associated with this cost */
  virtual VarVector getVars() {return VarVector();}

  string name() {return name_;}
  void setName(const string& name) {name_=name;}
  Cost() : name_("unnamed") {}
  Cost(const string& name) : name_(name) {}
  virtual ~Cost() {}
protected:
  string name_;
};

/**
Non-convex vector-valued constraint function, which knows how to calculate its convex approximation
*/
class Constraint {
public:

  /** inequality vs equality */
  virtual ConstraintType type() = 0;
  /** Evaluate at solution vector x*/
  virtual vector<double> value(const vector<double>& x) = 0;
  /** Convexify at solution vector x (x is only used for numerical constraints)*/
  virtual ConvexConstraintsPtr convex(const vector<double>& x, Model* model) = 0;
  /** Calculate constraint violations (positive part for inequality constraint,
      absolute value for inequality constraint)*/
  vector<double> violations(const vector<double>& x);
  /** Sum of violations */
  double violation(const vector<double>& x);
  /** Get problem variables associated with this constraint */
  virtual VarVector getVars() {return VarVector();}

  string name() {return name_;}
  void setName(const string& name) {name_=name;}
  Constraint() : name_("unnamed") {}
  Constraint(const string& name) : name_(name) {}
  virtual ~Constraint() {}

protected:
  string name_;
};

class EqConstraint : public Constraint{
public:
  ConstraintType type() {return EQ;}
};

class IneqConstraint : public Constraint {
public:
  ConstraintType type() {return INEQ;}
};

/**
Non-convex optimization problem
*/
class OptProb {
public:
  OptProb();
  /** create variables with bounds [-INFINITY, INFINITY]  */
  VarVector createVariables(const vector<string>& names);
  /** create variables with bounds [lb[i], ub[i] */
  VarVector createVariables(const vector<string>& names, const vector<double>& lb, const vector<double>& ub);
  /** set the lower bounds of all the variables */
  void setLowerBounds(const vector<double>& lb);
  /** set the upper bounds of all the variables */
  void setUpperBounds(const vector<double>& ub);
  /** set lower bounds of some of the variables */
  void setLowerBounds(const vector<double>& lb, const vector<Var>& vars);
  /** set upper bounds of some of the variables */
  void setUpperBounds(const vector<double>& ub, const vector<Var>& vars);
  /** Note: in the current implementation, this function just adds the constraint to the
   * model. So if you're not careful, you might end up with an infeasible problem. */
  void addLinearConstraint(const AffExpr&, ConstraintType type);
  /** Add nonlinear cost function */
  void addCost(CostPtr);
  /** Add nonlinear constraint function */
  void addConstraint(ConstraintPtr);
  void addEqConstraint(ConstraintPtr);
  void addIneqConstraint(ConstraintPtr);
  virtual ~OptProb() {}
  /** Find closest point to solution vector x that satisfies linear inequality constraints */
  vector<double> getCentralFeasiblePoint(const vector<double>& x);
  vector<double> getClosestFeasiblePoint(const vector<double>& x);

  vector<ConstraintPtr> getConstraints() const;
  vector<CostPtr>& getCosts() {return costs_;}
  vector<ConstraintPtr>& getIneqConstraints() {return ineqcnts_;}
  vector<ConstraintPtr>& getEqConstraints() {return eqcnts_;}
  DblVec& getLowerBounds() {return lower_bounds_;}
  DblVec& getUpperBounds() {return upper_bounds_;}
  ModelPtr getModel() {return model_;}
  vector<Var>& getVars() {return vars_;}
  int getNumCosts() {return costs_.size();}
  int getNumConstraints() {return eqcnts_.size() + ineqcnts_.size();}
  int getNumVars() {return vars_.size();}

protected:
  ModelPtr model_;
  vector<Var> vars_;
  vector<double> lower_bounds_;
  vector<double> upper_bounds_;
  vector<CostPtr> costs_;
  vector<ConstraintPtr> eqcnts_;
  vector<ConstraintPtr> ineqcnts_;

  OptProb(OptProb&);
};

template <typename VecType>
inline void setVec(DblVec& x, const VarVector& vars, const VecType& vals) {
  assert(vars.size() == vals.size());
  for (size_t i = 0; i < vars.size(); ++i) {
    x[vars[i].var_rep->index] = vals[i];
  }
}

template <typename OutVecType>
inline OutVecType getVec1(const vector<double>& x, const VarVector& vars) {
  OutVecType out(vars.size());
  for (size_t i=0; i < vars.size(); ++i) out[i] = x[vars[i].var_rep->index];
  return out;
}
}
