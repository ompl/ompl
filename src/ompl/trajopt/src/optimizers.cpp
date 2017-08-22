/* Authors: Jon Schulman. */

#include <cmath>
#include <cstdio>
#include <boost/foreach.hpp>
#include <boost/format.hpp>

#include "ompl/trajopt/optimizers.h"
#include "ompl/trajopt/modeling.h"
#include "ompl/trajopt/logging.h"
#include "ompl/trajopt/solver_interface.h"
#include "ompl/trajopt/expr_ops.h"
#include "ompl/trajopt/sco_common.h"
#include "ompl/trajopt/stl_to_string.h"
#include "ompl/trajopt/macros.h"

using namespace std;
using namespace util;

namespace sco {

typedef vector<double> DblVec;

std::ostream& operator<<(std::ostream& o, const OptResults& r) {
  o << "Optimization results:" << endl
    << "status: " << statusToString(r.status) << endl
    << "cost values: " << Str(r.cost_vals) << endl
    << "constraint violations: " << Str(r.cnt_viols) << endl
    << "n func evals: " << r.n_func_evals << endl
    << "n qp solves: " << r.n_qp_solves << endl;
  return o;
}

//////////////////////////////////////////////////
////////// private utility functions for  sqp /////////
//////////////////////////////////////////////////

static DblVec evaluateCosts(vector<CostPtr>& costs, const DblVec& x) {
  DblVec out(costs.size());
  for (size_t i=0; i < costs.size(); ++i) {
    out[i] = costs[i]->value(x);
  }
  return out;
}
static DblVec evaluateConstraintViols(vector<ConstraintPtr>& constraints, const DblVec& x) {
    DblVec out(constraints.size());
    for (size_t i=0; i < constraints.size(); ++i) {
      out[i] = constraints[i]->violation(x);
    }
    return out;
}
static vector<ConvexObjectivePtr> convexifyCosts(vector<CostPtr>& costs, const DblVec& x, Model* model) {
    vector<ConvexObjectivePtr> out(costs.size());
    for (size_t i=0; i < costs.size(); ++i) {
      out[i] = costs[i]->convex(x,  model);
    }
    return out;
}
static vector<ConvexConstraintsPtr> convexifyConstraints(vector<ConstraintPtr>& cnts, const DblVec& x, Model* model) {
  vector<ConvexConstraintsPtr> out(cnts.size());
  for (size_t i=0; i < cnts.size(); ++i) {
    out[i] = cnts[i]->convex(x, model);
  }
  return out;
}

DblVec evaluateModelCosts(vector<ConvexObjectivePtr>& costs, const DblVec& x) {
  DblVec out(costs.size());
  for (size_t i=0; i < costs.size(); ++i) {
    out[i] = costs[i]->value(x);
  }
  return out;
}
DblVec evaluateModelCntViols(vector<ConvexConstraintsPtr>& cnts, const DblVec& x) {
  DblVec out(cnts.size());
  for (size_t i=0; i < cnts.size(); ++i) {
    out[i] = cnts[i]->violation(x);
  }
  return out;
}

static vector<string> getCostNames(const vector<CostPtr>& costs) {
  vector<string> out(costs.size());
  for (size_t i=0; i < costs.size(); ++i) out[i] = costs[i]->name();
  return out;
}
static vector<string> getCntNames(const vector<ConstraintPtr>& cnts) {
  vector<string> out(cnts.size());
  for (size_t i=0; i < cnts.size(); ++i) out[i] = cnts[i]->name();
  return out;
}

void printCostInfo(const vector<double>& old_cost_vals, const vector<double>& model_cost_vals, const vector<double>& new_cost_vals,
                  const vector<double>& old_cnt_vals, const vector<double>& model_cnt_vals, const vector<double>& new_cnt_vals,
    const vector<string>& cost_names, const vector<string>& cnt_names, double merit_coeff) {
    printf("%15s | %10s | %10s | %10s | %10s\n", "", "oldexact", "dapprox", "dexact", "ratio");
    printf("%15s | %10s---%10s---%10s---%10s\n", "COSTS", "----------", "----------", "----------", "----------");
    for (size_t i=0; i < old_cost_vals.size(); ++i) {
      double approx_improve = old_cost_vals[i] - model_cost_vals[i];
      double exact_improve = old_cost_vals[i] - new_cost_vals[i];
      if (fabs(approx_improve) > 1e-8)
        printf("%15s | %10.3e | %10.3e | %10.3e | %10.3e\n", cost_names[i].c_str(), old_cost_vals[i], approx_improve, exact_improve, exact_improve/approx_improve);
      else
        printf("%15s | %10.3e | %10.3e | %10.3e | %10s\n",cost_names[i].c_str(), old_cost_vals[i], approx_improve, exact_improve, "  ------  ");
    }
    if (cnt_names.size() == 0) return;
    printf("%15s | %10s---%10s---%10s---%10s\n", "CONSTRAINTS", "----------", "----------", "----------", "----------");
    for (size_t i=0; i < old_cnt_vals.size(); ++i) {
      double approx_improve = old_cnt_vals[i] - model_cnt_vals[i];
      double exact_improve = old_cnt_vals[i] - new_cnt_vals[i];
      if (fabs(approx_improve) > 1e-8)
        printf("%15s | %10.3e | %10.3e | %10.3e | %10.3e\n", cnt_names[i].c_str(), merit_coeff*old_cnt_vals[i], merit_coeff*approx_improve, merit_coeff*exact_improve, exact_improve/approx_improve);
      else
        printf("%15s | %10.3e | %10.3e | %10.3e | %10s\n", cnt_names[i].c_str(), merit_coeff*old_cnt_vals[i], merit_coeff*approx_improve, merit_coeff*exact_improve, "  ------  ");
    }

}

// todo: use different coeffs for each constraint
vector<ConvexObjectivePtr> cntsToCosts(const vector<ConvexConstraintsPtr>& cnts, double err_coeff, Model* model) {
  vector<ConvexObjectivePtr> out;
  BOOST_FOREACH(const ConvexConstraintsPtr& cnt, cnts) {
    ConvexObjectivePtr obj(new ConvexObjective(model));
    BOOST_FOREACH(const AffExpr& aff, cnt->eqs_) {
      obj->addAbs(aff, err_coeff);
    }
    BOOST_FOREACH(const AffExpr& aff, cnt->ineqs_) {
      obj->addHinge(aff, err_coeff);
    }
    out.push_back(obj);
  }
  return out;
}

void Optimizer::addCallback(const Callback& cb) {
  callbacks_.push_back(cb);
}
void Optimizer::callCallbacks(DblVec& x) {
  for (size_t i=0; i < callbacks_.size(); ++i) {
    callbacks_[i](prob_.get(), x);
  }
}

void Optimizer::initialize(const vector<double>& x) {
  if (!prob_) PRINT_AND_THROW("need to set the problem before initializing");
  if (prob_->getVars().size() != x.size())
    PRINT_AND_THROW(boost::format("initialization vector has wrong length. expected %i got %i")%prob_->getVars().size()%x.size());
  results_.clear();
  results_.x = x;
}

BasicTrustRegionSQP::BasicTrustRegionSQP() {
  initParameters();
}
BasicTrustRegionSQP::BasicTrustRegionSQP(OptProbPtr prob) {
  initParameters();
  setProblem(prob);
}

// TODO: All these are initialized in hpp, get rid of this function.
void BasicTrustRegionSQP::initParameters() {
    improve_ratio_threshold_ = .25;
    minTrustBoxSize_ = 1e-4;
    minApproxImprove_= 1e-4;
    minApproxImproveFrac_ = -INFINITY;
    maxIter_ = 50;
    trust_shrink_ratio_=.1;
    trust_expand_ratio_ = 1.5;
    cnt_tolerance_ = 1e-4;
    max_merit_coeff_increases_ = 5;
    merit_coeff_increase_ratio_ = 10;
    max_time_ = INFINITY;

    merit_error_coeff_ = 10;
    trust_box_size_ = 1e-1;
}

void BasicTrustRegionSQP::setProblem(OptProbPtr prob) {
  Optimizer::setProblem(prob);
  model_ = prob->getModel();
}

void BasicTrustRegionSQP::adjustTrustRegion(double ratio) {
  trust_box_size_ *= ratio;
}

void BasicTrustRegionSQP::setTrustBoxConstraints(const DblVec& x) {
  vector<Var>& vars = prob_->getVars();
  assert(vars.size() == x.size());
  DblVec& lb=prob_->getLowerBounds(), ub=prob_->getUpperBounds();
  DblVec lbtrust(x.size()), ubtrust(x.size());
  for (size_t i=0; i < x.size(); ++i) {
    lbtrust[i] = fmax(x[i] - trust_box_size_, lb[i]);
    ubtrust[i] = fmin(x[i] + trust_box_size_, ub[i]);
  }
  model_->setVarBounds(vars, lbtrust, ubtrust);
}

OptStatus BasicTrustRegionSQP::optimize() {
  vector<string> cost_names = getCostNames(prob_->getCosts());
  vector<ConstraintPtr> constraints = prob_->getConstraints();
  vector<string> cnt_names = getCntNames(constraints);

  DblVec& x_ = results_.x; // just so I don't have to rewrite code
  if (x_.size() == 0) PRINT_AND_THROW("you forgot to initialize!");
  if (!prob_) PRINT_AND_THROW("you forgot to set the optimization problem");

  x_ = prob_->getClosestFeasiblePoint(x_);

  assert(x_.size() == prob_->getVars().size());
  assert(prob_->getCosts().size() > 0 || constraints.size() > 0);

  OptStatus retval = INVALID;

  /* merit adjustment loop */
  for (int merit_increases=0; merit_increases < max_merit_coeff_increases_; ++merit_increases) {
    for (int iter = 1; ; ++iter) { /* sqp loop */
      callCallbacks(x_);

      LOG_DEBUG("current iterate: %s", CSTR(x_));
      LOG_INFO("iteration %i", iter);

      // speedup: if you just evaluated the cost when doing the line search, use that
      if (results_.cost_vals.empty() && results_.cnt_viols.empty()) { //only happens on the first iteration
        results_.cnt_viols = evaluateConstraintViols(constraints, x_);
        results_.cost_vals = evaluateCosts(prob_->getCosts(), x_);
        assert(results_.n_func_evals == 0);
        ++results_.n_func_evals;
      }

      vector<ConvexObjectivePtr> cost_models = convexifyCosts(prob_->getCosts(),x_, model_.get());
      vector<ConvexConstraintsPtr> cnt_models = convexifyConstraints(constraints, x_, model_.get());
      vector<ConvexObjectivePtr> cnt_cost_models = cntsToCosts(cnt_models, merit_error_coeff_, model_.get());
      model_->update();
      BOOST_FOREACH(ConvexObjectivePtr& cost, cost_models)cost->addConstraintsToModel();
      BOOST_FOREACH(ConvexObjectivePtr& cost, cnt_cost_models)cost->addConstraintsToModel();
      model_->update();
      QuadExpr objective;
      BOOST_FOREACH(ConvexObjectivePtr& co, cost_models)exprInc(objective, co->quad_);
      BOOST_FOREACH(ConvexObjectivePtr& co, cnt_cost_models){
        exprInc(objective, co->quad_);
      }

      model_->setObjective(objective);

      while (trust_box_size_ >= minTrustBoxSize_) {

        setTrustBoxConstraints(x_);
        CvxOptStatus status = model_->optimize();
        ++results_.n_qp_solves;
        if (status != CVX_SOLVED) {
          LOG_ERROR("convex solver failed! set TRAJOPT_LOG_THRESH=DEBUG to see solver output. saving model to /tmp/fail.lp and IIS to /tmp/fail.ilp");
          model_->writeToFile("/tmp/fail.lp");
          model_->writeToFile("/tmp/fail.ilp");
          return cleanup(OPT_FAILED, x_);
        }
        DblVec model_var_vals = model_->getVarValues(model_->getVars());

        DblVec model_cost_vals = evaluateModelCosts(cost_models, model_var_vals);
        DblVec model_cnt_viols = evaluateModelCntViols(cnt_models, model_var_vals);

        // the n variables of the OptProb happen to be the first n variables in the Model
        DblVec new_x(model_var_vals.begin(), model_var_vals.begin() + x_.size());

        if (GetLogLevel() >= util::LevelDebug) {
          DblVec cnt_costs1 = evaluateModelCosts(cnt_cost_models, model_var_vals);
          DblVec cnt_costs2 = model_cnt_viols;
          for (size_t i=0; i < cnt_costs2.size(); ++i) cnt_costs2[i] *= merit_error_coeff_;
          LOG_DEBUG("SHOULD BE ALMOST THE SAME: %s ?= %s", CSTR(cnt_costs1), CSTR(cnt_costs2) );
          // not exactly the same because cnt_costs1 is based on aux variables, but they might not be at EXACTLY the right value
        }

        DblVec new_cost_vals = evaluateCosts(prob_->getCosts(), new_x);
        DblVec new_cnt_viols = evaluateConstraintViols(constraints, new_x);
        ++results_.n_func_evals;

        double old_merit = vecSum(results_.cost_vals) + merit_error_coeff_ * vecSum(results_.cnt_viols);
        double model_merit = vecSum(model_cost_vals) + merit_error_coeff_ * vecSum(model_cnt_viols);
        double new_merit = vecSum(new_cost_vals) + merit_error_coeff_ * vecSum(new_cnt_viols);
        double approx_merit_improve = old_merit - model_merit;
        double exact_merit_improve = old_merit - new_merit;
        double merit_improve_ratio = exact_merit_improve / approx_merit_improve;

        if (util::GetLogLevel() >= util::LevelInfo) {
          LOG_INFO(" ");
          printCostInfo(results_.cost_vals, model_cost_vals, new_cost_vals,
                        results_.cnt_viols, model_cnt_viols, new_cnt_viols, cost_names,
                        cnt_names, merit_error_coeff_);
          printf("%15s | %10.3e | %10.3e | %10.3e | %10.3e\n", "TOTAL", old_merit, approx_merit_improve, exact_merit_improve, merit_improve_ratio);
        }

        if (approx_merit_improve < -1e-5) {
          LOG_ERROR("approximate merit function got worse (%.3e). (convexification is probably wrong to zeroth order)", approx_merit_improve);
        }
        if (approx_merit_improve < minApproxImprove_) {
          LOG_INFO("converged because improvement was small (%.3e < %.3e)", approx_merit_improve, minApproxImprove_);
          retval = OPT_CONVERGED;
          goto penaltyadjustment;
        }
        if (approx_merit_improve / old_merit < minApproxImproveFrac_) {
          LOG_INFO(
              "converged because improvement ratio was small (%.3e < %.3e)",
              approx_merit_improve/old_merit, minApproxImproveFrac_);
          retval = OPT_CONVERGED;
          goto penaltyadjustment;
        }
        else if (exact_merit_improve < 0 || merit_improve_ratio < improve_ratio_threshold_) {
          adjustTrustRegion(trust_shrink_ratio_);
          LOG_INFO("shrunk trust region. new box size: %.4f",
              trust_box_size_);
        } else {
          x_ = new_x;
          results_.cost_vals = new_cost_vals;
          results_.cnt_viols = new_cnt_viols;
          adjustTrustRegion(trust_expand_ratio_);
          LOG_INFO("expanded trust region. niterew box size: %.4f",trust_box_size_);
          break;
        }
      }

      if (trust_box_size_ < minTrustBoxSize_) {
        LOG_INFO("converged because trust region is tiny");
        retval = OPT_CONVERGED;
        goto penaltyadjustment;
      } else if (iter >= maxIter_) {
        LOG_INFO("iteration limit: iter %d, maxIter_ %f", iter, maxIter_);
        return cleanup(OPT_SCO_ITERATION_LIMIT, x_);
      }
    }

    penaltyadjustment:
    if (results_.cnt_viols.empty() || vecMax(results_.cnt_viols) < cnt_tolerance_) {
      if (results_.cnt_viols.size() > 0) {
          LOG_INFO("woo-hoo! constraints are satisfied (to tolerance %.2e)", cnt_tolerance_);
      }
      return cleanup(retval, x_);
    } else {
      LOG_INFO("not all constraints are satisfied. increasing penalties");
      merit_error_coeff_ *= merit_coeff_increase_ratio_;
      trust_box_size_ = fmax(trust_box_size_, minTrustBoxSize_ / trust_shrink_ratio_ * 1.5);
    }
  }
  LOG_INFO("optimization couldn't satisfy all constraints");
  return cleanup(OPT_PENALTY_ITERATION_LIMIT, x_);
}

OptStatus BasicTrustRegionSQP::cleanup(OptStatus retval, DblVec& x_) {
  assert(retval != INVALID && "should never happen");
  results_.status = retval;
  results_.total_cost = vecSum(results_.cost_vals);
  LOG_INFO("\n==================\n%s==================", CSTR(results_));
  callCallbacks(x_);

  return retval;
}

}
