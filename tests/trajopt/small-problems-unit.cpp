#define BOOST_TEST_MODULE "SQP"
#include <boost/test/unit_test.hpp>
#include <boost/test/included/unit_test.hpp>

#include <cmath>
#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>
#include <iostream>
#include <Eigen/Dense>

#include "ompl/trajopt/optimizers.h"
#include "ompl/trajopt/solver_interface.h"
#include "ompl/trajopt/expr_op_overloads.h"
#include "ompl/trajopt/modeling_utils.h"
#include "ompl/trajopt/sco_common.h"
#include "ompl/trajopt/stl_to_string.h"
#include "ompl/trajopt/logging.h"

void setupProblem(sco::OptProbPtr& probptr, size_t nvars) {
    probptr.reset(new sco::OptProb());
    std::vector<std::string> var_names;
    for (size_t i=0; i < nvars; ++i) {
      var_names.push_back( (boost::format("x_%i")%i).str() );
    }
    probptr->createVariables(var_names);
}

void expectAllNear(const std::vector<double>& x, const std::vector<double>& y, double abstol) {
    BOOST_CHECK_EQUAL(x.size(), y.size());
    LOG_INFO("checking %s ?= %s", CSTR(x), CSTR(y));
    for (size_t i=0; i < x.size(); ++i) {
        BOOST_CHECK_SMALL(x[i] - y[i], abstol);
    }
}

double f_QuadraticSeparable(const Eigen::VectorXd& x) {
    return x(0)*x(0) + sco::sq(x(1) - 1) + sco::sq(x(2)-2);
}

BOOST_AUTO_TEST_CASE(QuadraticSeparable)  {
    // if the problem is exactly a QP, it should be solved in one iteration
    sco::OptProbPtr prob;
    setupProblem(prob, 3);
    prob->addCost(sco::CostPtr(new sco::CostFromFunc(sco::ScalarOfVector::construct(&f_QuadraticSeparable), prob->getVars(), "f")));
    sco::BasicTrustRegionSQP solver(prob);
    solver.trust_box_size_ = 100;
    std::vector<double> x = boost::assign::list_of(3)(4)(5);
    solver.initialize(x);
    sco::OptStatus status = solver.optimize();
    BOOST_CHECK_EQUAL(status, sco::OPT_CONVERGED);
    expectAllNear(solver.x(), boost::assign::list_of(0)(1)(2), 1e-3);
    // todo: checks on number of iterations and function evaluates
}

double f_QuadraticNonseparable(const Eigen::VectorXd& x) {
    return sco::sq(x(0) - x(1) + 3*x(2)) + sco::sq(x(0)-1) + sco::sq(x(2) - 2);
}

BOOST_AUTO_TEST_CASE(QuadraticNonseparable)  {
    sco::OptProbPtr prob;
    setupProblem(prob, 3);
    prob->addCost(sco::CostPtr(new sco::CostFromFunc(sco::ScalarOfVector::construct(&f_QuadraticNonseparable), prob->getVars(), "f",  true)));
    sco::BasicTrustRegionSQP solver(prob);
    solver.trust_box_size_ = 100;
    solver.minTrustBoxSize_ = 1e-5;
    solver.minApproxImprove_ = 1e-6;
    std::vector<double> x = boost::assign::list_of(3)(4)(5);
    solver.initialize(x);
    sco::OptStatus status = solver.optimize();
    BOOST_CHECK_EQUAL(status, sco::OPT_CONVERGED);
    expectAllNear(solver.x(), boost::assign::list_of(1)(7)(2), .01);
    // todo: checks on number of iterations and function evaluates
}

void testProblem(sco::ScalarOfVectorPtr f, sco::VectorOfVectorPtr g, sco::ConstraintType cnt_type,
  const std::vector<double>& init, const std::vector<double>& sol) {
    sco::OptProbPtr prob;
    size_t n = init.size();
    assert(sol.size() == n);
    setupProblem(prob, n);
    prob->addCost(sco::CostPtr(new sco::CostFromFunc(f, prob->getVars(), "f", true)));
    prob->addConstraint(sco::ConstraintPtr(new sco::ConstraintFromFunc(g, prob->getVars(), Eigen::VectorXd(), cnt_type,"g")));
    sco::BasicTrustRegionSQP solver(prob);
    solver.maxIter_ = 1000;
    solver.minTrustBoxSize_ = 1e-5;
    solver.minApproxImprove_ = 1e-10;
    solver.merit_error_coeff_ = 1;

    solver.initialize(init);
    sco::OptStatus status = solver.optimize();
    BOOST_CHECK_EQUAL(status, sco::OPT_CONVERGED);
    expectAllNear(solver.x(), sol, .01);
}
// http://www.ai7.uni-bayreuth.de/test_problem_coll.pdf

// g_* defines the constraints on the given problem.
using sco::sq;


double f_TP1(const Eigen::VectorXd& x) {
    return 1*sq(x(1)-sq(x(0))) + sq(1-x(0));
}
Eigen::VectorXd g_TP1(const Eigen::VectorXd& x) {
    Eigen::VectorXd out(1);
    out(0) = -1.5 - x(1);
    return out;
}
double f_TP2(const Eigen::VectorXd& x) {
    return 100*sq(x(1)-sq(x(0))) + sq(1-x(0));
}
Eigen::VectorXd g_TP2(const Eigen::VectorXd& x) {
    Eigen::VectorXd out(1);
    out(0) = -1.5 - x(1);
    return out;
}
double f_TP3(const Eigen::VectorXd& x) {
    return (x(1) + 1e-5 * sq(x(1)-x(0)));
}
Eigen::VectorXd g_TP3(const Eigen::VectorXd& x) {
   Eigen::VectorXd out(1);
   out(0) = 0 - x(1);
    return out;
}
double f_TP6(const Eigen::VectorXd& x) {
    return sq(1-x(0));
}
Eigen::VectorXd g_TP6(const Eigen::VectorXd& x) {
    Eigen::VectorXd out(1);
    out(0) = 10*(x(1)-sq(x(0)));
    return out;
}
double f_TP7(const Eigen::VectorXd& x) {
   return log(1+sq(x(0))) - x(1);
}

Eigen::VectorXd g_TP7(const Eigen::VectorXd& x) {
    Eigen::VectorXd out(1);
    out(0) = sq(1+sq(x(0))) + sq(x(1)) - 4;
    return out;
}

BOOST_AUTO_TEST_CASE(TP1) {
    testProblem(sco::ScalarOfVector::construct(&f_TP1),
                sco::VectorOfVector::construct(&g_TP1), sco::INEQ,
                boost::assign::list_of(-2)(1),
                boost::assign::list_of(1)(1));
}
BOOST_AUTO_TEST_CASE(TP3) {
    testProblem(sco::ScalarOfVector::construct(&f_TP3),
                sco::VectorOfVector::construct(&g_TP3), sco::INEQ,
                boost::assign::list_of(10)(1),
                boost::assign::list_of(0)(0));
}
BOOST_AUTO_TEST_CASE(TP6) {
    testProblem(sco::ScalarOfVector::construct(&f_TP6),
                sco::VectorOfVector::construct(&g_TP6), sco::EQ,
                boost::assign::list_of(10)(1),
                boost::assign::list_of(1)(1));
}
BOOST_AUTO_TEST_CASE(TP7) {
    testProblem(sco::ScalarOfVector::construct(&f_TP7),
                sco::VectorOfVector::construct(&g_TP7), sco::EQ,
                boost::assign::list_of(2)(2),
                boost::assign::list_of(0.)(sqrtf(3.)));
}
