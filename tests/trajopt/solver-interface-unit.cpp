#define BOOST_TEST_MODULE "solver_interface"
#include <boost/test/unit_test.hpp>
#include <boost/test/included/unit_test.hpp>

#include <cstdio>
#include <boost/foreach.hpp>
#include <iostream>

#include "ompl/trajopt/solver_interface.h"
#include "ompl/trajopt/logging.h"
#include "ompl/trajopt/expr_ops.h"
#include "ompl/trajopt/stl_to_string.h"
using namespace std;

using namespace sco;

BOOST_AUTO_TEST_CASE(setup_problem) {
    sco::ModelPtr solver = createModel();
    std::vector<sco::Var> vars;
    for (int i=0; i < 3; ++i) {
        char namebuf[5];
        sprintf(namebuf, "v%i", i);
        vars.push_back(solver->addVar(namebuf));
    }
    solver->update();

    sco::AffExpr aff;
    for (int i=0; i < 3; ++i) {
        exprInc(aff, vars[i]);
        solver->setVarBounds(vars[i], 0, 10);
    }
    aff.constant -= 3;
    sco::QuadExpr affsquared = exprSquare(aff);
    solver->setObjective(affsquared);
    solver->update();
    LOG_INFO("objective: %s", CSTR(affsquared));
    LOG_INFO("please manually check that /tmp/solver-interface-test.lp matches this");
    solver->writeToFile("/tmp/solver-interface-test.lp");

    solver->optimize();

    std::vector<double> soln(3);
    for (int i=0; i < 3; ++i) {
        soln[i] = solver->getVarValue(vars[i]);
    }
    BOOST_CHECK_SMALL(aff.value(soln), 1e-6); // checks for = 0

    solver->removeVar(vars[2]);
    solver->update();
    BOOST_CHECK_EQUAL(solver->getVars().size(), 2);
}
