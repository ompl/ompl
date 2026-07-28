#define BOOST_TEST_MODULE QpmadVendoredTest
#include <boost/test/unit_test.hpp>

#include <limits>
#include <stdexcept>

#include <Eigen/Dense>

#include <qpmad/solver.h>

// Checks that the vendored qpmad builds and behaves the way the CBF steering
// step depends on. The QP there always has this shape:
//
//   minimize   0.5 (u - uNom)^T W (u - uNom)      W diagonal, positive definite
//   subject to A u >= -gamma * h                  one clearance row per sphere
//              uMin <= u <= uMax                  control and joint-limit box
//
// which qpmad takes as (H = W, h = -W uNom, lb, ub, A, Alb = -gamma h, Aub = inf).

namespace
{
    constexpr int nJoints = 6;
    const double infinity = std::numeric_limits<double>::infinity();

    using Vector = Eigen::Matrix<double, nJoints, 1>;

    Vector nominalControl()
    {
        Vector uNom;
        uNom << 0.05, -0.03, 0.02, 0.01, -0.04, 0.06;
        return uNom;
    }

    // Unit weights keep the analytic projection below easy to state; the filter
    // will use a genuine diagonal W.
    Eigen::MatrixXd weights()
    {
        return Eigen::MatrixXd::Identity(nJoints, nJoints);
    }
}  // namespace

// The dual active set starts from the unconstrained minimum, which for this
// objective *is* the nominal control. So a step that already satisfies every
// barrier costs almost nothing and comes back bit-for-bit unchanged — the filter
// needs no "is the nominal safe?" pre-check of its own.
BOOST_AUTO_TEST_CASE(FeasibleNominalIsReturnedUnchanged)
{
    const Vector uNom = nominalControl();
    Eigen::MatrixXd H = weights();
    const Eigen::VectorXd h = -uNom;
    const Eigen::VectorXd lb = Eigen::VectorXd::Constant(nJoints, -0.08);
    const Eigen::VectorXd ub = Eigen::VectorXd::Constant(nJoints, 0.08);

    Eigen::MatrixXd A(1, nJoints);
    A << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    Eigen::VectorXd Alb(1);
    Alb << -1.0;  // slack by a mile
    Eigen::VectorXd Aub(1);
    Aub << infinity;

    Eigen::VectorXd u;
    qpmad::Solver solver;
    BOOST_CHECK_EQUAL(solver.solve(u, H, h, lb, ub, A, Alb, Aub), qpmad::Solver::OK);
    BOOST_CHECK_EQUAL((u - uNom).norm(), 0.0);
}

// One violated barrier: the solution is the W-weighted projection of the nominal
// control onto that halfspace, and the constraint ends up tight.
BOOST_AUTO_TEST_CASE(SingleViolatedRowGivesTheWeightedProjection)
{
    const Vector uNom = nominalControl();
    Eigen::MatrixXd H = weights();
    const Eigen::VectorXd h = -uNom;
    const Eigen::VectorXd lb = Eigen::VectorXd::Constant(nJoints, -1.0);
    const Eigen::VectorXd ub = Eigen::VectorXd::Constant(nJoints, 1.0);

    Eigen::MatrixXd A(1, nJoints);
    A << 1.0, 1.0, 0.0, 0.0, 0.0, 0.0;
    const double bound = 0.05;  // a^T uNom = 0.02, so this row is violated
    Eigen::VectorXd Alb(1);
    Alb << bound;
    Eigen::VectorXd Aub(1);
    Aub << infinity;

    Eigen::VectorXd u;
    qpmad::Solver solver;
    BOOST_CHECK_EQUAL(solver.solve(u, H, h, lb, ub, A, Alb, Aub), qpmad::Solver::OK);

    // u = uNom + W^-1 a (bound - a^T uNom) / (a^T W^-1 a), with W = I here.
    const Eigen::VectorXd a = A.row(0).transpose();
    const Eigen::VectorXd expected = uNom + a * (bound - a.dot(uNom)) / a.dot(a);
    BOOST_CHECK_LE((u - expected).norm(), 1e-12);
    BOOST_CHECK_LE(std::abs(a.dot(u) - bound), 1e-12);
}

BOOST_AUTO_TEST_CASE(BoxBoundsAreRespected)
{
    // A nominal control outside the box must be pulled back into it.
    Vector uNom = Vector::Constant(0.5);
    Eigen::MatrixXd H = weights();
    const Eigen::VectorXd h = -uNom;
    const Eigen::VectorXd lb = Eigen::VectorXd::Constant(nJoints, -0.08);
    const Eigen::VectorXd ub = Eigen::VectorXd::Constant(nJoints, 0.08);

    Eigen::VectorXd u;
    qpmad::Solver solver;
    BOOST_CHECK_EQUAL(solver.solve(u, H, h, lb, ub), qpmad::Solver::OK);
    for (Eigen::Index j = 0; j < u.size(); ++j)
    {
        BOOST_CHECK_LE(u[j], ub[j] + 1e-12);
        BOOST_CHECK_GE(u[j], lb[j] - 1e-12);
        BOOST_CHECK_CLOSE(u[j], ub[j], 1e-9);  // clipped to the upper bound
    }
}

// Being cornered is a normal event for a steering filter, and qpmad signals it
// by *throwing*, not by a return status. The filter must catch this and report
// the step as blocked rather than let it escape into the planner.
BOOST_AUTO_TEST_CASE(InfeasibleConstraintsThrow)
{
    const Vector uNom = nominalControl();
    Eigen::MatrixXd H = weights();
    const Eigen::VectorXd h = -uNom;
    const Eigen::VectorXd lb = Eigen::VectorXd::Constant(nJoints, -0.08);
    const Eigen::VectorXd ub = Eigen::VectorXd::Constant(nJoints, 0.08);

    Eigen::MatrixXd A(2, nJoints);
    A << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,  //
        -1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    Eigen::VectorXd Alb(2);
    Alb << 0.5, 0.5;  // u0 >= 0.5 and -u0 >= 0.5 cannot both hold
    Eigen::VectorXd Aub(2);
    Aub << infinity, infinity;

    Eigen::VectorXd u;
    qpmad::Solver solver;
    BOOST_CHECK_THROW(solver.solve(u, H, h, lb, ub, A, Alb, Aub), std::runtime_error);
}

// Compile-time sizing avoids heap allocation in the planner's inner loop, and
// must agree with the dynamically sized solver.
BOOST_AUTO_TEST_CASE(FixedSizeSolverAgreesWithDynamic)
{
    const Vector uNom = nominalControl();
    constexpr int nRows = 2;

    Eigen::MatrixXd Arows(nRows, nJoints);
    Arows << 1.0, 1.0, 0.0, 0.0, 0.0, 0.0,  //
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
    Eigen::VectorXd bounds(nRows);
    bounds << 0.05, -1.0;

    Eigen::VectorXd dynamicSolution;
    {
        Eigen::MatrixXd H = weights();
        Eigen::VectorXd Aub = Eigen::VectorXd::Constant(nRows, infinity);
        qpmad::Solver solver;
        BOOST_CHECK_EQUAL(solver.solve(dynamicSolution, H, Eigen::VectorXd(-uNom),
                                       Eigen::VectorXd::Constant(nJoints, -0.08),
                                       Eigen::VectorXd::Constant(nJoints, 0.08), Arows, bounds, Aub),
                          qpmad::Solver::OK);
    }

    Eigen::Matrix<double, nJoints, 1> fixedSolution;
    {
        Eigen::Matrix<double, nJoints, nJoints> H = Eigen::Matrix<double, nJoints, nJoints>::Identity();
        const Eigen::Matrix<double, nJoints, 1> h = -uNom;
        const Eigen::Matrix<double, nJoints, 1> lb = Eigen::Matrix<double, nJoints, 1>::Constant(-0.08);
        const Eigen::Matrix<double, nJoints, 1> ub = Eigen::Matrix<double, nJoints, 1>::Constant(0.08);
        const Eigen::Matrix<double, nRows, nJoints> A = Arows;
        const Eigen::Matrix<double, nRows, 1> Alb = bounds;
        const Eigen::Matrix<double, nRows, 1> Aub = Eigen::Matrix<double, nRows, 1>::Constant(infinity);

        qpmad::SolverTemplate<double, nJoints, 1, nRows> solver;
        BOOST_CHECK_EQUAL(solver.solve(fixedSolution, H, h, lb, ub, A, Alb, Aub), qpmad::Solver::OK);
    }

    BOOST_CHECK_LE((dynamicSolution - fixedSolution).norm(), 1e-12);
}
