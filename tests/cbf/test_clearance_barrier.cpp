#define BOOST_TEST_MODULE ClearanceBarrierTest
#include <boost/test/unit_test.hpp>

#include <array>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ompl/cbf/ClearanceBarrier.h>

using Barrier = ompl::cbf::ClearanceBarrier;
using UR5 = ompl::robots::UR5;
namespace sdf = ompl::sdf;

namespace
{
    // A single spherical obstacle: the signed distance and its gradient are exact
    // in closed form, so the barrier can be checked against the truth rather than
    // against another approximation.
    const Eigen::Vector3d obstacleCenter(0.3, 0.3, 1.1);
    constexpr double obstacleRadius = 0.2;
    constexpr double voxel = 0.02;

    // Errors measured against the analytic field at this voxel size: barrier
    // values to 0.44 mm, rows to 1.5 mm. Both scale as O(voxel^2).
    constexpr double valueTolerance = 1e-3;
    constexpr double rowTolerance = 3e-3;

    sdf::DistanceFn sphereField(const Eigen::Vector3d &center, double radius)
    {
        return [center, radius](const Eigen::Vector3d &p) { return (p - center).norm() - radius; };
    }

    // Large enough that every sphere center stays inside at the test configurations.
    Eigen::AlignedBox3d reachableBox()
    {
        return Eigen::AlignedBox3d(Eigen::Vector3d(-1.1, -1.1, 0.35), Eigen::Vector3d(1.1, 1.1, 2.05));
    }

    UR5::Configuration configuration(double q0, double q1, double q2, double q3, double q4, double q5)
    {
        return (UR5::Configuration() << q0, q1, q2, q3, q4, q5).finished();
    }

    std::array<UR5::Configuration, 4> testConfigurations()
    {
        return {{
            UR5::Configuration::Zero(),
            configuration(0.0, -1.2, 1.8, -0.6, 1.57, 0.0),
            configuration(1.0, -0.5, 0.7, 0.3, -1.0, 0.5),
            configuration(-0.7, -0.9, 1.1, 0.4, 0.8, -1.2),
        }};
    }

    const sdf::GridSDF &obstacleField()
    {
        static const sdf::GridSDF field(sphereField(obstacleCenter, obstacleRadius), reachableBox(), voxel);
        return field;
    }
}  // namespace

BOOST_AUTO_TEST_CASE(ValuesMatchTheAnalyticField)
{
    const UR5 robot;
    const Barrier barrier(robot, obstacleField(), /*margin=*/0.0);

    for (const UR5::Configuration &q : testConfigurations())
    {
        const Barrier::Evaluation evaluation = barrier.evaluate(q);
        BOOST_REQUIRE(evaluation.inBounds);

        const UR5::Kinematics kin = robot.kinematics(q);
        for (std::size_t i = 0; i < UR5::nSpheres; ++i)
        {
            const Eigen::Vector3d p = UR5::sphereCenter(kin, i);
            const double exact = (p - obstacleCenter).norm() - obstacleRadius - UR5::spheres()[i].radius;
            BOOST_CHECK_LE(std::abs(evaluation.values[static_cast<Eigen::Index>(i)] - exact), valueTolerance);
        }
    }
}

// The rows are what a QP steering step differentiates through. Compare them to
// the exact chain rule J_i^T * grad d, not to finite differences of the cached
// values: GridSDF interpolates values and gradients independently, so the
// interpolated gradient is deliberately not the derivative of the interpolated
// value, and a finite-difference check measures that inconsistency (~25x larger,
// and only O(voxel)) rather than the row's actual accuracy.
BOOST_AUTO_TEST_CASE(RowsMatchTheAnalyticGradient)
{
    const UR5 robot;
    const Barrier barrier(robot, obstacleField(), /*margin=*/0.0);

    for (const UR5::Configuration &q : testConfigurations())
    {
        const Barrier::Evaluation evaluation = barrier.evaluate(q);
        const UR5::Kinematics kin = robot.kinematics(q);

        for (std::size_t i = 0; i < UR5::nSpheres; ++i)
        {
            const Eigen::Vector3d p = UR5::sphereCenter(kin, i);
            const Eigen::Vector3d gradient = (p - obstacleCenter).normalized();
            const UR5::Configuration exact = UR5::sphereJacobian(kin, i).transpose() * gradient;
            const Eigen::Index row = static_cast<Eigen::Index>(i);
            BOOST_CHECK_LE((evaluation.rows.row(row).transpose() - exact).norm(), rowTolerance);
        }
    }
}

// The base sphere cannot be moved by any joint, so it can never contribute a
// useful constraint — its row must be exactly zero.
BOOST_AUTO_TEST_CASE(BaseSphereRowIsZero)
{
    const UR5 robot;
    const Barrier barrier(robot, obstacleField());
    for (const UR5::Configuration &q : testConfigurations())
        BOOST_CHECK_EQUAL(barrier.evaluate(q).rows.row(0).norm(), 0.0);
}

BOOST_AUTO_TEST_CASE(MarginShiftsEveryValueUniformly)
{
    const UR5 robot;
    const Barrier bare(robot, obstacleField(), /*margin=*/0.0);
    const double margin = 0.06;
    const Barrier padded(robot, obstacleField(), margin);

    for (const UR5::Configuration &q : testConfigurations())
    {
        const Barrier::Values without = bare.values(q);
        const Barrier::Values with = padded.values(q);
        for (Eigen::Index i = 0; i < without.size(); ++i)
            BOOST_CHECK_LE(std::abs((without[i] - margin) - with[i]), 1e-15);

        // The margin must not change the constraint directions, only the offsets.
        BOOST_CHECK_LE((bare.evaluate(q).rows - padded.evaluate(q).rows).norm(), 1e-15);
    }
}

BOOST_AUTO_TEST_CASE(WorstIsTheArgminAndDrivesIsSafe)
{
    const UR5 robot;
    const Barrier barrier(robot, obstacleField());

    for (const UR5::Configuration &q : testConfigurations())
    {
        const Barrier::Evaluation evaluation = barrier.evaluate(q);
        const Eigen::Index worst = static_cast<Eigen::Index>(evaluation.worst);
        BOOST_CHECK_EQUAL(evaluation.values[worst], evaluation.values.minCoeff());
        BOOST_CHECK_EQUAL(barrier.worstValue(q), evaluation.values.minCoeff());
        BOOST_CHECK_EQUAL(barrier.isSafe(q), evaluation.values.minCoeff() >= 0.0);
    }
}

// values() is the cheap path used for plain safety questions; it must agree with
// the full evaluation it is meant to shortcut.
BOOST_AUTO_TEST_CASE(CheapValuesAgreeWithFullEvaluation)
{
    const UR5 robot;
    const Barrier barrier(robot, obstacleField());
    for (const UR5::Configuration &q : testConfigurations())
        BOOST_CHECK_LE((barrier.values(q) - barrier.evaluate(q).values).norm(), 1e-12);
}

// A sphere inside the obstacle must report negative clearance -- the barrier has
// to be able to say "already violated", not just "close".
BOOST_AUTO_TEST_CASE(ClearanceGoesNegativeInsideAnObstacle)
{
    const UR5 robot;
    const UR5::Configuration q = UR5::Configuration::Zero();

    // Wrap an obstacle around a sphere that is occupied at this configuration.
    const Eigen::Vector3d occupied = robot.sphereCenters(q).col(7);
    const sdf::GridSDF field(sphereField(occupied, 0.1), reachableBox(), voxel);
    const Barrier barrier(robot, field, /*margin=*/0.0);

    const Barrier::Evaluation evaluation = barrier.evaluate(q);
    BOOST_CHECK(!barrier.isSafe(q));
    BOOST_CHECK_LT(evaluation.values.minCoeff(), 0.0);
    // Sphere 7 sits at the obstacle's center, so it is buried by radius + r_7.
    BOOST_CHECK_LE(evaluation.values[7], -0.1);
}

// GridSDF clamps out-of-bounds queries, which *over*-reports clearance -- the one
// direction a barrier must never fail in. Leaving the box has to be visible.
BOOST_AUTO_TEST_CASE(LeavingTheFieldIsReported)
{
    const UR5 robot;
    // A box that only covers the pedestal, so the arm itself is outside it.
    const Eigen::AlignedBox3d tiny(Eigen::Vector3d(-0.2, -0.2, 0.8), Eigen::Vector3d(0.2, 0.2, 1.0));
    const sdf::GridSDF field(sphereField(obstacleCenter, obstacleRadius), tiny, voxel);
    const Barrier barrier(robot, field);

    const UR5::Configuration q = configuration(0.0, -1.2, 1.8, -0.6, 1.57, 0.0);
    BOOST_CHECK(!barrier.evaluate(q).inBounds);

    bool inBounds = true;
    Barrier::Values values;
    barrier.values(q, values, &inBounds);
    BOOST_CHECK(!inBounds);

    // And the whole-arm case is genuinely inside, so the flag discriminates.
    BOOST_CHECK(barrier.robot().sphereCenters(q).cols() > 0);
    const Barrier reference(robot, obstacleField());
    BOOST_CHECK(reference.evaluate(q).inBounds);
}

BOOST_AUTO_TEST_CASE(MarginIsSettable)
{
    const UR5 robot;
    Barrier barrier(robot, obstacleField());
    BOOST_CHECK_EQUAL(barrier.margin(), Barrier::defaultMargin);

    const UR5::Configuration q = UR5::Configuration::Zero();
    const double before = barrier.worstValue(q);
    barrier.setMargin(barrier.margin() + 0.01);
    BOOST_CHECK_LE(std::abs(barrier.worstValue(q) - (before - 0.01)), 1e-15);
}
