#define BOOST_TEST_MODULE CBFControlFilterTest
#include <boost/test/unit_test.hpp>

#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ompl/cbf/CBFControlFilter.h>
#include <ompl/util/RandomNumbers.h>

using Barrier = ompl::cbf::ClearanceBarrier;
using Filter = ompl::cbf::CBFControlFilter;
using ompl::cbf::ControlFilter;
using UR5 = ompl::robots::UR5;
namespace sdf = ompl::sdf;

namespace
{
    const Eigen::Vector3d obstacleCenter(0.3, 0.3, 1.1);
    constexpr double obstacleRadius = 0.2;
    constexpr double voxel = 0.02;

    sdf::DistanceFn sphereField(const Eigen::Vector3d &center, double radius)
    {
        return [center, radius](const Eigen::Vector3d &p) { return (p - center).norm() - radius; };
    }

    Eigen::AlignedBox3d reachableBox()
    {
        return Eigen::AlignedBox3d(Eigen::Vector3d(-1.1, -1.1, 0.35), Eigen::Vector3d(1.1, 1.1, 2.05));
    }

    /// An obstacle close enough to bind at the zero configuration.
    const sdf::GridSDF &nearField()
    {
        static const sdf::GridSDF field(sphereField(obstacleCenter, obstacleRadius), reachableBox(), voxel);
        return field;
    }

    /// An obstacle far enough away that no clearance row ever binds, so tests can
    /// isolate the control box.
    const sdf::GridSDF &farField()
    {
        static const sdf::GridSDF field(sphereField(Eigen::Vector3d(8.0, 8.0, 8.0), 0.2), reachableBox(),
                                        voxel);
        return field;
    }

    /// The step length every test propagates over.
    constexpr double duration = 0.05;

    Filter::Parameters parameters(double gamma = 0.4)
    {
        Filter::Parameters p;
        p.gamma = gamma;
        // Generous, so the clearance rows rather than the box are what bind.
        p.maxSpeed = UR5::Configuration::Constant(10.0);
        p.respectJointLimits = false;
        return p;
    }

    /// A nominal control aimed straight at the tightest obstacle, scaled by \p speed.
    UR5::Configuration towardWorstSphere(const Barrier &barrier, const UR5::Configuration &q, double speed)
    {
        const Barrier::Evaluation evaluation = barrier.evaluate(q);
        const Eigen::Index worst = static_cast<Eigen::Index>(evaluation.worst);
        return -evaluation.rows.row(worst).transpose().normalized() * speed;
    }
}  // namespace

// qpmad's dual active set starts from the unconstrained minimum, which for this
// objective is the nominal control. A safe step therefore costs zero active-set
// iterations and comes back bit-for-bit.
BOOST_AUTO_TEST_CASE(SafeNominalPassesThroughUntouched)
{
    const UR5 robot;
    const Barrier barrier(robot, nearField(), /*margin=*/0.0);
    const Filter filter(barrier, parameters());

    const UR5::Configuration q = UR5::Configuration::Zero();
    const UR5::Configuration nominal = towardWorstSphere(barrier, q, 1.0);

    UR5::Configuration filtered;
    Filter::Diagnostics diagnostics;
    BOOST_CHECK(filter.filter(q, nominal, duration, filtered, diagnostics) == ControlFilter::Status::Unchanged);
    BOOST_CHECK_EQUAL((filtered - nominal).norm(), 0.0);
    BOOST_CHECK_EQUAL(diagnostics.solverIterations, 0);
    BOOST_CHECK(diagnostics.inBounds);
}

// With one row active the QP has a closed form: projecting uNom onto
// {u : a^T u >= b} under the identity metric gives u = a*b/|a|^2. Because the
// nominal here points straight down -a, that result is *independent of how
// aggressive the nominal is* -- the filter always lands on the same boundary point.
BOOST_AUTO_TEST_CASE(ProjectionMatchesTheClosedForm)
{
    const UR5 robot;
    const Barrier barrier(robot, nearField(), /*margin=*/0.0);
    const Filter::Parameters p = parameters();
    const Filter filter(barrier, p);

    const UR5::Configuration q = UR5::Configuration::Zero();
    const Barrier::Evaluation evaluation = barrier.evaluate(q);
    const Eigen::Index worst = static_cast<Eigen::Index>(evaluation.worst);
    const UR5::Configuration a = evaluation.rows.row(worst).transpose();
    const double b = -p.gamma * evaluation.values[worst] / duration;
    const UR5::Configuration expected = a * b / a.squaredNorm();

    for (double speed : {2.0, 4.0, 10.0, 20.0})
    {
        UR5::Configuration filtered;
        BOOST_CHECK(filter.filter(q, towardWorstSphere(barrier, q, speed), duration, filtered) ==
                    ControlFilter::Status::Filtered);
        BOOST_CHECK_LE((filtered - expected).norm(), 1e-9);
        // The row it activated ends up exactly tight.
        BOOST_CHECK_LE(std::abs(a.dot(filtered) - b), 1e-9);
    }
}

// Every row must hold, not just the one that bound.
BOOST_AUTO_TEST_CASE(EveryClearanceRowIsSatisfied)
{
    const UR5 robot;
    const Barrier barrier(robot, nearField(), /*margin=*/0.0);
    const Filter::Parameters p = parameters();
    const Filter filter(barrier, p);

    const UR5::Configuration q = UR5::Configuration::Zero();
    const Barrier::Evaluation evaluation = barrier.evaluate(q);

    UR5::Configuration filtered;
    BOOST_REQUIRE(filter.filter(q, towardWorstSphere(barrier, q, 10.0), duration, filtered) ==
                  ControlFilter::Status::Filtered);

    // rows * u >= -gamma * h / dt, for all spheres.
    const Barrier::Values slack =
        evaluation.rows * filtered + evaluation.values * (p.gamma / duration);
    BOOST_CHECK_GE(slack.minCoeff(), -1e-9);
}

// The point of the whole exercise: a nominal control that would drive the arm into
// the obstacle is replaced by one that keeps clearance above (1 - gamma) of what it
// was, and the true clearance -- not just the linearized prediction -- holds up.
BOOST_AUTO_TEST_CASE(ClearanceDecayIsCappedByGamma)
{
    const UR5 robot;
    const Barrier barrier(robot, nearField(), /*margin=*/0.0);
    const Filter::Parameters p = parameters();
    const Filter filter(barrier, p);

    const UR5::Configuration q = UR5::Configuration::Zero();
    const double before = barrier.worstValue(q);
    BOOST_REQUIRE_GT(before, 0.0);

    const UR5::Configuration nominal = towardWorstSphere(barrier, q, 10.0);
    UR5::Configuration filtered;
    BOOST_REQUIRE(filter.filter(q, nominal, duration, filtered) == ControlFilter::Status::Filtered);

    // Unfiltered, this step ends in collision.
    BOOST_CHECK_LT(barrier.worstValue(q + nominal * duration), 0.0);

    // Filtered, clearance stays above the CBF floor. The 1 mm slack covers step
    // linearization; measured error at this step size is about 0.3 mm.
    const double after = barrier.worstValue(q + filtered * duration);
    BOOST_CHECK_GE(after, (1.0 - p.gamma) * before - 1e-3);
    BOOST_CHECK_GT(after, 0.0);
}

BOOST_AUTO_TEST_CASE(SmallerGammaIsMoreConservative)
{
    const UR5 robot;
    // Self rows are switched off for this one, and they have to be. The claim under test is
    // about the row the step is being driven *into* -- `nominal` aims at the worst sphere
    // against `nearField()` -- and gamma decides how much of that row's clearance the step
    // may spend. Leave the self rows in and a different row binds at some gammas but not
    // others, so `worstValue()` stops tracking the row the experiment is about and the
    // sequence is not monotone in gamma. That is not gamma failing to be conservative; it
    // is the measurement changing what it measures. A large negative margin puts every self
    // row far out of reach so none can ever be the minimum.
    const Barrier barrier(robot, nearField(), /*margin=*/0.0, /*selfMargin=*/-100.0);
    const UR5::Configuration q = UR5::Configuration::Zero();
    const UR5::Configuration nominal = towardWorstSphere(barrier, q, 10.0);

    double previous = -1.0;
    for (double gamma : {0.2, 0.4, 0.8, 1.0})
    {
        const Filter::Parameters p = parameters(gamma);
        const Filter filter(barrier, p);
        UR5::Configuration filtered;
        BOOST_REQUIRE(filter.filter(q, nominal, duration, filtered) == ControlFilter::Status::Filtered);

        // Larger gamma permits more decay, so the resulting clearance shrinks.
        const double after = barrier.worstValue(q + filtered * duration);
        if (previous >= 0.0)
            BOOST_CHECK_LT(after, previous);
        previous = after;
    }
}

BOOST_AUTO_TEST_CASE(SpeedBoxIsRespected)
{
    const UR5 robot;
    const Barrier barrier(robot, farField(), /*margin=*/0.0);
    Filter::Parameters p = parameters();
    p.maxSpeed = UR5::velocityLimits();  // 0.5 rad/s
    const Filter filter(barrier, p);

    const UR5::Configuration q = UR5::Configuration::Zero();
    UR5::Configuration filtered;
    BOOST_CHECK(filter.filter(q, UR5::Configuration::Constant(50.0), duration, filtered) ==
                ControlFilter::Status::Filtered);
    for (Eigen::Index j = 0; j < filtered.size(); ++j)
        BOOST_CHECK_LE(std::abs(filtered[j]), p.maxSpeed[j] + 1e-12);
}

BOOST_AUTO_TEST_CASE(JointLimitsAreRespected)
{
    const UR5 robot;
    const Barrier barrier(robot, farField(), /*margin=*/0.0);
    Filter::Parameters p = parameters();
    p.respectJointLimits = true;
    const Filter filter(barrier, p);

    // Start a hair below the upper limit and push hard against it.
    const UR5::Configuration upper = UR5::upperBounds();
    UR5::Configuration q = UR5::Configuration::Zero();
    q[0] = upper[0] - 0.001;

    UR5::Configuration filtered;
    BOOST_REQUIRE(filter.filter(q, UR5::Configuration::Constant(10.0), duration, filtered) !=
                  ControlFilter::Status::Blocked);

    const UR5::Configuration next = q + filtered * duration;
    for (Eigen::Index j = 0; j < next.size(); ++j)
        BOOST_CHECK_LE(next[j], upper[j] + 1e-12);
    BOOST_CHECK_LE(filtered[0], 0.001 / duration + 1e-12);
}

// Outside the baked field GridSDF clamps and over-reports clearance, so there is
// no trustworthy barrier. That must read as "cannot move", never as "safe".
BOOST_AUTO_TEST_CASE(LeavingTheFieldBlocksTheStep)
{
    const UR5 robot;
    const Eigen::AlignedBox3d tiny(Eigen::Vector3d(-0.2, -0.2, 0.8), Eigen::Vector3d(0.2, 0.2, 1.0));
    const sdf::GridSDF field(sphereField(obstacleCenter, obstacleRadius), tiny, voxel);
    const Barrier barrier(robot, field);
    const Filter filter(barrier, parameters());

    UR5::Configuration filtered;
    Filter::Diagnostics diagnostics;
    BOOST_CHECK(filter.filter(UR5::Configuration::Zero(), UR5::Configuration::Constant(0.1), duration,
                              filtered, diagnostics) == ControlFilter::Status::Blocked);
    BOOST_CHECK_EQUAL(filtered.norm(), 0.0);
    BOOST_CHECK(!diagnostics.inBounds);
}

// The base sphere's row is identically zero -- no joint can move it -- so an
// obstacle swallowing the base makes the QP infeasible. qpmad signals that by
// throwing, and the filter has to turn it into a clean "blocked".
BOOST_AUTO_TEST_CASE(CorneredReportsBlockedRatherThanThrowing)
{
    const UR5 robot;
    const UR5::Configuration q = UR5::Configuration::Zero();
    const Eigen::Vector3d base = robot.sphereCenters(q).col(0);
    const sdf::GridSDF field(sphereField(base, 0.5), reachableBox(), voxel);
    const Barrier barrier(robot, field, /*margin=*/0.0);
    const Filter filter(barrier, parameters());

    UR5::Configuration filtered;
    BOOST_CHECK_NO_THROW(filter.filter(q, UR5::Configuration::Constant(0.1), duration, filtered));
    BOOST_CHECK(filter.filter(q, UR5::Configuration::Constant(0.1), duration, filtered) ==
                ControlFilter::Status::Blocked);
    BOOST_CHECK_EQUAL(filtered.norm(), 0.0);
}

// PathControl::check() re-propagates an edge and compares against the stored
// state, so a non-deterministic filter would make every CBF path fail validation.
BOOST_AUTO_TEST_CASE(FilterIsDeterministic)
{
    const UR5 robot;
    const Barrier barrier(robot, nearField(), /*margin=*/0.0);
    const Filter filter(barrier, parameters());

    const UR5::Configuration q = UR5::Configuration::Zero();
    const UR5::Configuration nominal = towardWorstSphere(barrier, q, 10.0);

    UR5::Configuration first;
    UR5::Configuration second;
    filter.filter(q, nominal, duration, first);
    for (int repeat = 0; repeat < 5; ++repeat)
    {
        filter.filter(q, nominal, duration, second);
        BOOST_CHECK_EQUAL((first - second).norm(), 0.0);
    }
}

BOOST_AUTO_TEST_CASE(PassthroughFilterIsIdentity)
{
    const ompl::cbf::PassthroughFilter filter;
    const UR5::Configuration nominal = UR5::Configuration::Constant(123.0);
    UR5::Configuration filtered;
    BOOST_CHECK(filter.filter(UR5::Configuration::Zero(), nominal, duration, filtered) ==
                ControlFilter::Status::Unchanged);
    BOOST_CHECK_EQUAL((filtered - nominal).norm(), 0.0);
}

BOOST_AUTO_TEST_CASE(ControlBoundsIntersectSpeedAndJointLimits)
{
    const UR5 robot;
    const Barrier barrier(robot, farField());
    Filter::Parameters p = parameters();
    p.maxSpeed = UR5::Configuration::Constant(0.5);
    p.respectJointLimits = true;
    const Filter filter(barrier, p);

    UR5::Configuration q = UR5::Configuration::Zero();
    q[1] = UR5::lowerBounds()[1] + 0.002;  // near the lower limit

    UR5::Configuration lower;
    UR5::Configuration upper;
    filter.controlBounds(q, duration, lower, upper);

    BOOST_CHECK_LE(lower[1], upper[1]);
    BOOST_CHECK_CLOSE(lower[1], -0.002 / duration, 1e-9);  // joint limit binds
    BOOST_CHECK_CLOSE(upper[1], 0.5, 1e-9);                  // speed limit binds
    BOOST_CHECK_CLOSE(lower[0], -0.5, 1e-9);                 // mid-range joint: speed only
    BOOST_CHECK_CLOSE(upper[0], 0.5, 1e-9);
}

// Screening skips constraint rows for spheres that cannot bind within the step. Two
// things have to hold for that to be worth having: the control must almost always be the
// one the full 40-row solve would have produced, and the row count must actually drop.
//
// "Almost" is the honest word. For a screened-out sphere the CBF *decay* condition is no
// longer imposed, so a filtered control may differ from the unscreened one -- it is only
// guaranteed to keep that sphere safe, which is checked in test_clearance_barrier's
// DecreaseRatesBoundHowFastClearanceCanActuallyFall. What is checked here is that the
// difference is rare, and that when it happens the screened control is still admissible.
BOOST_AUTO_TEST_CASE(ScreeningMatchesTheFullSolveAndDropsMostRows)
{
    const UR5 robot;
    const Barrier barrier(robot, nearField(), 0.0);

    // Realistic speeds: the screen's threshold scales with maxSpeed, and at the
    // deliberately generous 10 rad/s the other tests use, nothing is ever screened.
    Filter::Parameters base = parameters();
    base.maxSpeed = UR5::velocityLimits();
    Filter::Parameters screenedParameters = base;
    screenedParameters.screening = true;
    Filter::Parameters fullParameters = base;
    fullParameters.screening = false;

    const Filter screened(barrier, screenedParameters);
    const Filter full(barrier, fullParameters);

    ompl::RNG rng;
    long rows = 0;
    int steps = 0, agreed = 0;
    for (int sample = 0; sample < 1500; ++sample)
    {
        UR5::Configuration q, nominal;
        for (Eigen::Index j = 0; j < 6; ++j)
        {
            q[j] = rng.uniformReal(UR5::lowerBounds()[j], UR5::upperBounds()[j]);
            nominal[j] = rng.uniformReal(-base.maxSpeed[j], base.maxSpeed[j]);
        }

        UR5::Configuration screenedControl, fullControl;
        Filter::Diagnostics diagnostics;
        const ControlFilter::Status status =
            screened.filter(q, nominal, duration, screenedControl, diagnostics);
        if (status == ControlFilter::Status::Blocked)
            continue;
        full.filter(q, nominal, duration, fullControl);

        // Admissible whatever else is true.
        for (Eigen::Index j = 0; j < 6; ++j)
            BOOST_REQUIRE_LE(std::abs(screenedControl[j]), base.maxSpeed[j] + 1e-9);

        rows += diagnostics.activeRows;
        ++steps;
        if ((screenedControl - fullControl).norm() <= 1e-9)
            ++agreed;
    }

    BOOST_REQUIRE_GT(steps, 200);
    const double meanRows = static_cast<double>(rows) / steps;
    const double agreement = static_cast<double>(agreed) / steps;
    BOOST_TEST_MESSAGE("screened rows " << meanRows << " of 40, agreement " << agreement);
    BOOST_CHECK_LT(meanRows, 25.0);
    BOOST_CHECK_GT(agreement, 0.9);
}
