#define BOOST_TEST_MODULE ClearanceBarrierTest
#include <boost/test/unit_test.hpp>

#include <array>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>

#include <ompl/cbf/ClearanceBarrier.h>
#include <ompl/util/RandomNumbers.h>

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
    // values to 0.45 mm, rows to 26 mm. The values are second-order accurate and
    // scale as O(voxel^2); the rows are only first-order and scale as O(voxel) —
    // see RowsMatchTheAnalyticGradient for why. Both tolerances leave ~2x headroom
    // over the measurement.
    constexpr double valueTolerance = 1e-3;
    constexpr double rowTolerance = 5e-2;

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

// The rows are what a QP steering step differentiates through, so check them
// against the exact chain rule J_i^T * grad d.
//
// The tolerance here is ~17x looser than valueTolerance because GridSDF's
// gradient is the exact derivative of its trilinearly interpolated value. That
// derivative is piecewise constant along each axis within a cell, which makes it
// only O(voxel) accurate — a differentiated interpolant loses an order relative
// to the interpolant itself. Interpolating central-difference node gradients
// instead would land inside 1.5 mm here, but it yields a gradient that is not the
// derivative of any field the barrier evaluates, and the Lipschitz screening in
// ClearanceBarrier::decreaseRates needs those two to agree to be sound. Accuracy
// is the deliberate trade; shrink the voxel if a row needs to be tighter.
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

// The two margins are independent offsets on the two families of barrier, and neither
// touches a constraint direction. They are separate because they absorb different
// errors: the world margin carries the field's discretization, which a pair of sphere
// centres never queries.
BOOST_AUTO_TEST_CASE(MarginShiftsEveryValueUniformly)
{
    const UR5 robot;
    const Barrier bare(robot, obstacleField(), /*margin=*/0.0, /*selfMargin=*/0.0);
    const double margin = 0.06;
    const double selfMargin = 0.01;
    const Barrier padded(robot, obstacleField(), margin, selfMargin);

    for (const UR5::Configuration &q : testConfigurations())
    {
        const Barrier::Values without = bare.values(q);
        const Barrier::Values with = padded.values(q);
        for (Eigen::Index i = 0; i < Barrier::nSpheres; ++i)
            BOOST_CHECK_LE(std::abs((without[i] - margin) - with[i]), 1e-15);
        for (Eigen::Index i = Barrier::nSpheres; i < Barrier::nConstraints; ++i)
            BOOST_CHECK_LE(std::abs((without[i] - selfMargin) - with[i]), 1e-15);

        // The margins must not change the constraint directions, only the offsets.
        BOOST_CHECK_LE((bare.evaluate(q).rows - padded.evaluate(q).rows).norm(), 1e-15);
    }
}

// `worst` indexes the world spheres and `worstPair` the self-collision pairs, kept
// apart on purpose — see the field's comment for what reading a self row as "which way
// is the obstacle" would do. Safety, by contrast, is a question about all of them.
BOOST_AUTO_TEST_CASE(WorstIsTheArgminAndDrivesIsSafe)
{
    const UR5 robot;
    const Barrier barrier(robot, obstacleField());

    for (const UR5::Configuration &q : testConfigurations())
    {
        const Barrier::Evaluation evaluation = barrier.evaluate(q);
        const Eigen::Index worst = static_cast<Eigen::Index>(evaluation.worst);
        const Eigen::Index worstPair =
            Barrier::nSpheres + static_cast<Eigen::Index>(evaluation.worstPair);

        BOOST_CHECK_LT(worst, Barrier::nSpheres);
        BOOST_CHECK_EQUAL(evaluation.values[worst],
                          evaluation.values.head<Barrier::nSpheres>().minCoeff());
        BOOST_CHECK_EQUAL(evaluation.values[worstPair],
                          evaluation.values.tail<Barrier::nSelfPairs>().minCoeff());

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

    // Check the shift on the *world* rows, not on worstValue(). Since the self rows carry
    // their own per-pair margins, the smallest row overall is often a self row -- at the
    // zero configuration it is -- and setMargin() does not touch those. Asserting on
    // worstValue() would be asserting that no self row is ever the tightest, which is
    // false by design and has nothing to do with whether setMargin works.
    const UR5::Configuration q = UR5::Configuration::Zero();
    const Barrier::Values before = barrier.values(q);
    barrier.setMargin(barrier.margin() + 0.01);
    const Barrier::Values after = barrier.values(q);

    for (Eigen::Index i = 0; i < UR5::nSpheres; ++i)
        BOOST_CHECK_LE(std::abs(after[i] - (before[i] - 0.01)), 1e-15);
    // and the self rows are unmoved by it
    for (Eigen::Index i = UR5::nSpheres; i < Barrier::nConstraints; ++i)
        BOOST_CHECK_LE(std::abs(after[i] - before[i]), 1e-15);
}

BOOST_AUTO_TEST_CASE(SelfMarginIsSettableAndSeparate)
{
    const UR5 robot;
    Barrier barrier(robot, obstacleField());
    const UR5::Configuration q = UR5::Configuration::Zero();
    const Barrier::Values before = barrier.values(q);
    barrier.setSelfMargin(barrier.selfMargin() + 0.01);
    const Barrier::Values after = barrier.values(q);

    // The mirror image: selfMargin shifts every self row and no world row. It stacks on
    // top of each pair's own calibrated margin rather than replacing it.
    for (Eigen::Index i = 0; i < UR5::nSpheres; ++i)
        BOOST_CHECK_LE(std::abs(after[i] - before[i]), 1e-15);
    for (Eigen::Index i = UR5::nSpheres; i < Barrier::nConstraints; ++i)
        BOOST_CHECK_LE(std::abs(after[i] - (before[i] - 0.01)), 1e-15);
}

// Every self-collision row is claimed to be the exact derivative of the pair clearance.
// Unlike the world rows, which are only as good as the field's interpolated gradient,
// this one has no approximation in it at all, so central differences should agree to
// several digits rather than to a tolerance set by the voxel size.
BOOST_AUTO_TEST_CASE(SelfPairRowsMatchCentralDifferences)
{
    const UR5 robot;
    const Barrier barrier(robot, obstacleField(), 0.0);
    constexpr double step = 1e-6;

    ompl::RNG rng;
    double worst = 0.0;
    for (int sample = 0; sample < 40; ++sample)
    {
        UR5::Configuration q;
        for (Eigen::Index j = 0; j < 6; ++j)
            q[j] = rng.uniformReal(UR5::lowerBounds()[j], UR5::upperBounds()[j]);

        const Barrier::Evaluation evaluation = barrier.evaluate(q);
        for (Eigen::Index j = 0; j < 6; ++j)
        {
            UR5::Configuration offset = UR5::Configuration::Zero();
            offset[j] = step;
            const Barrier::Values ahead = barrier.values(q + offset);
            const Barrier::Values behind = barrier.values(q - offset);

            for (Eigen::Index p = 0; p < Barrier::nSelfPairs; ++p)
            {
                const Eigen::Index i = Barrier::nSpheres + p;
                const double finite = (ahead[i] - behind[i]) / (2.0 * step);
                worst = std::max(worst, std::abs(evaluation.rows(i, j) - finite));
            }
        }
    }
    BOOST_CHECK_LT(worst, 1e-6);
}

// The sparsity that `UR5::selfPairLeverArms()` derives, checked as a property of the
// geometry rather than of the code that assumes it. A joint upstream of both spheres
// carries them as one rigid body; a joint downstream of both moves neither. Only the
// joints strictly between the two frames can change the separation at all.
//
// "Exactly zero" is the claim worth testing, not "small". A spurious 1e-17 in column 0
// would let the QP believe the base joint can relieve a wrist-against-forearm collision,
// and the certificate would charge for a motion that cannot happen.
BOOST_AUTO_TEST_CASE(SelfPairRowsVanishOutsideTheJointsBetweenTheFrames)
{
    const UR5 robot;
    const Barrier barrier(robot, obstacleField(), 0.0);

    ompl::RNG rng;
    for (int sample = 0; sample < 50; ++sample)
    {
        UR5::Configuration q;
        for (Eigen::Index j = 0; j < 6; ++j)
            q[j] = rng.uniformReal(UR5::lowerBounds()[j], UR5::upperBounds()[j]);

        const Barrier::Evaluation evaluation = barrier.evaluate(q);
        for (std::size_t p = 0; p < UR5::nSelfPairs; ++p)
        {
            const UR5::SelfPair &pair = UR5::selfPairs()[p];
            const std::size_t first = UR5::spheres()[pair.a].frame;
            const std::size_t second = UR5::spheres()[pair.b].frame;
            BOOST_REQUIRE_LT(first, second);  // the table is ordered by frame

            for (std::size_t k = 0; k < UR5::nJoints; ++k)
            {
                if (k >= first && k < second)
                    continue;
                const Eigen::Index i = Barrier::nSpheres + static_cast<Eigen::Index>(p);
                BOOST_REQUIRE_EQUAL(evaluation.rows(i, static_cast<Eigen::Index>(k)), 0.0);
            }
        }
    }
}

// The same statement about the clearance itself rather than its derivative, and over a
// large perturbation rather than an infinitesimal one: rotating a joint outside the
// window through half a turn cannot move two spheres relative to each other.
BOOST_AUTO_TEST_CASE(SelfPairClearanceIgnoresJointsOutsideTheWindow)
{
    const UR5 robot;
    const Barrier barrier(robot, obstacleField(), 0.0);

    ompl::RNG rng;
    for (int sample = 0; sample < 40; ++sample)
    {
        UR5::Configuration q;
        for (Eigen::Index j = 0; j < 6; ++j)
            q[j] = rng.uniformReal(UR5::lowerBounds()[j], UR5::upperBounds()[j]);
        const Barrier::Values before = barrier.values(q);

        for (std::size_t k = 0; k < UR5::nJoints; ++k)
        {
            UR5::Configuration moved = q;
            moved[static_cast<Eigen::Index>(k)] = rng.uniformReal(-3.14, 3.14);
            const Barrier::Values after = barrier.values(moved);

            for (std::size_t p = 0; p < UR5::nSelfPairs; ++p)
            {
                const UR5::SelfPair &pair = UR5::selfPairs()[p];
                if (k >= UR5::spheres()[pair.a].frame && k < UR5::spheres()[pair.b].frame)
                    continue;
                const Eigen::Index i = Barrier::nSpheres + static_cast<Eigen::Index>(p);
                BOOST_REQUIRE_LE(std::abs(after[i] - before[i]), 1e-12);
            }
        }
    }
}

// The claim that makes row screening sound, stated as directly as it can be tested:
// clearance cannot fall faster than `decreaseRates()` says, for *any* admissible control.
// Unlike the CBF row itself this is not a linearisation -- the lever-arm bound holds at
// every configuration along the step, so integrating it bounds the true change in h.
// Adversarial controls are used (full speed, random signs, plus the exact worst-case
// direction for the tightest sphere) rather than benign ones.
BOOST_AUTO_TEST_CASE(DecreaseRatesBoundHowFastClearanceCanActuallyFall)
{
    const UR5 robot;
    const Barrier barrier(robot, obstacleField(), 0.0);
    const UR5::Configuration maxSpeed = UR5::velocityLimits();
    const Barrier::Values rates = barrier.decreaseRates(maxSpeed);
    constexpr double duration = 0.05;

    ompl::RNG rng;
    double worstSlack = std::numeric_limits<double>::infinity();
    double worstPairSlack = std::numeric_limits<double>::infinity();
    for (int sample = 0; sample < 4000; ++sample)
    {
        UR5::Configuration q;
        for (Eigen::Index j = 0; j < 6; ++j)
            q[j] = rng.uniformReal(UR5::lowerBounds()[j], UR5::upperBounds()[j]);

        // Full-speed control, signs chosen to drive the worst sphere's clearance down as
        // hard as the linear model allows -- the nastiest admissible direction there is.
        const Barrier::Evaluation evaluation = barrier.evaluate(q);
        UR5::Configuration u;
        for (Eigen::Index j = 0; j < 6; ++j)
        {
            const double descend =
                -evaluation.rows(static_cast<Eigen::Index>(evaluation.worst), j);
            u[j] = maxSpeed[j] * ((descend >= 0.0) ? 1.0 : -1.0);
            if (sample % 2 == 0)  // and plain random signs on the other half
                u[j] = maxSpeed[j] * (rng.uniform01() < 0.5 ? -1.0 : 1.0);
        }

        const Barrier::Values before = barrier.values(q);
        const Barrier::Values after = barrier.values(q + u * duration);
        for (Eigen::Index i = 0; i < Barrier::nConstraints; ++i)
        {
            const double allowed = rates[i] * duration;
            const double fell = before[i] - after[i];
            BOOST_REQUIRE_LE(fell, allowed + 1e-9);
            if (i < Barrier::nSpheres)
                worstSlack = std::min(worstSlack, allowed - fell);
            else if (rates[i] > 0.0)
                worstPairSlack = std::min(worstPairSlack, allowed - fell);
        }
    }

    // Sound, and not so slack as to be useless: something must come close to the bound.
    // The two families are measured apart because a pair whose bound is *identically*
    // zero -- the sphere sits on the only axis between the two frames -- would otherwise
    // report perfect tightness for free and hide a loose bound everywhere else.
    BOOST_CHECK_LT(worstSlack, 0.05);
    BOOST_CHECK_LT(worstPairSlack, 0.05);
}

// A screened evaluation must agree with a full one on everything it reports: identical
// barrier values for every sphere, and identical rows for the spheres it kept. It is a
// cheaper way to compute the same numbers, not a different barrier.
BOOST_AUTO_TEST_CASE(ScreenedEvaluationAgreesWithTheFullOneOnWhatItKeeps)
{
    const UR5 robot;
    const Barrier barrier(robot, obstacleField(), 0.0);
    const Barrier::Values rates = barrier.decreaseRates(UR5::velocityLimits());
    const Barrier::Values threshold = rates * 0.05;

    ompl::RNG rng;
    long kept = 0;
    int evaluations = 0;
    for (int sample = 0; sample < 500; ++sample)
    {
        UR5::Configuration q;
        for (Eigen::Index j = 0; j < 6; ++j)
            q[j] = rng.uniformReal(UR5::lowerBounds()[j], UR5::upperBounds()[j]);

        const Barrier::Evaluation full = barrier.evaluate(q);
        Barrier::Evaluation screened;
        barrier.evaluateScreened(q, threshold, screened);

        BOOST_REQUIRE_LE((screened.values - full.values).cwiseAbs().maxCoeff(), 1e-12);
        BOOST_REQUIRE_EQUAL(screened.worst, full.worst);
        BOOST_REQUIRE_EQUAL(screened.worstPair, full.worstPair);
        BOOST_REQUIRE_EQUAL(screened.inBounds, full.inBounds);
        BOOST_REQUIRE_LE(screened.active, Barrier::nConstraints);

        for (Eigen::Index r = 0; r < screened.active; ++r)
        {
            const Eigen::Index i = screened.constraint[r];
            // Kept barriers are exactly those that could bind.
            BOOST_REQUIRE_LE(screened.values[i], threshold[i]);
            BOOST_REQUIRE_LE((screened.rows.row(r) - full.rows.row(i)).cwiseAbs().maxCoeff(), 1e-12);
        }
        kept += screened.active;
        ++evaluations;
    }

    // And it earns its keep: of 357 barriers, almost none is anywhere near binding over
    // one step. The self-collision pairs are the bulk of what does survive -- in open
    // space the arm is nearer to itself than to anything else.
    BOOST_CHECK_LT(static_cast<double>(kept) / evaluations, 20.0);
}

// The certificate is what lets a caller stop checking, so it has to hold *along* the
// span and not merely at its end: sample the straight motion it certifies and require
// the CBF condition every sphere would have been held to at every point of it.
//
// This is the same Lipschitz bound as DecreaseRatesBoundHowFastClearanceCanActuallyFall,
// solved for the duration instead of the drop, so what is being tested is the inversion
// rather than the bound. What that adds is the interval: a duration is only useful if
// nothing goes wrong before it elapses.
BOOST_AUTO_TEST_CASE(NothingCanBindWithinTheCertifiedDuration)
{
    const UR5 robot;
    const Barrier barrier(robot, obstacleField(), 0.0);
    const UR5::Configuration maxSpeed = UR5::velocityLimits();
    constexpr double gamma = 0.4;

    ompl::RNG rng;
    double longest = 0.0;
    int certified = 0;
    for (int sample = 0; sample < 2000; ++sample)
    {
        UR5::Configuration q;
        for (Eigen::Index j = 0; j < 6; ++j)
            q[j] = rng.uniformReal(UR5::lowerBounds()[j], UR5::upperBounds()[j]);

        const Barrier::Evaluation evaluation = barrier.evaluate(q);
        if (!evaluation.inBounds || evaluation.values.minCoeff() <= 0.0)
            continue;  // nothing to certify from a configuration already in violation

        // Aim straight down the steepest descent of the worst sphere's barrier, at full
        // speed: the fastest any admissible control can spend the clearance.
        UR5::Configuration u;
        for (Eigen::Index j = 0; j < 6; ++j)
        {
            const double descend = -evaluation.rows(static_cast<Eigen::Index>(evaluation.worst), j);
            u[j] = maxSpeed[j] * ((descend >= 0.0) ? 1.0 : -1.0);
        }

        const double duration = barrier.certifiedDuration(evaluation, u, gamma);
        BOOST_REQUIRE_GT(duration, 0.0);
        longest = std::max(longest, duration);
        ++certified;

        constexpr int samples = 25;
        for (int step = 1; step <= samples; ++step)
        {
            const double t = duration * step / samples;
            const Barrier::Values along = barrier.values(q + u * t);
            for (Eigen::Index i = 0; i < Barrier::nConstraints; ++i)
                BOOST_REQUIRE_GE(along[i], (1.0 - gamma) * evaluation.values[i] - 1e-9);
        }
    }

    BOOST_REQUIRE_GT(certified, 100);
    // Worth having: somewhere there is room for a step well past the 0.05 s the rollout
    // would otherwise have taken.
    BOOST_CHECK_GT(longest, 0.05);
}

// The box the field was baked over is not an obstacle, so no barrier value falls as a
// centre approaches it -- but a query outside it is clamped and comes back optimistic.
// A certificate that only watched the clearances would happily run a sphere out of the
// field; this one is cut short by the boundary as well.
BOOST_AUTO_TEST_CASE(TheCertificateStopsAtTheEdgeOfTheField)
{
    const UR5 robot;
    const UR5::Configuration q = configuration(0.0, -1.2, 1.8, -0.6, 1.57, 0.0);

    // A field with nothing in it, baked over the arm's own bounding box plus a hand's
    // width: clearance is enormous everywhere and the boundary is the only thing that can
    // bind. Derived from the configuration rather than hardcoded, so it is the box that
    // is tight around the arm rather than the arm that has to be posed to suit the box.
    Eigen::AlignedBox3d box;
    const UR5::SphereCenters centers = robot.sphereCenters(q);
    for (Eigen::Index i = 0; i < centers.cols(); ++i)
        box.extend(centers.col(i));
    box.min().array() -= 0.1;
    box.max().array() += 0.1;

    const sdf::GridSDF cramped(sphereField(Eigen::Vector3d(0.0, 0.0, 40.0), 0.1), box, voxel);
    const Barrier barrier(robot, cramped, 0.0);

    const Barrier::Evaluation evaluation = barrier.evaluate(q);
    BOOST_REQUIRE(evaluation.inBounds);
    // The obstacle is 40 m up. Only the world clearances are enormous: a self-collision
    // pair measures the arm against itself and has no idea where the obstacle is.
    BOOST_REQUIRE_GT(evaluation.values.head<Barrier::nSpheres>().minCoeff(), 1.0);
    BOOST_REQUIRE_LE(evaluation.boundary.minCoeff(), 0.1);

    UR5::Configuration u = UR5::Configuration::Zero();
    u[1] = UR5::velocityLimits()[1];  // swing the arm through the wall of the box
    const double duration = barrier.certifiedDuration(evaluation, u, 1.0);

    BOOST_REQUIRE_GT(duration, 0.0);
    BOOST_REQUIRE(std::isfinite(duration));

    // The same arm in a field wide enough that no centre can ever leave it: the
    // clearances are the same, the self pairs are identical, and the only term that
    // disappears is the boundary. It certifies strictly longer, which is what shows the
    // boundary — and not the self-collision rows, which are also in the minimum now —
    // is what cut the span above.
    const sdf::GridSDF roomy(sphereField(Eigen::Vector3d(0.0, 0.0, 40.0), 0.1),
                             UR5::reachableBounds(), 4 * voxel);
    const Barrier unbounded(robot, roomy, 0.0);
    BOOST_CHECK_GT(unbounded.certifiedDuration(unbounded.evaluate(q), u, 1.0), duration);

    for (int step = 0; step <= 20; ++step)
    {
        bool inBounds = false;
        Barrier::Values values;
        barrier.values(q + u * (duration * step / 20), values, &inBounds);
        BOOST_CHECK(inBounds);
    }
}
