#define BOOST_TEST_MODULE UR5Test
#include <boost/test/unit_test.hpp>

#include <array>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ompl/robots/UR5.h>
#include <ompl/util/RandomNumbers.h>

using UR5 = ompl::robots::UR5;

namespace
{
    UR5::Configuration configuration(double q0, double q1, double q2, double q3, double q4, double q5)
    {
        return (UR5::Configuration() << q0, q1, q2, q3, q4, q5).finished();
    }

    // A handful of configurations that exercise every joint.
    std::array<UR5::Configuration, 4> testConfigurations()
    {
        return {{
            UR5::Configuration::Zero(),
            configuration(0.0, -1.2, 1.8, -0.6, 1.57, 0.0),
            configuration(1.0, -0.5, 0.7, 0.3, -1.0, 0.5),
            configuration(-2.4, 2.1, -1.5, 0.9, 2.8, -1.7),
        }};
    }
}  // namespace

BOOST_AUTO_TEST_CASE(SphereTableIsWellFormed)
{
    const auto &spheres = UR5::spheres();
    BOOST_CHECK_EQUAL(spheres.size(), UR5::nSpheres);

    for (std::size_t i = 0; i < UR5::nSpheres; ++i)
    {
        BOOST_CHECK_LT(spheres[i].frame, UR5::nFrames);
        BOOST_CHECK_GT(spheres[i].radius, 0.0);
        BOOST_CHECK_EQUAL(UR5::radii()[static_cast<Eigen::Index>(i)], spheres[i].radius);
    }

    // Frames are assigned in chain order, so the table is sorted by frame.
    for (std::size_t i = 1; i < UR5::nSpheres; ++i)
        BOOST_CHECK_GE(spheres[i].frame, spheres[i - 1].frame);
}

// Reference centers cross-validated against vamp.ur5.fk (single precision there,
// so agreement is to ~1e-6 m). Pins the joint origins, axis conventions, and the
// pre-composed gripper transforms all at once.
BOOST_AUTO_TEST_CASE(ForwardKinematicsMatchesVampReference)
{
    const UR5 robot;
    const double tolerance = 1e-6;

    {
        const UR5::SphereCenters centers = robot.sphereCenters(UR5::Configuration::Zero());
        BOOST_CHECK_LE((centers.col(0) - Eigen::Vector3d(0.0, 0.0, 0.9144)).norm(), tolerance);
        BOOST_CHECK_LE((centers.col(1) - Eigen::Vector3d(0.0, 0.0, 1.003559)).norm(), tolerance);
        BOOST_CHECK_LE((centers.col(7) - Eigen::Vector3d(-0.015811556, 0.425012726, 1.003559001)).norm(),
                       tolerance);
        BOOST_CHECK_LE((centers.col(22) - Eigen::Vector3d(-0.168499148, 0.81738444, 0.908909001)).norm(),
                       tolerance);
        BOOST_CHECK_LE((centers.col(39) - Eigen::Vector3d(-0.306890422, 0.879870034, 0.908344347)).norm(),
                       tolerance);
    }
    {
        const UR5::SphereCenters centers = robot.sphereCenters(configuration(0.0, -1.2, 1.8, -0.6, 1.57, 0.0));
        BOOST_CHECK_LE((centers.col(7) - Eigen::Vector3d(-0.016027359, 0.154014857, 1.399675612)).norm(),
                       tolerance);
        BOOST_CHECK_LE((centers.col(22) - Eigen::Vector3d(-0.108769528, 0.537826708, 1.083544602)).norm(),
                       tolerance);
        BOOST_CHECK_LE((centers.col(39) - Eigen::Vector3d(-0.046394159, 0.676267697, 1.082979949)).norm(),
                       tolerance);
    }
}

BOOST_AUTO_TEST_CASE(BaseSphereNeverMoves)
{
    const UR5 robot;
    const Eigen::Vector3d expected = robot.basePose().translation();
    for (const UR5::Configuration &q : testConfigurations())
        BOOST_CHECK_LE((robot.sphereCenters(q).col(0) - expected).norm(), 1e-12);
}

BOOST_AUTO_TEST_CASE(BasePoseRelocatesTheWholeArm)
{
    // Working in base_link coordinates should differ from the pedestal-mounted
    // default by exactly that fixed transform.
    const UR5 mounted;
    const UR5 atOrigin(Eigen::Isometry3d::Identity());
    const Eigen::Isometry3d base = UR5::vampBasePose();

    for (const UR5::Configuration &q : testConfigurations())
    {
        const UR5::SphereCenters mine = atOrigin.sphereCenters(q);
        const UR5::SphereCenters reference = mounted.sphereCenters(q);
        for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(UR5::nSpheres); ++i)
            BOOST_CHECK_LE((base * mine.col(i) - reference.col(i)).norm(), 1e-12);
    }
}

// The analytic Jacobian is what the CBF-QP differentiates through, so check it
// against central differences of the forward kinematics itself.
BOOST_AUTO_TEST_CASE(SphereJacobianMatchesCentralDifferences)
{
    const UR5 robot;
    const double step = 1e-6;

    for (const UR5::Configuration &q : testConfigurations())
    {
        const UR5::Kinematics kin = robot.kinematics(q);

        UR5::SphereCenters forward[UR5::nJoints];
        UR5::SphereCenters backward[UR5::nJoints];
        for (std::size_t j = 0; j < UR5::nJoints; ++j)
        {
            UR5::Configuration offset = UR5::Configuration::Zero();
            offset[static_cast<Eigen::Index>(j)] = step;
            forward[j] = robot.sphereCenters(q + offset);
            backward[j] = robot.sphereCenters(q - offset);
        }

        for (std::size_t i = 0; i < UR5::nSpheres; ++i)
        {
            const UR5::PositionJacobian analytic = UR5::sphereJacobian(kin, i);
            for (std::size_t j = 0; j < UR5::nJoints; ++j)
            {
                const Eigen::Index column = static_cast<Eigen::Index>(j);
                const Eigen::Index sphere = static_cast<Eigen::Index>(i);
                const Eigen::Vector3d numeric =
                    (forward[j].col(sphere) - backward[j].col(sphere)) / (2.0 * step);
                BOOST_CHECK_LE((analytic.col(column) - numeric).norm(), 1e-7);
            }
        }
    }
}

BOOST_AUTO_TEST_CASE(DownstreamJointsCannotMoveASphere)
{
    const UR5 robot;
    for (const UR5::Configuration &q : testConfigurations())
    {
        const UR5::Kinematics kin = robot.kinematics(q);
        for (std::size_t i = 0; i < UR5::nSpheres; ++i)
        {
            const UR5::PositionJacobian jacobian = UR5::sphereJacobian(kin, i);
            // Sphere on frame f is moved by joints 0..f-1 only.
            for (std::size_t j = UR5::spheres()[i].frame; j < UR5::nJoints; ++j)
                BOOST_CHECK_EQUAL(jacobian.col(static_cast<Eigen::Index>(j)).norm(), 0.0);
        }
    }
}

// barrierGradient is the row a CBF constraint contributes: dh/dq = grad(d)^T J.
BOOST_AUTO_TEST_CASE(BarrierGradientIsTheChainRule)
{
    const UR5 robot;
    const Eigen::Vector3d gradient = Eigen::Vector3d(0.3, -0.5, 0.8).normalized();

    for (const UR5::Configuration &q : testConfigurations())
    {
        const UR5::Kinematics kin = robot.kinematics(q);
        for (std::size_t i = 0; i < UR5::nSpheres; ++i)
        {
            const UR5::Configuration expected = UR5::sphereJacobian(kin, i).transpose() * gradient;
            BOOST_CHECK_LE((UR5::barrierGradient(kin, i, gradient) - expected).norm(), 1e-15);
        }
    }
}

BOOST_AUTO_TEST_CASE(EndEffectorIsTheWristFrame)
{
    const UR5 robot;
    for (const UR5::Configuration &q : testConfigurations())
    {
        const UR5::Kinematics kin = robot.kinematics(q);
        const Eigen::Isometry3d pose = UR5::endEffectorPose(kin);
        BOOST_CHECK_LE((pose.translation() - kin.translation[UR5::nFrames - 1]).norm(), 1e-15);
        // Rotation matrices stay orthonormal down the chain.
        BOOST_CHECK_LE((pose.linear() * pose.linear().transpose() - Eigen::Matrix3d::Identity()).norm(),
                       1e-12);
    }
}

BOOST_AUTO_TEST_CASE(JointBoundsAreSymmetric)
{
    BOOST_CHECK_EQUAL(UR5::lowerBounds().size(), static_cast<Eigen::Index>(UR5::nJoints));
    for (Eigen::Index j = 0; j < static_cast<Eigen::Index>(UR5::nJoints); ++j)
    {
        BOOST_CHECK_LT(UR5::lowerBounds()[j], UR5::upperBounds()[j]);
        BOOST_CHECK_CLOSE(UR5::lowerBounds()[j], -UR5::upperBounds()[j], 1e-9);
    }
}

// leverArmBounds() claims a configuration-independent bound on how fast each joint can
// move each sphere. That claim is what makes CBF row screening sound, so it is checked
// against the exact Jacobian over a large random sample: every column norm of every
// sphere Jacobian, at every configuration, must fall under its table entry.
BOOST_AUTO_TEST_CASE(LeverArmBoundsDominateEveryActualJacobianColumn)
{
    const UR5 robot;
    const auto &bounds = UR5::leverArmBounds();
    ompl::RNG rng;

    double worstRatio = 0.0;
    for (int sample = 0; sample < 3000; ++sample)
    {
        UR5::Configuration q;
        for (std::size_t j = 0; j < UR5::nJoints; ++j)
            q[static_cast<Eigen::Index>(j)] =
                rng.uniformReal(UR5::lowerBounds()[static_cast<Eigen::Index>(j)],
                                UR5::upperBounds()[static_cast<Eigen::Index>(j)]);

        const UR5::Kinematics kin = robot.kinematics(q);
        for (std::size_t i = 0; i < UR5::nSpheres; ++i)
        {
            const UR5::PositionJacobian jacobian = UR5::sphereJacobian(kin, i);
            for (std::size_t k = 0; k < UR5::nJoints; ++k)
            {
                const double actual = jacobian.col(static_cast<Eigen::Index>(k)).norm();
                const double bound =
                    bounds(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(k));
                BOOST_REQUIRE_LE(actual, bound + 1e-12);
                if (bound > 1e-12)
                    worstRatio = std::max(worstRatio, actual / bound);
            }
        }
    }

    // Sound is the requirement; useful is the point. If the bound were wildly loose the
    // screen would select every sphere and buy nothing, so pin that it is within a small
    // factor of something actually attained.
    BOOST_CHECK_GT(worstRatio, 0.5);
}

// Downstream joints cannot move a sphere at all, so their bound must be exactly zero --
// this is what lets a screened evaluation drop the base sphere's all-zero row for free.
BOOST_AUTO_TEST_CASE(LeverArmBoundsAreZeroForDownstreamJoints)
{
    const auto &bounds = UR5::leverArmBounds();
    for (std::size_t i = 0; i < UR5::nSpheres; ++i)
        for (std::size_t k = UR5::spheres()[i].frame; k < UR5::nJoints; ++k)
            BOOST_CHECK_EQUAL(bounds(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(k)),
                              0.0);
}
