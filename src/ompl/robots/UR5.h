#pragma once

#include <array>
#include <cstddef>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ompl::robots
{
    /// A 6-DoF Universal Robots UR5 with its links approximated by spheres.
    ///
    /// This is the robot-side half of the CBF primitive: for each collision
    /// sphere i with radius r_i and center p_i(q), a workspace signed distance
    /// field gives the barrier
    ///
    ///     h_i(q) = d(p_i(q)) - r_i
    ///
    /// and this class supplies both p_i(q) and the exact Jacobian dp_i/dq, so
    /// the barrier derivative follows by the chain rule:
    ///
    ///     dh_i/dq = grad d(p_i)^T * (dp_i/dq)
    ///
    /// Everything is closed form in one pass down the kinematic chain (no
    /// numerical differentiation), and the only dependency is Eigen.
    ///
    /// Sphere model
    /// ------------
    /// The 40 spheres are lifted verbatim from VAMP's hand-authored
    /// `ur5_spherized.urdf`, so this model and VAMP's planner see the same
    /// collision geometry. Each sphere is attached to one of 7 frames — the base
    /// plus the 6 links moved by the 6 revolute joints. The 17 spheres covering
    /// the force/torque sensor and the Robotiq 85 gripper are joined to the arm
    /// by *fixed* joints, so their centers are pre-expressed in `wrist_3_link`'s
    /// frame; they simply ride along with the wrist. See
    /// `scripts/generate_ur5_spheres.py`, which derives the table below from the
    /// URDF and cross-validates it against `vamp.ur5.fk`.
    ///
    /// Frames
    /// ------
    /// Poses are reported in the frame of `basePose`'s parent. The default
    /// `vampBasePose()` reproduces VAMP's ur5.urdf setup — the arm sitting on a
    /// 0.9144 m pedestal, yawed 1.57 rad — which puts sphere centers in exactly
    /// the coordinates `vamp.ur5.fk` reports and VAMP's benchmark scenes use.
    /// Pass `Eigen::Isometry3d::Identity()` to work in `base_link` coordinates
    /// instead, or any other transform to mount the arm somewhere else.
    class UR5
    {
    public:
        static constexpr std::size_t nJoints = 6;
        static constexpr std::size_t nSpheres = 40;
        /// base_link plus the 6 links moved by the joints. Frame f is moved by
        /// joints 0..f-1, so frame 0 never moves.
        static constexpr std::size_t nFrames = 7;

        using Configuration = Eigen::Matrix<double, nJoints, 1>;
        /// d(point)/dq for a single workspace point: 3 rows, one column per joint.
        using PositionJacobian = Eigen::Matrix<double, 3, nJoints>;
        using SphereCenters = Eigen::Matrix<double, 3, nSpheres>;

        /// One collision sphere, rigidly attached to frame `frame`.
        struct Sphere
        {
            std::size_t frame;
            Eigen::Vector3d center;  ///< in `frame`'s coordinates
            double radius;
            const char *link;  ///< URDF link it was authored on, for debugging
        };

        /// Forward kinematics evaluated at one configuration.
        ///
        /// `rotation`/`translation` place a point given in frame f into world
        /// coordinates. `jointOrigin`/`jointAxis` are the world-frame position and
        /// rotation axis of each joint — the two things a position Jacobian column
        /// needs.
        ///
        /// `jointMoment` is `jointAxis x jointOrigin`, which is what lets
        /// `barrierGradient()` skip building a Jacobian at all: it is the only part of a
        /// Jacobian column that does not depend on which sphere is being asked about, so
        /// it belongs to the configuration rather than to the sphere. Six cross products
        /// here replace 240 there.
        struct Kinematics
        {
            std::array<Eigen::Matrix3d, nFrames> rotation;
            std::array<Eigen::Vector3d, nFrames> translation;
            std::array<Eigen::Vector3d, nJoints> jointOrigin;
            std::array<Eigen::Vector3d, nJoints> jointAxis;
            std::array<Eigen::Vector3d, nJoints> jointMoment;
        };

        explicit UR5(const Eigen::Isometry3d &basePose = vampBasePose()) : basePose_(basePose)
        {
        }

        /// The fixed `offset_link -> base_link` hop in VAMP's ur5.urdf: a 0.9144 m
        /// pedestal with a 1.57 rad yaw. This is the default base pose, so sphere
        /// centers match `vamp.ur5.fk` and VAMP's scenes out of the box.
        static Eigen::Isometry3d vampBasePose()
        {
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.linear() = Eigen::AngleAxisd(1.57, Eigen::Vector3d::UnitZ()).toRotationMatrix();
            pose.translation() = Eigen::Vector3d(0.0, 0.0, 0.9144);
            return pose;
        }

        const Eigen::Isometry3d &basePose() const
        {
            return basePose_;
        }

        /// The sphere approximation of the links: 40 (frame, center, radius) triples.
        static const std::array<Sphere, nSpheres> &spheres()
        {
            // Generated by scripts/generate_ur5_spheres.py from
            // vamp/resources/ur5/ur5_spherized.urdf. Do not hand-edit.
            static const std::array<Sphere, nSpheres> table{{
                // clang-format off
                {0, {0.0, 0.0, 0.0}, 0.08, "base_link"},
                {1, {0.0, 0.0, 0.0}, 0.08, "shoulder_link"},
                {2, {0.0, 0.0, 0.105}, 0.08, "upper_arm_link"},
                {2, {0.0, 0.0, 0.21}, 0.08, "upper_arm_link"},
                {2, {0.0, 0.0, 0.315}, 0.08, "upper_arm_link"},
                {2, {0.0, 0.0, 0.42}, 0.08, "upper_arm_link"},
                {2, {0.0, 0.0, 0.0}, 0.08, "upper_arm_link"},
                {3, {0.0, 0.0, 0.0}, 0.08, "forearm_link"},
                {3, {0.0, 0.0, 0.1}, 0.04, "forearm_link"},
                {3, {0.0, 0.0, 0.14}, 0.04, "forearm_link"},
                {3, {0.0, 0.0, 0.18}, 0.04, "forearm_link"},
                {3, {0.0, 0.0, 0.21999999999999997}, 0.04, "forearm_link"},
                {3, {0.0, 0.0, 0.25999999999999995}, 0.04, "forearm_link"},
                {3, {0.0, 0.0, 0.3}, 0.04, "forearm_link"},
                {3, {0.0, 0.0, 0.34}, 0.04, "forearm_link"},
                {3, {0.0, 0.0, 0.38000000000000006}, 0.04, "forearm_link"},
                {4, {0.0, 0.09, 0.03}, 0.04, "wrist_1_link"},
                {4, {0.0, 0.09, -0.03}, 0.04, "wrist_1_link"},
                {4, {0.0, 0.09, 0.0}, 0.04, "wrist_1_link"},
                {5, {0.0, 0.03, 0.09}, 0.04, "wrist_2_link"},
                {5, {0.0, -0.03, 0.09}, 0.04, "wrist_2_link"},
                {5, {0.0, 0.0, 0.09}, 0.04, "wrist_2_link"},
                {6, {0.0, 0.06, 0.0}, 0.04, "wrist_3_link"},
                {6, {1.5926561138124373e-05, 0.09730000634133472, 0.0}, 0.04, "fts_robotside"},
                {6, {-4.777941697950423e-05, 0.17729996195191194, 0.0005477796026439954}, 0.04, "robotiq_85_base_link"},
                {6, {-1.5926430445582725e-05, 0.13729998731730397, 0.0005159265342146654}, 0.04, "robotiq_85_base_link"},
                {6, {0.03055113195505351, 0.18011634490469636, 0.0005500029597762315}, 0.02, "robotiq_85_left_knuckle_link"},
                {6, {0.06221185321003585, 0.21820759337657858, 0.0005803159612080763}, 0.015, "robotiq_85_left_finger_link"},
                {6, {0.06224370619656978, 0.17820761874197064, 0.0005484628927787463}, 0.015, "robotiq_85_left_finger_link"},
                {6, {0.062227779703302816, 0.19820760605927462, 0.0005643894269934113}, 0.015, "robotiq_85_left_finger_link"},
                {6, {0.03262887193001728, 0.2066334831917182, 0.0005711179477187814}, 0.02, "robotiq_85_left_inner_knuckle_link"},
                {6, {0.04717396614483508, 0.25714208519804244, 0.0006113300730649224}, 0.015, "robotiq_85_left_finger_tip_link"},
                {6, {0.047193874261418786, 0.23214210105141242, 0.0005914219052965894}, 0.015, "robotiq_85_left_finger_tip_link"},
                {6, {-0.032771107333818245, 0.20658140354229462, 0.0005711179477187744}, 0.02, "robotiq_85_right_inner_knuckle_link"},
                {6, {-0.046917649399262115, 0.25742752614889924, 0.0006116170441051241}, 0.015, "robotiq_85_right_finger_tip_link"},
                {6, {-0.04689774128267841, 0.23242754200226923, 0.0005917088763367911}, 0.015, "robotiq_85_right_finger_tip_link"},
                {6, {-0.030651137491679723, 0.18006760799717728, 0.0005500029597762315}, 0.02, "robotiq_85_right_knuckle_link"},
                {6, {-0.06239129616120959, 0.2184409760144585, 0.0005805808252547159}, 0.015, "robotiq_85_right_finger_link"},
                {6, {-0.06235944317467567, 0.17844100137985053, 0.000548727756825379}, 0.015, "robotiq_85_right_finger_link"},
                {6, {-0.06237536966794264, 0.1984409886971545, 0.000564654291040044}, 0.015, "robotiq_85_right_finger_link"},
                // clang-format on
            }};
            return table;
        }

        /// Sphere radii in the order `spheres()` lists them — the r_i in h_i = d - r_i.
        static const Eigen::Matrix<double, nSpheres, 1> &radii()
        {
            static const Eigen::Matrix<double, nSpheres, 1> values = []
            {
                Eigen::Matrix<double, nSpheres, 1> r;
                for (std::size_t i = 0; i < nSpheres; ++i)
                    r[static_cast<Eigen::Index>(i)] = spheres()[i].radius;
                return r;
            }();
            return values;
        }

        /// Joint limits from VAMP's ur5.urdf (+/- pi on every joint).
        static Configuration lowerBounds()
        {
            return Configuration::Constant(-3.14159265);
        }

        static Configuration upperBounds()
        {
            return Configuration::Constant(3.14159265);
        }

        /// Joint velocity limits from VAMP's ur5.urdf (0.5 rad/s on every joint).
        static Configuration velocityLimits()
        {
            return Configuration::Constant(0.5);
        }

        /// A box enclosing every collision sphere's *surface* at every reachable
        /// configuration, in the same frame as `basePose`.
        ///
        /// Size a workspace signed distance field to at least this, or larger: a
        /// `sdf::GridSDF` clamps queries outside its box and so *over*-reports
        /// clearance, which silently invalidates any barrier built on it. The arm
        /// reaches further than it looks — 1.13 m in cylindrical radius, and from
        /// below the pedestal top up to 2.12 m — so a box drawn around a few
        /// hand-picked configurations will not do.
        ///
        /// Measured over 200k uniformly sampled configurations with the default
        /// (VAMP pedestal) base pose, then padded; recompute if `basePose` differs.
        static Eigen::AlignedBox3d reachableBounds()
        {
            return Eigen::AlignedBox3d(Eigen::Vector3d(-1.15, -1.15, -0.15),
                                       Eigen::Vector3d(1.15, 1.15, 2.20));
        }

        /// Walk the chain once, filling in every frame pose and joint axis.
        void kinematics(const Configuration &q, Kinematics &out) const
        {
            Eigen::Matrix3d rotation = basePose_.linear();
            Eigen::Vector3d translation = basePose_.translation();
            out.rotation[0] = rotation;
            out.translation[0] = translation;

            for (std::size_t i = 0; i < nJoints; ++i)
            {
                const JointOrigin &origin = jointOrigins()[i];
                translation += rotation * origin.translation;
                rotation *= origin.rotation;

                // The joint's frame, before its own rotation is applied: this is
                // the point and axis the Jacobian column pivots about.
                out.jointOrigin[i] = translation;
                out.jointAxis[i] = rotation * origin.axis;
                out.jointMoment[i] = out.jointAxis[i].cross(out.jointOrigin[i]);

                rotation *= Eigen::AngleAxisd(q[static_cast<Eigen::Index>(i)], origin.axis).toRotationMatrix();
                out.rotation[i + 1] = rotation;
                out.translation[i + 1] = translation;
            }
        }

        Kinematics kinematics(const Configuration &q) const
        {
            Kinematics out;
            kinematics(q, out);
            return out;
        }

        /// World-frame pose of one of the `nFrames` kinematic frames.
        static Eigen::Isometry3d framePose(const Kinematics &kin, std::size_t frame)
        {
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.linear() = kin.rotation[frame];
            pose.translation() = kin.translation[frame];
            return pose;
        }

        /// World-frame pose of the tool flange (`wrist_3_link`).
        static Eigen::Isometry3d endEffectorPose(const Kinematics &kin)
        {
            return framePose(kin, nFrames - 1);
        }

        /// World-frame center of sphere `i` — the p_i(q) in h_i(q) = d(p_i(q)) - r_i.
        static Eigen::Vector3d sphereCenter(const Kinematics &kin, std::size_t i)
        {
            const Sphere &sphere = spheres()[i];
            return kin.translation[sphere.frame] + kin.rotation[sphere.frame] * sphere.center;
        }

        /// All 40 sphere centers, one per column.
        static void sphereCenters(const Kinematics &kin, SphereCenters &out)
        {
            for (std::size_t i = 0; i < nSpheres; ++i)
                out.col(static_cast<Eigen::Index>(i)) = sphereCenter(kin, i);
        }

        SphereCenters sphereCenters(const Configuration &q) const
        {
            SphereCenters out;
            sphereCenters(kinematics(q), out);
            return out;
        }

        /// Configuration-independent upper bounds on `|dp_i/dq_k|`: entry (i, k) bounds
        /// how fast joint k can move sphere i, for every reachable configuration at once.
        ///
        /// Column k of a sphere Jacobian is `axis_k x (p_i - o_k)`, whose norm is sphere
        /// i's distance to joint k's axis — so this table bounds that distance, and
        /// therefore (since `|grad d| <= 1` for a signed distance field) also bounds
        /// `|dh_i/dq_k|` for any barrier built on the field. That is what makes it useful:
        /// it says which spheres *could* lose clearance over a step before any gradient or
        /// Jacobian has been computed. See `cbf::ClearanceBarrier::decreaseRates()`.
        ///
        /// The bound comes from the chain, not from sampling. Expanding the forward
        /// kinematics,
        ///
        ///     p_i - o_k = sum_{j=k+1}^{f_i - 1} R_j t_j  +  R_{f_i} c_i
        ///
        /// where `t_j` is joint j's fixed parent offset and `c_i` the sphere's centre in
        /// its own frame, so the triangle inequality gives
        ///
        ///     |p_i - o_k| <= sum_{j=k+1}^{f_i - 1} |t_j|  +  |c_i|
        ///
        /// independent of q. Note which joints drop out and why: joints below k move the
        /// axis and the sphere together, joint k rotates the sphere *about* the axis, and
        /// joints past `f_i` do not move the sphere at all — none of them can change the
        /// distance. Only the joints strictly between k and the sphere's frame can, and
        /// those are exactly the terms in the sum. Zero for k >= f_i, since a downstream
        /// joint cannot move the sphere.
        ///
        /// It is sound but not tight: bounding the distance-to-axis by the full distance
        /// discards the component along the axis, and the straight-chain sum overshoots
        /// what the joint limits actually reach. For sphere-on-frame-6 versus joint 0 it
        /// gives ~1.16 m against a true horizontal reach nearer 0.85 m. Tightening it
        /// makes every user of it strictly better, so it is worth doing eventually --
        /// per-joint-limit interval arithmetic over the intervening joints would.
        static const Eigen::Matrix<double, nSpheres, nJoints> &leverArmBounds()
        {
            static const Eigen::Matrix<double, nSpheres, nJoints> table = []
            {
                Eigen::Matrix<double, nSpheres, nJoints> bounds;
                bounds.setZero();
                for (std::size_t i = 0; i < nSpheres; ++i)
                {
                    const std::size_t frame = spheres()[i].frame;
                    // Walk k downward, accumulating the offsets newly enclosed each time.
                    double reach = spheres()[i].center.norm();
                    for (std::size_t k = frame; k-- > 0;)
                    {
                        // Joint k+1's offset separates the sphere from axis k but not from
                        // axis k+1, so it enters here rather than on the previous step.
                        if (k + 1 < frame)
                            reach += jointOrigins()[k + 1].translation.norm();
                        bounds(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(k)) = reach;
                    }
                }
                return bounds;
            }();
            return table;
        }

        /// Exact d(center_i)/dq. Column k is the standard revolute-joint term
        /// axis_k x (p_i - origin_k) for the joints upstream of sphere i, and zero
        /// for the joints downstream of it (they cannot move it).
        static PositionJacobian sphereJacobian(const Kinematics &kin, std::size_t i)
        {
            const std::size_t frame = spheres()[i].frame;
            const Eigen::Vector3d p = sphereCenter(kin, i);

            PositionJacobian jacobian = PositionJacobian::Zero();
            for (std::size_t k = 0; k < frame; ++k)  // frame f is moved by joints 0..f-1
                jacobian.col(static_cast<Eigen::Index>(k)) = kin.jointAxis[k].cross(p - kin.jointOrigin[k]);
            return jacobian;
        }

        /// Gradient of h_i = d(p_i(q)) - r_i with respect to q, given the workspace
        /// SDF gradient at the sphere center. This is the row a CBF constraint on
        /// sphere i contributes; `sdfGradient` comes straight from
        /// `ompl::sdf::GridSDF::gradient()`.
        ///
        /// Equal to `sphereJacobian(kin, i).transpose() * sdfGradient` to the last bit,
        /// but it never forms the Jacobian. Expanding the scalar triple product,
        ///
        ///     (a_k x (p - o_k)) . g  =  a_k . (p x g)  -  g . (a_k x o_k)
        ///
        /// leaves one cross product per sphere instead of one per (sphere, joint) pair,
        /// because `a_k x o_k` is `kin.jointMoment[k]` — already computed once by the
        /// forward kinematics. This is the hot path: a CBF step asks for all 40 rows, so
        /// it runs 40 times per filtered control, and measures ~2.5x faster than building
        /// and contracting the Jacobians. Use `sphereJacobian()` when you want the
        /// Jacobian itself rather than this contraction of it.
        static Configuration barrierGradient(const Kinematics &kin, std::size_t i,
                                             const Eigen::Vector3d &sdfGradient)
        {
            const std::size_t frame = spheres()[i].frame;
            const Eigen::Vector3d moment = sphereCenter(kin, i).cross(sdfGradient);

            Configuration row = Configuration::Zero();
            for (std::size_t k = 0; k < frame; ++k)  // frame f is moved by joints 0..f-1
                row[static_cast<Eigen::Index>(k)] =
                    kin.jointAxis[k].dot(moment) - kin.jointMoment[k].dot(sdfGradient);
            return row;
        }

        /// A pair of spheres the arm can drive into each other, ordered so that sphere
        /// `a` sits on the earlier frame. Every user of `selfPairs()` relies on that
        /// ordering; see `selfPairLeverArms()` for what it buys.
        struct SelfPair
        {
            std::size_t a;
            std::size_t b;
            /// How much clearance this pair must hold for the *meshes* to be clear.
            ///
            /// The spheres are not an outer bound on the links, so `h_ab >= 0` does not
            /// mean the arm is clear of itself: it means these two spheres are. This is
            /// the gap, measured against the bodies PyBullet actually checks -- the convex
            /// hulls of the collision meshes -- as the largest sphere clearance ever seen
            /// while those hulls overlapped. See `scripts/calibrate_self_margins.py`.
            ///
            /// It is per pair because the deficit is not uniform: `forearm_link` and
            /// `upper_arm_link` carry meshes whose hulls are half again their own volume,
            /// and one margin large enough for those would exceed what most other pairs
            /// are ever clear by, making the QP infeasible everywhere.
            ///
            /// Most pairs are zero. Only one pair per link pair carries the calibration --
            /// the one needing the smallest margin -- because the barrier is a conjunction
            /// and a collision only needs one row to fire.
            double margin;
        };

        static constexpr std::size_t nSelfPairs = 303;

        /// The sphere pairs worth constraining, out of the 780 the model admits.
        ///
        /// A CBF built only on a workspace field cannot see the arm folding through
        /// itself, so self-collision enters as extra barriers of the same shape:
        ///
        ///     h_ab(q) = |p_a(q) - p_b(q)| - r_a - r_b - margin
        ///
        /// Most pairs must not get one, and for two opposite reasons. A pair the
        /// geometry never lets *separate* — the 18 gripper spheres against each other,
        /// `wrist_1` against `wrist_3` — yields a row that is either infeasible at every
        /// configuration or permanently binding at every configuration, and the planner
        /// can do nothing about either. A pair that never *approaches* yields a row that
        /// can never bind, which is not merely wasted work: `certifiedDuration()` takes a
        /// minimum over all rows, so a vacuous row's clearance shortens the certified
        /// step exactly as a dangerous one's would, and the long certified hops are what
        /// pay for this planner's wall time.
        ///
        /// So a pair earns a row only if its clearance crosses a 50 mm band somewhere in
        /// configuration space. `scripts/generate_ur5_self_pairs.py` derives the table by
        /// sampling, and checks the one thing that would make the result unsound: no
        /// dropped pair both overlaps somewhere and stays inside the band everywhere. At
        /// 50 mm none does — every pair dropped as never-separating either always
        /// overlaps, so no barrier could have helped it, or never overlaps at all.
        ///
        /// The band is also the price. No kept pair is ever clearer than 58.2 mm
        /// (`forearm` sphere 15 against `wrist_2` sphere 21), so once these rows are in,
        /// no certificate can exceed what 58.2 mm of clearance buys, however empty the
        /// workspace is.
        static const std::array<SelfPair, nSelfPairs> &selfPairs()
        {
            static const std::array<SelfPair, nSelfPairs> table{{
                // clang-format off
                {0, 2, 0.021920},  // base_link vs upper_arm_link, h in [-23.2, 77.0] mm
                {0, 3, 0.000000},  // base_link vs upper_arm_link, h in [21.8, 168.6] mm
                {0, 10, 0.000000},  // base_link vs forearm_link, h in [36.7, 574.3] mm
                {0, 11, 0.000000},  // base_link vs forearm_link, h in [-3.0, 614.3] mm
                {0, 12, 0.000000},  // base_link vs forearm_link, h in [-42.5, 654.3] mm
                {0, 13, 0.000000},  // base_link vs forearm_link, h in [-80.7, 694.2] mm
                {0, 14, 0.000000},  // base_link vs forearm_link, h in [-103.8, 734.2] mm
                {0, 15, 0.010004},  // base_link vs forearm_link, h in [-103.8, 774.2] mm
                {0, 16, 0.000000},  // base_link vs wrist_1_link, h in [-13.8, 821.6] mm
                {0, 17, 0.025403},  // base_link vs wrist_1_link, h in [-13.8, 822.2] mm
                {0, 18, 0.000000},  // base_link vs wrist_1_link, h in [-13.8, 792.5] mm
                {0, 19, 0.000000},  // base_link vs wrist_2_link, h in [-40.4, 887.7] mm
                {0, 20, 0.000000},  // base_link vs wrist_2_link, h in [-40.7, 886.9] mm
                {0, 21, 0.034145},  // base_link vs wrist_2_link, h in [-10.8, 881.6] mm
                {0, 22, 0.009713},  // base_link vs wrist_3_link, h in [-69.7, 903.6] mm
                {0, 23, 0.007828},  // base_link vs fts_robotside, h in [-106.6, 925.1] mm
                {0, 24, 0.000000},  // base_link vs robotiq_85_base_link, h in [-108.5, 992.5] mm
                {0, 25, 0.023608},  // base_link vs robotiq_85_base_link, h in [-111.3, 957.3] mm
                {0, 26, 0.010118},  // base_link vs robotiq_85_left_knuckle_link, h in [-83.9, 1024.6] mm
                {0, 27, 0.000000},  // base_link vs robotiq_85_left_finger_link, h in [-72.6, 1078.1] mm
                {0, 28, 0.000000},  // base_link vs robotiq_85_left_finger_link, h in [-86.8, 1048.9] mm
                {0, 29, 0.000000},  // base_link vs robotiq_85_left_finger_link, h in [-81.0, 1063.4] mm
                {0, 30, 0.010391},  // base_link vs robotiq_85_left_inner_knuckle_link, h in [-89.4, 1048.6] mm
                {0, 31, 0.000000},  // base_link vs robotiq_85_left_finger_tip_link, h in [-93.1, 1102.9] mm
                {0, 32, 0.000000},  // base_link vs robotiq_85_left_finger_tip_link, h in [-71.1, 1080.8] mm
                {0, 33, 0.028676},  // base_link vs robotiq_85_right_inner_knuckle_link, h in [-90.5, 1043.2] mm
                {0, 34, 0.000000},  // base_link vs robotiq_85_right_finger_tip_link, h in [-68.3, 1096.4] mm
                {0, 35, 0.010179},  // base_link vs robotiq_85_right_finger_tip_link, h in [-74.1, 1076.4] mm
                {0, 36, 0.002407},  // base_link vs robotiq_85_right_knuckle_link, h in [-91.1, 1021.2] mm
                {0, 37, 0.000000},  // base_link vs robotiq_85_right_finger_link, h in [-83.1, 1073.8] mm
                {0, 38, 0.000000},  // base_link vs robotiq_85_right_finger_link, h in [-85.3, 1043.2] mm
                {0, 39, 0.006211},  // base_link vs robotiq_85_right_finger_link, h in [-83.9, 1058.2] mm
                {1, 13, 0.000000},  // shoulder_link vs forearm_link, h in [6.0, 605.2] mm
                {1, 14, 0.000000},  // shoulder_link vs forearm_link, h in [-33.5, 645.2] mm
                {1, 15, 0.008721},  // shoulder_link vs forearm_link, h in [-72.2, 685.2] mm
                {1, 16, 0.000000},  // shoulder_link vs wrist_1_link, h in [-13.8, 733.8] mm
                {1, 17, 0.000000},  // shoulder_link vs wrist_1_link, h in [-13.8, 733.9] mm
                {1, 18, 0.031474},  // shoulder_link vs wrist_1_link, h in [-8.9, 704.1] mm
                {1, 19, 0.000000},  // shoulder_link vs wrist_2_link, h in [-40.0, 799.3] mm
                {1, 20, 0.000000},  // shoulder_link vs wrist_2_link, h in [-39.1, 799.2] mm
                {1, 21, 0.033830},  // shoulder_link vs wrist_2_link, h in [-10.8, 793.8] mm
                {1, 22, 0.009215},  // shoulder_link vs wrist_3_link, h in [-70.2, 816.3] mm
                {1, 23, 0.007381},  // shoulder_link vs fts_robotside, h in [-107.3, 840.2] mm
                {1, 24, 0.033399},  // shoulder_link vs robotiq_85_base_link, h in [-106.4, 904.6] mm
                {1, 25, 0.000000},  // shoulder_link vs robotiq_85_base_link, h in [-113.1, 870.9] mm
                {1, 26, 0.015503},  // shoulder_link vs robotiq_85_left_knuckle_link, h in [-94.3, 941.4] mm
                {1, 27, 0.000000},  // shoulder_link vs robotiq_85_left_finger_link, h in [-73.1, 995.7] mm
                {1, 28, 0.000000},  // shoulder_link vs robotiq_85_left_finger_link, h in [-74.3, 965.7] mm
                {1, 29, 0.004203},  // shoulder_link vs robotiq_85_left_finger_link, h in [-76.7, 980.2] mm
                {1, 30, 0.024251},  // shoulder_link vs robotiq_85_left_inner_knuckle_link, h in [-92.1, 963.8] mm
                {1, 31, 0.000000},  // shoulder_link vs robotiq_85_left_finger_tip_link, h in [-84.2, 1018.9] mm
                {1, 32, 0.010010},  // shoulder_link vs robotiq_85_left_finger_tip_link, h in [-81.4, 997.9] mm
                {1, 33, 0.003926},  // shoulder_link vs robotiq_85_right_inner_knuckle_link, h in [-91.0, 963.5] mm
                {1, 34, 0.011636},  // shoulder_link vs robotiq_85_right_finger_tip_link, h in [-68.6, 1020.1] mm
                {1, 35, 0.000000},  // shoulder_link vs robotiq_85_right_finger_tip_link, h in [-73.2, 997.4] mm
                {1, 36, 0.004334},  // shoulder_link vs robotiq_85_right_knuckle_link, h in [-85.9, 941.6] mm
                {1, 37, 0.000000},  // shoulder_link vs robotiq_85_right_finger_link, h in [-85.4, 996.3] mm
                {1, 38, 0.016037},  // shoulder_link vs robotiq_85_right_finger_link, h in [-87.5, 966.5] mm
                {1, 39, 0.000000},  // shoulder_link vs robotiq_85_right_finger_link, h in [-81.2, 981.3] mm
                {2, 11, 0.000000},  // upper_arm_link vs forearm_link, h in [36.0, 433.1] mm
                {2, 12, 0.000000},  // upper_arm_link vs forearm_link, h in [13.9, 472.2] mm
                {2, 13, 0.000000},  // upper_arm_link vs forearm_link, h in [1.4, 511.4] mm
                {2, 14, 0.000000},  // upper_arm_link vs forearm_link, h in [1.4, 550.8] mm
                {2, 15, 0.000000},  // upper_arm_link vs forearm_link, h in [13.9, 590.2] mm
                {2, 16, 0.000000},  // upper_arm_link vs wrist_1_link, h in [-68.4, 622.8] mm
                {2, 17, 0.000000},  // upper_arm_link vs wrist_1_link, h in [-68.3, 622.8] mm
                {2, 18, 0.000000},  // upper_arm_link vs wrist_1_link, h in [-41.9, 592.9] mm
                {2, 19, 0.000000},  // upper_arm_link vs wrist_2_link, h in [-116.9, 687.2] mm
                {2, 20, 0.000000},  // upper_arm_link vs wrist_2_link, h in [-113.3, 687.6] mm
                {2, 21, 0.063413},  // upper_arm_link vs wrist_2_link, h in [-93.3, 682.7] mm
                {2, 22, 0.000000},  // upper_arm_link vs wrist_3_link, h in [-115.4, 704.5] mm
                {2, 23, 0.000000},  // upper_arm_link vs fts_robotside, h in [-116.2, 728.0] mm
                {2, 24, 0.000000},  // upper_arm_link vs robotiq_85_base_link, h in [-102.0, 793.4] mm
                {2, 25, 0.084942},  // upper_arm_link vs robotiq_85_base_link, h in [-107.9, 758.6] mm
                {2, 26, 0.074331},  // upper_arm_link vs robotiq_85_left_knuckle_link, h in [-87.0, 830.3] mm
                {2, 27, 0.000000},  // upper_arm_link vs robotiq_85_left_finger_link, h in [-89.1, 884.8] mm
                {2, 28, 0.125614},  // upper_arm_link vs robotiq_85_left_finger_link, h in [-92.2, 853.3] mm
                {2, 29, 0.000000},  // upper_arm_link vs robotiq_85_left_finger_link, h in [-85.1, 869.0] mm
                {2, 30, 0.109862},  // upper_arm_link vs robotiq_85_left_inner_knuckle_link, h in [-78.4, 852.8] mm
                {2, 31, 0.000000},  // upper_arm_link vs robotiq_85_left_finger_tip_link, h in [-82.5, 907.9] mm
                {2, 32, 0.130939},  // upper_arm_link vs robotiq_85_left_finger_tip_link, h in [-81.0, 886.9] mm
                {2, 33, 0.101690},  // upper_arm_link vs robotiq_85_right_inner_knuckle_link, h in [-86.3, 854.4] mm
                {2, 34, 0.000000},  // upper_arm_link vs robotiq_85_right_finger_tip_link, h in [-83.1, 910.9] mm
                {2, 35, 0.143073},  // upper_arm_link vs robotiq_85_right_finger_tip_link, h in [-81.5, 888.8] mm
                {2, 36, 0.058171},  // upper_arm_link vs robotiq_85_right_knuckle_link, h in [-89.1, 830.0] mm
                {2, 37, 0.106956},  // upper_arm_link vs robotiq_85_right_finger_link, h in [-71.1, 883.9] mm
                {2, 38, 0.000000},  // upper_arm_link vs robotiq_85_right_finger_link, h in [-76.7, 850.6] mm
                {2, 39, 0.000000},  // upper_arm_link vs robotiq_85_right_finger_link, h in [-81.1, 866.4] mm
                {3, 9, 0.000000},  // upper_arm_link vs forearm_link, h in [21.3, 254.6] mm
                {3, 10, 0.000000},  // upper_arm_link vs forearm_link, h in [4.7, 292.7] mm
                {3, 11, 0.000000},  // upper_arm_link vs forearm_link, h in [-0.2, 331.2] mm
                {3, 12, 0.000000},  // upper_arm_link vs forearm_link, h in [7.9, 369.9] mm
                {3, 13, 0.000000},  // upper_arm_link vs forearm_link, h in [26.8, 408.7] mm
                {3, 16, 0.000000},  // upper_arm_link vs wrist_1_link, h in [30.2, 517.9] mm
                {3, 17, 0.000000},  // upper_arm_link vs wrist_1_link, h in [30.3, 517.9] mm
                {3, 19, 0.000000},  // upper_arm_link vs wrist_2_link, h in [-35.2, 582.3] mm
                {3, 20, 0.000000},  // upper_arm_link vs wrist_2_link, h in [-35.6, 582.7] mm
                {3, 21, 0.000000},  // upper_arm_link vs wrist_2_link, h in [-28.7, 577.7] mm
                {3, 22, 0.000000},  // upper_arm_link vs wrist_3_link, h in [-51.5, 599.7] mm
                {3, 23, 0.000000},  // upper_arm_link vs fts_robotside, h in [-73.6, 623.2] mm
                {3, 24, 0.000000},  // upper_arm_link vs robotiq_85_base_link, h in [-111.3, 688.5] mm
                {3, 25, 0.000000},  // upper_arm_link vs robotiq_85_base_link, h in [-105.0, 653.7] mm
                {3, 26, 0.000000},  // upper_arm_link vs robotiq_85_left_knuckle_link, h in [-88.9, 725.6] mm
                {3, 27, 0.000000},  // upper_arm_link vs robotiq_85_left_finger_link, h in [-85.6, 780.0] mm
                {3, 28, 0.000000},  // upper_arm_link vs robotiq_85_left_finger_link, h in [-86.8, 748.6] mm
                {3, 29, 0.000000},  // upper_arm_link vs robotiq_85_left_finger_link, h in [-92.0, 764.2] mm
                {3, 30, 0.000000},  // upper_arm_link vs robotiq_85_left_inner_knuckle_link, h in [-87.8, 748.1] mm
                {3, 31, 0.000000},  // upper_arm_link vs robotiq_85_left_finger_tip_link, h in [-73.9, 803.6] mm
                {3, 32, 0.000000},  // upper_arm_link vs robotiq_85_left_finger_tip_link, h in [-76.2, 782.4] mm
                {3, 33, 0.000000},  // upper_arm_link vs robotiq_85_right_inner_knuckle_link, h in [-88.4, 749.5] mm
                {3, 34, 0.000000},  // upper_arm_link vs robotiq_85_right_finger_tip_link, h in [-77.8, 806.0] mm
                {3, 35, 0.000000},  // upper_arm_link vs robotiq_85_right_finger_tip_link, h in [-89.4, 783.9] mm
                {3, 36, 0.000000},  // upper_arm_link vs robotiq_85_right_knuckle_link, h in [-88.9, 725.1] mm
                {3, 37, 0.000000},  // upper_arm_link vs robotiq_85_right_finger_link, h in [-84.8, 779.0] mm
                {3, 38, 0.000000},  // upper_arm_link vs robotiq_85_right_finger_link, h in [-85.1, 746.2] mm
                {3, 39, 0.000000},  // upper_arm_link vs robotiq_85_right_finger_link, h in [-82.0, 761.6] mm
                {4, 8, 0.000000},  // upper_arm_link vs forearm_link, h in [0.1, 121.7] mm
                {4, 9, 0.000000},  // upper_arm_link vs forearm_link, h in [3.4, 157.2] mm
                {4, 10, 0.000000},  // upper_arm_link vs forearm_link, h in [18.7, 193.7] mm
                {4, 23, 0.000000},  // upper_arm_link vs fts_robotside, h in [29.2, 518.4] mm
                {4, 24, 0.000000},  // upper_arm_link vs robotiq_85_base_link, h in [-36.7, 583.6] mm
                {4, 25, 0.000000},  // upper_arm_link vs robotiq_85_base_link, h in [-3.5, 548.9] mm
                {4, 26, 0.000000},  // upper_arm_link vs robotiq_85_left_knuckle_link, h in [-34.0, 620.9] mm
                {4, 27, 0.000000},  // upper_arm_link vs robotiq_85_left_finger_link, h in [-77.2, 675.3] mm
                {4, 28, 0.000000},  // upper_arm_link vs robotiq_85_left_finger_link, h in [-44.4, 644.0] mm
                {4, 29, 0.000000},  // upper_arm_link vs robotiq_85_left_finger_link, h in [-62.6, 659.5] mm
                {4, 30, 0.000000},  // upper_arm_link vs robotiq_85_left_inner_knuckle_link, h in [-53.2, 643.7] mm
                {4, 31, 0.000000},  // upper_arm_link vs robotiq_85_left_finger_tip_link, h in [-79.9, 699.6] mm
                {4, 32, 0.000000},  // upper_arm_link vs robotiq_85_left_finger_tip_link, h in [-70.9, 678.1] mm
                {4, 33, 0.000000},  // upper_arm_link vs robotiq_85_right_inner_knuckle_link, h in [-54.4, 644.6] mm
                {4, 34, 0.000000},  // upper_arm_link vs robotiq_85_right_finger_tip_link, h in [-86.4, 701.1] mm
                {4, 35, 0.000000},  // upper_arm_link vs robotiq_85_right_finger_tip_link, h in [-68.7, 679.0] mm
                {4, 36, 0.000000},  // upper_arm_link vs robotiq_85_right_knuckle_link, h in [-32.7, 620.2] mm
                {4, 37, 0.000000},  // upper_arm_link vs robotiq_85_right_finger_link, h in [-71.9, 674.1] mm
                {4, 38, 0.000000},  // upper_arm_link vs robotiq_85_right_finger_link, h in [-41.1, 642.1] mm
                {4, 39, 0.000000},  // upper_arm_link vs robotiq_85_right_finger_link, h in [-58.7, 656.8] mm
                {5, 27, 0.000000},  // upper_arm_link vs robotiq_85_left_finger_link, h in [25.5, 570.7] mm
                {5, 31, 0.000000},  // upper_arm_link vs robotiq_85_left_finger_tip_link, h in [3.9, 595.7] mm
                {5, 32, 0.000000},  // upper_arm_link vs robotiq_85_left_finger_tip_link, h in [24.2, 574.1] mm
                {5, 34, 0.000000},  // upper_arm_link vs robotiq_85_right_finger_tip_link, h in [3.0, 596.3] mm
                {5, 35, 0.000000},  // upper_arm_link vs robotiq_85_right_finger_tip_link, h in [22.0, 574.4] mm
                {5, 37, 0.000000},  // upper_arm_link vs robotiq_85_right_finger_link, h in [25.9, 570.6] mm
                {6, 14, 0.000000},  // upper_arm_link vs forearm_link, h in [26.8, 654.3] mm
                {6, 15, 0.000000},  // upper_arm_link vs forearm_link, h in [7.9, 693.9] mm
                {6, 16, 0.000000},  // upper_arm_link vs wrist_1_link, h in [-90.1, 727.7] mm
                {6, 17, 0.000000},  // upper_arm_link vs wrist_1_link, h in [-90.2, 727.7] mm
                {6, 18, 0.014210},  // upper_arm_link vs wrist_1_link, h in [-75.8, 697.8] mm
                {6, 19, 0.000000},  // upper_arm_link vs wrist_2_link, h in [-115.3, 792.2] mm
                {6, 20, 0.000000},  // upper_arm_link vs wrist_2_link, h in [-114.6, 792.5] mm
                {6, 21, 0.000000},  // upper_arm_link vs wrist_2_link, h in [-93.3, 787.6] mm
                {6, 22, 0.067373},  // upper_arm_link vs wrist_3_link, h in [-108.4, 809.4] mm
                {6, 23, 0.078318},  // upper_arm_link vs fts_robotside, h in [-106.5, 832.9] mm
                {6, 24, 0.000000},  // upper_arm_link vs robotiq_85_base_link, h in [-96.9, 898.4] mm
                {6, 25, 0.000000},  // upper_arm_link vs robotiq_85_base_link, h in [-112.5, 863.4] mm
                {6, 26, 0.000000},  // upper_arm_link vs robotiq_85_left_knuckle_link, h in [-87.2, 935.1] mm
                {6, 27, 0.000000},  // upper_arm_link vs robotiq_85_left_finger_link, h in [-79.7, 989.6] mm
                {6, 28, 0.000000},  // upper_arm_link vs robotiq_85_left_finger_link, h in [-87.3, 958.1] mm
                {6, 29, 0.000000},  // upper_arm_link vs robotiq_85_left_finger_link, h in [-87.5, 973.8] mm
                {6, 30, 0.000000},  // upper_arm_link vs robotiq_85_left_inner_knuckle_link, h in [-86.5, 957.5] mm
                {6, 31, 0.000000},  // upper_arm_link vs robotiq_85_left_finger_tip_link, h in [-71.5, 1012.3] mm
                {6, 32, 0.000000},  // upper_arm_link vs robotiq_85_left_finger_tip_link, h in [-89.1, 991.7] mm
                {6, 33, 0.000000},  // upper_arm_link vs robotiq_85_right_inner_knuckle_link, h in [-92.6, 959.3] mm
                {6, 34, 0.000000},  // upper_arm_link vs robotiq_85_right_finger_tip_link, h in [-74.1, 1015.8] mm
                {6, 35, 0.000000},  // upper_arm_link vs robotiq_85_right_finger_tip_link, h in [-75.6, 993.7] mm
                {6, 36, 0.000000},  // upper_arm_link vs robotiq_85_right_knuckle_link, h in [-74.0, 935.0] mm
                {6, 37, 0.000000},  // upper_arm_link vs robotiq_85_right_finger_link, h in [-69.6, 988.8] mm
                {6, 38, 0.000000},  // upper_arm_link vs robotiq_85_right_finger_link, h in [-85.7, 955.3] mm
                {6, 39, 0.000000},  // upper_arm_link vs robotiq_85_right_finger_link, h in [-81.6, 971.3] mm
                {7, 31, 0.000000},  // forearm_link vs robotiq_85_left_finger_tip_link, h in [14.8, 601.1] mm
                {7, 32, 0.000000},  // forearm_link vs robotiq_85_left_finger_tip_link, h in [36.6, 579.6] mm
                {7, 34, 0.000000},  // forearm_link vs robotiq_85_right_finger_tip_link, h in [14.5, 601.9] mm
                {7, 35, 0.000000},  // forearm_link vs robotiq_85_right_finger_tip_link, h in [36.5, 580.2] mm
                {7, 37, 0.000000},  // forearm_link vs robotiq_85_right_finger_link, h in [39.7, 577.0] mm
                {8, 24, 0.000000},  // forearm_link vs robotiq_85_base_link, h in [25.6, 427.9] mm
                {8, 26, 0.000000},  // forearm_link vs robotiq_85_left_knuckle_link, h in [27.8, 465.8] mm
                {8, 27, 0.000000},  // forearm_link vs robotiq_85_left_finger_link, h in [-16.6, 520.1] mm
                {8, 28, 0.000000},  // forearm_link vs robotiq_85_left_finger_link, h in [15.0, 489.1] mm
                {8, 29, 0.000000},  // forearm_link vs robotiq_85_left_finger_link, h in [-0.7, 504.2] mm
                {8, 30, 0.000000},  // forearm_link vs robotiq_85_left_inner_knuckle_link, h in [4.3, 489.1] mm
                {8, 31, 0.000000},  // forearm_link vs robotiq_85_left_finger_tip_link, h in [-41.1, 545.1] mm
                {8, 32, 0.000000},  // forearm_link vs robotiq_85_left_finger_tip_link, h in [-19.7, 523.5] mm
                {8, 33, 0.000000},  // forearm_link vs robotiq_85_right_inner_knuckle_link, h in [4.3, 489.3] mm
                {8, 34, 0.000000},  // forearm_link vs robotiq_85_right_finger_tip_link, h in [-41.6, 545.3] mm
                {8, 35, 0.000000},  // forearm_link vs robotiq_85_right_finger_tip_link, h in [-20.0, 523.7] mm
                {8, 36, 0.000000},  // forearm_link vs robotiq_85_right_knuckle_link, h in [27.6, 465.9] mm
                {8, 37, 0.000000},  // forearm_link vs robotiq_85_right_finger_link, h in [-15.6, 520.5] mm
                {8, 38, 0.000000},  // forearm_link vs robotiq_85_right_finger_link, h in [15.0, 489.1] mm
                {8, 39, 0.000000},  // forearm_link vs robotiq_85_right_finger_link, h in [-0.7, 504.5] mm
                {9, 24, 0.000000},  // forearm_link vs robotiq_85_base_link, h in [-12.2, 390.0] mm
                {9, 25, 0.000000},  // forearm_link vs robotiq_85_base_link, h in [21.9, 355.9] mm
                {9, 26, 0.000000},  // forearm_link vs robotiq_85_left_knuckle_link, h in [-8.8, 428.0] mm
                {9, 27, 0.000000},  // forearm_link vs robotiq_85_left_finger_link, h in [-48.9, 482.3] mm
                {9, 28, 0.000000},  // forearm_link vs robotiq_85_left_finger_link, h in [-21.8, 451.1] mm
                {9, 29, 0.000000},  // forearm_link vs robotiq_85_left_finger_link, h in [-38.2, 466.4] mm
                {9, 30, 0.000000},  // forearm_link vs robotiq_85_left_inner_knuckle_link, h in [-33.7, 451.4] mm
                {9, 31, 0.000000},  // forearm_link vs robotiq_85_left_finger_tip_link, h in [-48.7, 507.5] mm
                {9, 32, 0.000000},  // forearm_link vs robotiq_85_left_finger_tip_link, h in [-51.2, 485.8] mm
                {9, 33, 0.000000},  // forearm_link vs robotiq_85_right_inner_knuckle_link, h in [-33.7, 451.5] mm
                {9, 34, 0.049534},  // forearm_link vs robotiq_85_right_finger_tip_link, h in [-51.9, 507.6] mm
                {9, 35, 0.000000},  // forearm_link vs robotiq_85_right_finger_tip_link, h in [-52.9, 486.0] mm
                {9, 36, 0.000000},  // forearm_link vs robotiq_85_right_knuckle_link, h in [-8.9, 428.1] mm
                {9, 37, 0.000000},  // forearm_link vs robotiq_85_right_finger_link, h in [-51.6, 482.6] mm
                {9, 38, 0.000000},  // forearm_link vs robotiq_85_right_finger_link, h in [-22.1, 451.4] mm
                {9, 39, 0.000000},  // forearm_link vs robotiq_85_right_finger_link, h in [-39.1, 466.7] mm
                {10, 22, 0.000000},  // forearm_link vs wrist_3_link, h in [39.7, 263.8] mm
                {10, 23, 0.000000},  // forearm_link vs fts_robotside, h in [16.0, 287.5] mm
                {10, 24, 0.000000},  // forearm_link vs robotiq_85_base_link, h in [-49.0, 352.9] mm
                {10, 25, 0.000000},  // forearm_link vs robotiq_85_base_link, h in [-15.3, 318.8] mm
                {10, 26, 0.000000},  // forearm_link vs robotiq_85_left_knuckle_link, h in [-45.8, 390.6] mm
                {10, 27, 0.000000},  // forearm_link vs robotiq_85_left_finger_link, h in [-48.9, 445.0] mm
                {10, 28, 0.000000},  // forearm_link vs robotiq_85_left_finger_link, h in [-51.4, 414.1] mm
                {10, 29, 0.000000},  // forearm_link vs robotiq_85_left_finger_link, h in [-49.8, 429.2] mm
                {10, 30, 0.000000},  // forearm_link vs robotiq_85_left_inner_knuckle_link, h in [-53.1, 414.0] mm
                {10, 31, 0.000000},  // forearm_link vs robotiq_85_left_finger_tip_link, h in [-24.7, 470.3] mm
                {10, 32, 0.032215},  // forearm_link vs robotiq_85_left_finger_tip_link, h in [-48.3, 448.4] mm
                {10, 33, 0.000000},  // forearm_link vs robotiq_85_right_inner_knuckle_link, h in [-55.9, 414.4] mm
                {10, 34, 0.000000},  // forearm_link vs robotiq_85_right_finger_tip_link, h in [-24.7, 470.4] mm
                {10, 35, 0.000000},  // forearm_link vs robotiq_85_right_finger_tip_link, h in [-48.3, 448.8] mm
                {10, 36, 0.000000},  // forearm_link vs robotiq_85_right_knuckle_link, h in [-44.7, 390.9] mm
                {10, 37, 0.000000},  // forearm_link vs robotiq_85_right_finger_link, h in [-47.7, 445.5] mm
                {10, 38, 0.000000},  // forearm_link vs robotiq_85_right_finger_link, h in [-49.1, 414.3] mm
                {10, 39, 0.050687},  // forearm_link vs robotiq_85_right_finger_link, h in [-47.4, 429.7] mm
                {11, 19, 0.000000},  // forearm_link vs wrist_2_link, h in [23.6, 209.7] mm
                {11, 20, 0.000000},  // forearm_link vs wrist_2_link, h in [23.6, 209.7] mm
                {11, 22, 0.000000},  // forearm_link vs wrist_3_link, h in [3.7, 227.8] mm
                {11, 23, 0.000000},  // forearm_link vs fts_robotside, h in [-20.0, 251.5] mm
                {11, 24, 0.000000},  // forearm_link vs robotiq_85_base_link, h in [-74.5, 316.9] mm
                {11, 25, 0.000000},  // forearm_link vs robotiq_85_base_link, h in [-51.1, 282.8] mm
                {11, 26, 0.045551},  // forearm_link vs robotiq_85_left_knuckle_link, h in [-54.5, 354.5] mm
                {11, 27, 0.000000},  // forearm_link vs robotiq_85_left_finger_link, h in [-28.6, 408.9] mm
                {11, 28, 0.053129},  // forearm_link vs robotiq_85_left_finger_link, h in [-48.6, 378.0] mm
                {11, 29, 0.000000},  // forearm_link vs robotiq_85_left_finger_link, h in [-46.6, 393.2] mm
                {11, 30, 0.045887},  // forearm_link vs robotiq_85_left_inner_knuckle_link, h in [-39.0, 378.2] mm
                {11, 31, 0.000000},  // forearm_link vs robotiq_85_left_finger_tip_link, h in [10.9, 434.2] mm
                {11, 32, 0.000000},  // forearm_link vs robotiq_85_left_finger_tip_link, h in [-13.3, 412.5] mm
                {11, 33, 0.049553},  // forearm_link vs robotiq_85_right_inner_knuckle_link, h in [-39.2, 378.3] mm
                {11, 34, 0.000000},  // forearm_link vs robotiq_85_right_finger_tip_link, h in [11.4, 434.3] mm
                {11, 35, 0.000000},  // forearm_link vs robotiq_85_right_finger_tip_link, h in [-13.3, 412.7] mm
                {11, 36, 0.035961},  // forearm_link vs robotiq_85_right_knuckle_link, h in [-56.6, 355.0] mm
                {11, 37, 0.000000},  // forearm_link vs robotiq_85_right_finger_link, h in [-28.2, 409.6] mm
                {11, 38, 0.000000},  // forearm_link vs robotiq_85_right_finger_link, h in [-50.1, 378.3] mm
                {11, 39, 0.000000},  // forearm_link vs robotiq_85_right_finger_link, h in [-46.4, 393.7] mm
                {12, 19, 0.000000},  // forearm_link vs wrist_2_link, h in [-4.1, 174.0] mm
                {12, 20, 0.000000},  // forearm_link vs wrist_2_link, h in [-4.1, 174.0] mm
                {12, 21, 0.000000},  // forearm_link vs wrist_2_link, h in [22.1, 160.9] mm
                {12, 22, 0.000000},  // forearm_link vs wrist_3_link, h in [-30.0, 193.7] mm
                {12, 23, 0.000000},  // forearm_link vs fts_robotside, h in [-54.0, 217.4] mm
                {12, 24, 0.000000},  // forearm_link vs robotiq_85_base_link, h in [-40.8, 282.9] mm
                {12, 25, 0.042868},  // forearm_link vs robotiq_85_base_link, h in [-74.7, 248.7] mm
                {12, 26, 0.000000},  // forearm_link vs robotiq_85_left_knuckle_link, h in [-29.0, 320.7] mm
                {12, 27, 0.000000},  // forearm_link vs robotiq_85_left_finger_link, h in [4.5, 375.1] mm
                {12, 28, 0.000000},  // forearm_link vs robotiq_85_left_finger_link, h in [-34.3, 344.0] mm
                {12, 29, 0.000000},  // forearm_link vs robotiq_85_left_finger_link, h in [-15.1, 359.3] mm
                {12, 30, 0.000000},  // forearm_link vs robotiq_85_left_inner_knuckle_link, h in [-5.9, 344.0] mm
                {12, 32, 0.000000},  // forearm_link vs robotiq_85_left_finger_tip_link, h in [20.8, 378.3] mm
                {12, 33, 0.000000},  // forearm_link vs robotiq_85_right_inner_knuckle_link, h in [-5.4, 344.4] mm
                {12, 35, 0.000000},  // forearm_link vs robotiq_85_right_finger_tip_link, h in [20.8, 378.8] mm
                {12, 36, 0.000000},  // forearm_link vs robotiq_85_right_knuckle_link, h in [-30.5, 321.0] mm
                {12, 37, 0.000000},  // forearm_link vs robotiq_85_right_finger_link, h in [5.0, 375.6] mm
                {12, 38, 0.000000},  // forearm_link vs robotiq_85_right_finger_link, h in [-34.0, 344.2] mm
                {12, 39, 0.000000},  // forearm_link vs robotiq_85_right_finger_link, h in [-14.5, 359.5] mm
                {13, 16, 0.000000},  // forearm_link vs wrist_1_link, h in [29.4, 71.8] mm
                {13, 17, 0.000000},  // forearm_link vs wrist_1_link, h in [29.4, 71.8] mm
                {13, 19, 0.000000},  // forearm_link vs wrist_2_link, h in [-17.0, 139.9] mm
                {13, 20, 0.000000},  // forearm_link vs wrist_2_link, h in [-16.9, 139.9] mm
                {13, 21, 0.000000},  // forearm_link vs wrist_2_link, h in [13.0, 124.6] mm
                {13, 22, 0.024330},  // forearm_link vs wrist_3_link, h in [-46.9, 161.5] mm
                {13, 23, 0.026745},  // forearm_link vs fts_robotside, h in [-75.2, 186.7] mm
                {13, 24, 0.000000},  // forearm_link vs robotiq_85_base_link, h in [-10.1, 252.2] mm
                {13, 25, 0.000000},  // forearm_link vs robotiq_85_base_link, h in [-44.2, 218.0] mm
                {13, 26, 0.000000},  // forearm_link vs robotiq_85_left_knuckle_link, h in [1.0, 290.2] mm
                {13, 27, 0.000000},  // forearm_link vs robotiq_85_left_finger_link, h in [35.0, 344.5] mm
                {13, 28, 0.000000},  // forearm_link vs robotiq_85_left_finger_link, h in [-4.7, 313.3] mm
                {13, 29, 0.000000},  // forearm_link vs robotiq_85_left_finger_link, h in [15.1, 328.5] mm
                {13, 30, 0.000000},  // forearm_link vs robotiq_85_left_inner_knuckle_link, h in [25.3, 313.6] mm
                {13, 33, 0.000000},  // forearm_link vs robotiq_85_right_inner_knuckle_link, h in [24.9, 313.4] mm
                {13, 36, 0.000000},  // forearm_link vs robotiq_85_right_knuckle_link, h in [0.2, 290.2] mm
                {13, 37, 0.000000},  // forearm_link vs robotiq_85_right_finger_link, h in [35.5, 344.9] mm
                {13, 38, 0.000000},  // forearm_link vs robotiq_85_right_finger_link, h in [-4.3, 313.6] mm
                {13, 39, 0.000000},  // forearm_link vs robotiq_85_right_finger_link, h in [15.6, 328.9] mm
                {14, 16, 0.000000},  // forearm_link vs wrist_1_link, h in [12.7, 41.9] mm
                {14, 17, 0.000000},  // forearm_link vs wrist_1_link, h in [12.7, 41.9] mm
                {14, 19, 0.000000},  // forearm_link vs wrist_2_link, h in [-6.6, 108.1] mm
                {14, 20, 0.000000},  // forearm_link vs wrist_2_link, h in [-6.5, 108.1] mm
                {14, 21, 0.044711},  // forearm_link vs wrist_2_link, h in [20.4, 90.0] mm
                {14, 22, 0.000000},  // forearm_link vs wrist_3_link, h in [-26.3, 132.1] mm
                {14, 23, 0.000000},  // forearm_link vs fts_robotside, h in [-37.4, 160.4] mm
                {14, 24, 0.000000},  // forearm_link vs robotiq_85_base_link, h in [14.1, 227.9] mm
                {14, 25, 0.000000},  // forearm_link vs robotiq_85_base_link, h in [-19.0, 193.4] mm
                {14, 26, 0.000000},  // forearm_link vs robotiq_85_left_knuckle_link, h in [24.7, 265.7] mm
                {14, 28, 0.000000},  // forearm_link vs robotiq_85_left_finger_link, h in [19.5, 287.4] mm
                {14, 29, 0.000000},  // forearm_link vs robotiq_85_left_finger_link, h in [39.2, 303.5] mm
                {14, 36, 0.000000},  // forearm_link vs robotiq_85_right_knuckle_link, h in [24.6, 265.7] mm
                {14, 38, 0.000000},  // forearm_link vs robotiq_85_right_finger_link, h in [20.3, 287.7] mm
                {14, 39, 0.000000},  // forearm_link vs robotiq_85_right_finger_link, h in [39.9, 303.7] mm
                {15, 19, 0.000000},  // forearm_link vs wrist_2_link, h in [20.1, 79.9] mm
                {15, 20, 0.000000},  // forearm_link vs wrist_2_link, h in [20.1, 79.9] mm
                {15, 22, 0.000000},  // forearm_link vs wrist_3_link, h in [8.8, 106.6] mm
                {15, 23, 0.000000},  // forearm_link vs fts_robotside, h in [2.5, 138.3] mm
                {15, 24, 0.000000},  // forearm_link vs robotiq_85_base_link, h in [37.5, 210.9] mm
                {15, 25, 0.000000},  // forearm_link vs robotiq_85_base_link, h in [13.1, 174.1] mm
                {15, 28, 0.000000},  // forearm_link vs robotiq_85_left_finger_link, h in [32.7, 264.6] mm
                {15, 38, 0.000000},  // forearm_link vs robotiq_85_right_finger_link, h in [32.8, 264.8] mm
                // clang-format on
            }};
            return table;
        }

        /// `r_a + r_b` in `selfPairs()` order — the constant subtracted from each pair
        /// distance to get its clearance.
        static const Eigen::Matrix<double, nSelfPairs, 1> &selfPairRadii()
        {
            static const Eigen::Matrix<double, nSelfPairs, 1> values = []
            {
                Eigen::Matrix<double, nSelfPairs, 1> sums;
                for (std::size_t p = 0; p < nSelfPairs; ++p)
                    sums[static_cast<Eigen::Index>(p)] =
                        spheres()[selfPairs()[p].a].radius + spheres()[selfPairs()[p].b].radius;
                return sums;
            }();
            return values;
        }

        /// `SelfPair::margin` in `selfPairs()` order, so a barrier can subtract the whole
        /// column at once. Constant, so it changes no gradient and no Lipschitz bound —
        /// only where each row sits.
        static const Eigen::Matrix<double, nSelfPairs, 1> &selfPairMargins()
        {
            static const Eigen::Matrix<double, nSelfPairs, 1> values = []
            {
                Eigen::Matrix<double, nSelfPairs, 1> margins;
                for (std::size_t p = 0; p < nSelfPairs; ++p)
                    margins[static_cast<Eigen::Index>(p)] = selfPairs()[p].margin;
                return margins;
            }();
            return values;
        }

        /// Configuration-independent upper bounds on `|dh_ab/dq_k|`, the self-collision
        /// counterpart of `leverArmBounds()` and used the same way: to screen rows before
        /// any gradient is computed, and to turn a chosen control into a certified step
        /// length.
        ///
        /// The structure is sparse, and exactly so. With sphere a on frame f and sphere b
        /// on frame g, f <= g, differentiating `h_ab` gives
        ///
        ///     dh_ab/dq_k = n^T (J_a,k - J_b,k),   n = (p_a - p_b) / |p_a - p_b|
        ///
        /// and the three cases collapse to one:
        ///
        /// - `k < f`. Joint k is upstream of both frames, so it rotates the two spheres
        ///   as one rigid body and cannot change the distance between them. Formally
        ///   `J_a,k - J_b,k = axis_k x (p_a - p_b)`, and `n^T (axis_k x n)|p_a - p_b| = 0`
        ///   because the cross product is orthogonal to n. Identically zero, not small.
        /// - `f <= k < g`. Only b moves, so `|dh_ab/dq_k| = |n^T J_b,k| <= |J_b,k|`, and
        ///   `leverArmBounds()(b, k)` already bounds that over all of configuration space.
        /// - `k >= g`. Neither sphere moves; joint k is downstream of both.
        ///
        /// So the bound is `leverArmBounds()` of the *later* sphere, restricted to the
        /// joints strictly between the two frames — not, as the obvious guess has it, the
        /// sum of the two spheres' bounds. The difference is not cosmetic: the sum is
        /// loose by roughly a factor of two on every entry and, worse, is nonzero on
        /// columns where the true derivative vanishes, which drags spheres into the
        /// screened set that provably cannot move relative to each other.
        static const Eigen::Matrix<double, nSelfPairs, nJoints> &selfPairLeverArms()
        {
            static const Eigen::Matrix<double, nSelfPairs, nJoints> table = []
            {
                Eigen::Matrix<double, nSelfPairs, nJoints> bounds;
                bounds.setZero();
                for (std::size_t p = 0; p < nSelfPairs; ++p)
                {
                    const SelfPair &pair = selfPairs()[p];
                    const std::size_t first = spheres()[pair.a].frame;
                    const std::size_t second = spheres()[pair.b].frame;
                    for (std::size_t k = first; k < second; ++k)
                        bounds(static_cast<Eigen::Index>(p), static_cast<Eigen::Index>(k)) =
                            leverArmBounds()(static_cast<Eigen::Index>(pair.b),
                                             static_cast<Eigen::Index>(k));
                }
                return bounds;
            }();
            return table;
        }

        /// Surface separation of pair `p`: `|p_a - p_b| - r_a - r_b`. Subtract a margin
        /// to get the barrier value; negative means the two spheres overlap.
        static double selfPairClearance(const SphereCenters &centers, std::size_t p)
        {
            const SelfPair &pair = selfPairs()[p];
            const Eigen::Index index = static_cast<Eigen::Index>(p);
            return (centers.col(static_cast<Eigen::Index>(pair.a)) -
                    centers.col(static_cast<Eigen::Index>(pair.b)))
                       .norm() -
                   selfPairRadii()[index];
        }

        /// `dh_ab/dq` — the constraint row pair `p` contributes.
        ///
        /// Equal to `n^T (sphereJacobian(a) - sphereJacobian(b))`, but built from the
        /// derivation in `selfPairLeverArms()` instead: only columns `[f, g)` are
        /// touched, and they reduce to `-n^T J_b,k`, which is `barrierGradient()`'s
        /// contraction with n in place of an SDF gradient. Two payoffs over forming the
        /// difference honestly — one contraction instead of two, and the columns that
        /// cancel analytically come out as exact zeros rather than as rounding noise,
        /// which matters because a spurious 1e-17 in column 0 would let the QP believe
        /// the base joint can relieve a wrist-against-forearm collision.
        ///
        /// Zero when the centers coincide, where the distance is not differentiable.
        static Configuration selfPairGradient(const Kinematics &kin, const SphereCenters &centers,
                                              std::size_t p)
        {
            const SelfPair &pair = selfPairs()[p];
            const Eigen::Vector3d delta = centers.col(static_cast<Eigen::Index>(pair.a)) -
                                          centers.col(static_cast<Eigen::Index>(pair.b));
            const double distance = delta.norm();

            Configuration row = Configuration::Zero();
            if (distance <= 0.0)
                return row;

            const Eigen::Vector3d normal = delta / distance;
            const Eigen::Vector3d moment =
                centers.col(static_cast<Eigen::Index>(pair.b)).cross(normal);

            const std::size_t first = spheres()[pair.a].frame;
            const std::size_t second = spheres()[pair.b].frame;
            for (std::size_t k = first; k < second; ++k)
                row[static_cast<Eigen::Index>(k)] =
                    kin.jointMoment[k].dot(normal) - kin.jointAxis[k].dot(moment);
            return row;
        }

    private:
        /// One URDF `<joint>`: the fixed parent-to-child offset, then a rotation
        /// about `axis` by that joint's value.
        struct JointOrigin
        {
            Eigen::Matrix3d rotation;
            Eigen::Vector3d translation;
            Eigen::Vector3d axis;
        };

        /// The six revolute joints of VAMP's ur5.urdf, in chain order. Note the
        /// axes are not all Z: this URDF uses z, y, y, y, z, y.
        static const std::array<JointOrigin, nJoints> &jointOrigins()
        {
            static const std::array<JointOrigin, nJoints> origins = []
            {
                // <origin rpy="r p y"/> is R = Rz(y) Ry(p) Rx(r); every UR5 joint
                // origin here has only a pitch component, so Ry alone suffices.
                const auto pitch = [](double angle)
                { return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix(); };
                const Eigen::Matrix3d none = Eigen::Matrix3d::Identity();
                const Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
                const Eigen::Vector3d y = Eigen::Vector3d::UnitY();
                constexpr double halfPi = 1.570796325;  // as spelled in the URDF

                return std::array<JointOrigin, nJoints>{{
                    {none, {0.0, 0.0, 0.089159}, z},          // shoulder_pan_joint
                    {pitch(halfPi), {0.0, 0.13585, 0.0}, y},  // shoulder_lift_joint
                    {none, {0.0, -0.1197, 0.425}, y},         // elbow_joint
                    {pitch(halfPi), {0.0, 0.0, 0.39225}, y},  // wrist_1_joint
                    {none, {0.0, 0.093, 0.0}, z},             // wrist_2_joint
                    {none, {0.0, 0.0, 0.09465}, y},           // wrist_3_joint
                }};
            }();
            return origins;
        }

        Eigen::Isometry3d basePose_;
    };
}  // namespace ompl::robots
