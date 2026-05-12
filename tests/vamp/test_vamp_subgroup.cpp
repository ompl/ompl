/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Rice University and National University of Singapore.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holders nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#define BOOST_TEST_MODULE VampSubgroupTest
#include <boost/test/unit_test.hpp>

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <ompl/base/Constraint.h>
#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SubspaceStateSpace.h>
#include <ompl/base/spaces/constraint/AtlasStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/base/spaces/constraint/TangentBundleStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/util/Exception.h>

#include <ompl/vamp/VampMotionValidator.h>
#include <ompl/vamp/VampStateSpace.h>
#include <ompl/vamp/VampStateValidityChecker.h>
#include <ompl/vamp/VampSubgroupMotionValidator.h>
#include <ompl/vamp/VampSubgroupStateValidityChecker.h>

#include <vamp/collision/environment.hh>
#include <vamp/collision/factory.hh>
#include <vamp/robots/panda.hh>

namespace ob = ompl::base;
namespace og = ompl::geometric;

using Robot = vamp::robots::Panda;
using Environment = vamp::collision::Environment<vamp::FloatVector<vamp::FloatVectorWidth>>;

namespace
{
    // Sphere cage matching the full-body Panda demo so we can reuse known
    // start/goal poses that are known to be collision-free.
    static const std::vector<std::array<float, 3>> kObstacles = {
        {0.55, 0, 0.25},  {0.35, 0.35, 0.25},  {0, 0.55, 0.25},   {-0.55, 0, 0.25},   {-0.35, -0.35, 0.25},
        {0, -0.55, 0.25}, {0.35, -0.35, 0.25}, {0.35, 0.35, 0.8}, {0, 0.55, 0.8},     {-0.35, 0.35, 0.8},
        {-0.55, 0, 0.8},  {-0.35, -0.35, 0.8}, {0, -0.55, 0.8},   {0.35, -0.35, 0.8},
    };

    // Build the SIMD-vectorized obstacle environment once for the test module.
    auto makeEnvironment() -> Environment
    {
        vamp::collision::Environment<float> env_float;
        constexpr float radius = 0.2f;
        for (const auto &obs : kObstacles)
            env_float.spheres.emplace_back(vamp::collision::factory::sphere::array(obs, radius));
        env_float.sort();
        return Environment(env_float);
    }

    // Project the full Panda joint bounds onto a RealVectorBounds for the
    // SubspaceStateSpace constructor.
    auto makePandaBounds() -> ob::RealVectorBounds
    {
        return ompl::vamp::getRobotBounds<Robot>();
    }

    // Start pose used as the frozen reference and as a known-valid 7-DOF config.
    constexpr std::array<double, 7> kStartFull = {0., -0.785, 0., -2.356, 0., 1.571, 0.785};
    constexpr std::array<double, 7> kGoalFull = {2.35, 1., 0., -0.8, 0., 2.5, 0.785};

    auto toFrozen(const std::array<double, 7> &a) -> std::vector<double>
    {
        return std::vector<double>(a.begin(), a.end());
    }

    // Build a full-body SpaceInformation tied to the same environment so we
    // can cross-check the subgroup validators against the full-body ones.
    auto makeFullBodySI(const Environment &env)
    {
        auto space = std::make_shared<ompl::vamp::VampStateSpace<Robot>>();
        auto si = std::make_shared<ob::SpaceInformation>(space);
        si->setStateValidityChecker(std::make_shared<ompl::vamp::VampStateValidityChecker<Robot>>(si, env));
        si->setMotionValidator(std::make_shared<ompl::vamp::VampMotionValidator<Robot>>(si, env));
        si->setup();
        return std::pair{space, si};
    }

    // Build a subgroup SpaceInformation over the given active indices + frozen pose.
    auto makeSubgroupSI(const Environment &env, const std::vector<std::size_t> &active,
                        const std::vector<double> &frozen)
    {
        auto subspace = std::make_shared<ob::SubspaceStateSpace>(makePandaBounds(), active, frozen);
        auto si = std::make_shared<ob::SpaceInformation>(subspace);
        si->setStateValidityChecker(std::make_shared<ompl::vamp::VampSubgroupStateValidityChecker<Robot>>(si, env));
        si->setMotionValidator(std::make_shared<ompl::vamp::VampSubgroupMotionValidator<Robot>>(si, env));
        si->setup();
        return std::pair{subspace, si};
    }

    // Trivial constraint: pin the first active coordinate to zero. Linear,
    // so VAMP's linear edge interpolation stays on the manifold.
    class FirstCoordZero : public ob::Constraint
    {
    public:
        explicit FirstCoordZero(unsigned int dim) : ob::Constraint(dim, 1)
        {
        }

        void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
        {
            out[0] = x[0];
        }

        void jacobian(const Eigen::Ref<const Eigen::VectorXd> & /*x*/, Eigen::Ref<Eigen::MatrixXd> out) const override
        {
            out.setZero();
            out(0, 0) = 1.0;
        }
    };
}  // namespace

BOOST_AUTO_TEST_SUITE(VampSubgroup)

// ---------------------------------------------------------------------------
// Validator construction
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE(Validator_RejectsMismatchedAmbientDimension)
{
    auto env = makeEnvironment();
    // Ambient dim 5, but Robot::dimension == 7 → must throw.
    ob::RealVectorBounds wrong(5);
    for (unsigned int i = 0; i < 5; ++i)
    {
        wrong.setLow(i, -1.0);
        wrong.setHigh(i, 1.0);
    }
    auto subspace =
        std::make_shared<ob::SubspaceStateSpace>(wrong, std::vector<std::size_t>{0, 1}, std::vector<double>(5, 0.0));
    auto si = std::make_shared<ob::SpaceInformation>(subspace);

    BOOST_CHECK_THROW(ompl::vamp::VampSubgroupStateValidityChecker<Robot>(si, env), ompl::Exception);
    BOOST_CHECK_THROW(ompl::vamp::VampSubgroupMotionValidator<Robot>(si, env), ompl::Exception);
}

BOOST_AUTO_TEST_CASE(Validator_RejectsNonSubspaceStateSpace)
{
    auto env = makeEnvironment();
    // A bare VampStateSpace is not a SubspaceStateSpace.
    auto space = std::make_shared<ompl::vamp::VampStateSpace<Robot>>();
    auto si = std::make_shared<ob::SpaceInformation>(space);

    BOOST_CHECK_THROW(ompl::vamp::VampSubgroupStateValidityChecker<Robot>(si, env), ompl::Exception);
    BOOST_CHECK_THROW(ompl::vamp::VampSubgroupMotionValidator<Robot>(si, env), ompl::Exception);
}

// ---------------------------------------------------------------------------
// Subgroup vs. full-body equivalence
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE(IsValid_AgreesWithFullBodyOnEquivalentAmbientState)
{
    auto env = makeEnvironment();
    auto [full_space, full_si] = makeFullBodySI(env);

    // Active = wrist; the rest of the body stays at the canonical start pose.
    std::vector<std::size_t> active{3, 4, 5, 6};
    auto [subspace, sub_si] = makeSubgroupSI(env, active, toFrozen(kStartFull));

    // Three reduced-DOF states that we'll lift to the full body and compare.
    const std::array<std::array<double, 4>, 3> samples = {{
        {-2.356, 0.0, 1.571, 0.785},
        {-0.8, 0.0, 2.5, 0.785},
        {-1.5, 0.5, 2.0, -0.3},
    }};

    for (const auto &s : samples)
    {
        // Subgroup query.
        ob::ScopedState<> sub_state(subspace);
        for (std::size_t i = 0; i < active.size(); ++i)
            sub_state[i] = s[i];
        const bool sub_valid = sub_si->isValid(sub_state.get());

        // Equivalent full-body query, built from frozen + active overrides.
        ob::ScopedState<> full_state(full_space);
        for (std::size_t i = 0; i < Robot::dimension; ++i)
            full_state[i] = kStartFull[i];
        for (std::size_t i = 0; i < active.size(); ++i)
            full_state[active[i]] = s[i];
        const bool full_valid = full_si->isValid(full_state.get());

        BOOST_CHECK_EQUAL(sub_valid, full_valid);
    }
}

BOOST_AUTO_TEST_CASE(CheckMotion_AgreesWithFullBodyOnEquivalentEdge)
{
    auto env = makeEnvironment();
    auto [full_space, full_si] = makeFullBodySI(env);

    std::vector<std::size_t> active{3, 4, 5, 6};
    auto [subspace, sub_si] = makeSubgroupSI(env, active, toFrozen(kStartFull));

    const std::array<double, 4> sub_start{-2.356, 0.0, 1.571, 0.785};
    const std::array<double, 4> sub_goal{-0.8, 0.0, 2.5, 0.785};

    // Subgroup motion.
    ob::ScopedState<> s1(subspace), s2(subspace);
    for (std::size_t i = 0; i < active.size(); ++i)
    {
        s1[i] = sub_start[i];
        s2[i] = sub_goal[i];
    }
    const bool sub_ok = sub_si->getMotionValidator()->checkMotion(s1.get(), s2.get());

    // Equivalent full-body motion (frozen joints identical, active joints differ).
    ob::ScopedState<> f1(full_space), f2(full_space);
    for (std::size_t i = 0; i < Robot::dimension; ++i)
        f1[i] = f2[i] = kStartFull[i];
    for (std::size_t i = 0; i < active.size(); ++i)
    {
        f1[active[i]] = sub_start[i];
        f2[active[i]] = sub_goal[i];
    }
    const bool full_ok = full_si->getMotionValidator()->checkMotion(f1.get(), f2.get());

    BOOST_CHECK_EQUAL(sub_ok, full_ok);
}

// ---------------------------------------------------------------------------
// End-to-end planning + downstream features
// ---------------------------------------------------------------------------

BOOST_AUTO_TEST_CASE(Plan_RRTConnect_FindsSolutionOverSubgroup)
{
    auto env = makeEnvironment();
    std::vector<std::size_t> active{3, 4, 5, 6};
    auto subspace = std::make_shared<ob::SubspaceStateSpace>(makePandaBounds(), active, toFrozen(kStartFull));

    og::SimpleSetup ss(subspace);
    auto si = ss.getSpaceInformation();
    si->setStateValidityChecker(std::make_shared<ompl::vamp::VampSubgroupStateValidityChecker<Robot>>(si, env));
    si->setMotionValidator(std::make_shared<ompl::vamp::VampSubgroupMotionValidator<Robot>>(si, env));

    ob::ScopedState<> start(subspace), goal(subspace);
    for (std::size_t i = 0; i < active.size(); ++i)
    {
        start[i] = kStartFull[active[i]];
        goal[i] = kGoalFull[active[i]];
    }
    ss.setStartAndGoalStates(start, goal);
    ss.setPlanner(std::make_shared<og::RRTConnect>(si));

    BOOST_REQUIRE_EQUAL(ss.solve(5.0), ob::PlannerStatus::EXACT_SOLUTION);
    BOOST_CHECK_GE(ss.getSolutionPath().getStateCount(), 2u);
}

BOOST_AUTO_TEST_CASE(SimplifyAndInterpolate_OperateOnSubgroupPath)
{
    auto env = makeEnvironment();
    std::vector<std::size_t> active{3, 4, 5, 6};
    auto subspace = std::make_shared<ob::SubspaceStateSpace>(makePandaBounds(), active, toFrozen(kStartFull));

    og::SimpleSetup ss(subspace);
    auto si = ss.getSpaceInformation();
    si->setStateValidityChecker(std::make_shared<ompl::vamp::VampSubgroupStateValidityChecker<Robot>>(si, env));
    si->setMotionValidator(std::make_shared<ompl::vamp::VampSubgroupMotionValidator<Robot>>(si, env));

    ob::ScopedState<> start(subspace), goal(subspace);
    for (std::size_t i = 0; i < active.size(); ++i)
    {
        start[i] = kStartFull[active[i]];
        goal[i] = kGoalFull[active[i]];
    }
    ss.setStartAndGoalStates(start, goal);
    ss.setPlanner(std::make_shared<og::RRTConnect>(si));

    BOOST_REQUIRE_EQUAL(ss.solve(5.0), ob::PlannerStatus::EXACT_SOLUTION);

    auto &path = ss.getSolutionPath();
    const std::size_t before_simplify = path.getStateCount();
    ss.simplifySolution();
    BOOST_CHECK_GE(path.getStateCount(), 2u);
    BOOST_CHECK_LE(path.getStateCount(), before_simplify);

    const std::size_t after_simplify = path.getStateCount();
    path.interpolate(after_simplify + 10);
    BOOST_CHECK_GT(path.getStateCount(), after_simplify);

    // Endpoint preservation: lift the path endpoints back to ambient and confirm
    // the active-joint values still match the requested start/goal.
    auto first = subspace->expandToFull(path.getStates().front());
    auto last = subspace->expandToFull(path.getStates().back());
    for (std::size_t i = 0; i < active.size(); ++i)
    {
        BOOST_CHECK_CLOSE(first[active[i]], kStartFull[active[i]], 1e-6);
        BOOST_CHECK_CLOSE(last[active[i]], kGoalFull[active[i]], 1e-6);
    }
}

BOOST_AUTO_TEST_CASE(OptimizationObjective_PathLengthIntegratesOverSubgroup)
{
    auto env = makeEnvironment();
    std::vector<std::size_t> active{3, 4, 5, 6};
    auto subspace = std::make_shared<ob::SubspaceStateSpace>(makePandaBounds(), active, toFrozen(kStartFull));

    og::SimpleSetup ss(subspace);
    auto si = ss.getSpaceInformation();
    si->setStateValidityChecker(std::make_shared<ompl::vamp::VampSubgroupStateValidityChecker<Robot>>(si, env));
    si->setMotionValidator(std::make_shared<ompl::vamp::VampSubgroupMotionValidator<Robot>>(si, env));

    ob::ScopedState<> start(subspace), goal(subspace);
    for (std::size_t i = 0; i < active.size(); ++i)
    {
        start[i] = kStartFull[active[i]];
        goal[i] = kGoalFull[active[i]];
    }
    ss.setStartAndGoalStates(start, goal);

    auto obj = std::make_shared<ob::PathLengthOptimizationObjective>(si);
    ss.setOptimizationObjective(obj);
    ss.setPlanner(std::make_shared<og::RRTstar>(si));

    BOOST_REQUIRE_EQUAL(ss.solve(5.0), ob::PlannerStatus::EXACT_SOLUTION);

    auto cost = ss.getSolutionPath().cost(obj);
    BOOST_CHECK(std::isfinite(cost.value()));
    BOOST_CHECK_GT(cost.value(), 0.0);
}

BOOST_AUTO_TEST_CASE(Plan_FrozenIndicesAreExactlyFixedAlongSolutionPath)
{
    auto env = makeEnvironment();
    std::vector<std::size_t> active{3, 4, 5, 6};
    auto frozen = toFrozen(kStartFull);
    auto subspace = std::make_shared<ob::SubspaceStateSpace>(makePandaBounds(), active, frozen);

    og::SimpleSetup ss(subspace);
    auto si = ss.getSpaceInformation();
    si->setStateValidityChecker(std::make_shared<ompl::vamp::VampSubgroupStateValidityChecker<Robot>>(si, env));
    si->setMotionValidator(std::make_shared<ompl::vamp::VampSubgroupMotionValidator<Robot>>(si, env));

    ob::ScopedState<> start(subspace), goal(subspace);
    for (std::size_t i = 0; i < active.size(); ++i)
    {
        start[i] = kStartFull[active[i]];
        goal[i] = kGoalFull[active[i]];
    }
    ss.setStartAndGoalStates(start, goal);
    ss.setPlanner(std::make_shared<og::RRTConnect>(si));

    BOOST_REQUIRE_EQUAL(ss.solve(5.0), ob::PlannerStatus::EXACT_SOLUTION);
    auto &path = ss.getSolutionPath();
    path.interpolate();
    BOOST_REQUIRE_GE(path.getStateCount(), 2u);

    // For every waypoint, lift to ambient and confirm every non-active joint
    // matches the frozen pose bit-for-bit (we never write to those slots).
    for (std::size_t i = 0; i < path.getStateCount(); ++i)
    {
        auto full = subspace->expandToFull(path.getState(i));
        for (std::size_t k = 0; k < full.size(); ++k)
        {
            const bool is_active = std::find(active.begin(), active.end(), k) != active.end();
            if (!is_active)
                BOOST_CHECK_EQUAL(full[k], frozen[k]);
        }
        // Active joints must remain inside their subspace bounds (sanity check).
        for (std::size_t j = 0; j < active.size(); ++j)
        {
            BOOST_CHECK_GE(full[active[j]], subspace->getBounds().low[j]);
            BOOST_CHECK_LE(full[active[j]], subspace->getBounds().high[j]);
        }
    }
}

BOOST_AUTO_TEST_CASE(SwitchSubgroup_ReplanWithDifferentActiveSet)
{
    auto env = makeEnvironment();

    auto plan = [&](const std::vector<std::size_t> &active)
    {
        auto subspace = std::make_shared<ob::SubspaceStateSpace>(makePandaBounds(), active, toFrozen(kStartFull));

        og::SimpleSetup ss(subspace);
        auto si = ss.getSpaceInformation();
        si->setStateValidityChecker(std::make_shared<ompl::vamp::VampSubgroupStateValidityChecker<Robot>>(si, env));
        si->setMotionValidator(std::make_shared<ompl::vamp::VampSubgroupMotionValidator<Robot>>(si, env));

        ob::ScopedState<> start(subspace), goal(subspace);
        for (std::size_t i = 0; i < active.size(); ++i)
        {
            start[i] = kStartFull[active[i]];
            goal[i] = kGoalFull[active[i]];
        }
        ss.setStartAndGoalStates(start, goal);
        ss.setPlanner(std::make_shared<og::RRTConnect>(si));
        return ss.solve(5.0);
    };

    BOOST_CHECK_EQUAL(plan({3, 4, 5, 6}), ob::PlannerStatus::EXACT_SOLUTION);  // wrist subset
    BOOST_CHECK_EQUAL(plan({0, 1, 2}), ob::PlannerStatus::EXACT_SOLUTION);     // base subset
}

BOOST_AUTO_TEST_CASE(SetFrozenValues_LiveUpdatePicksUpInValidator)
{
    auto env = makeEnvironment();
    auto [full_space, full_si] = makeFullBodySI(env);

    std::vector<std::size_t> active{3, 4, 5, 6};
    auto [subspace, sub_si] = makeSubgroupSI(env, active, toFrozen(kStartFull));

    const std::array<double, 4> sample{-2.356, 0.0, 1.571, 0.785};

    ob::ScopedState<> sub_state(subspace);
    for (std::size_t i = 0; i < active.size(); ++i)
        sub_state[i] = sample[i];

    auto fullCheck = [&](const std::array<double, 7> &frozen)
    {
        ob::ScopedState<> full_state(full_space);
        for (std::size_t i = 0; i < Robot::dimension; ++i)
            full_state[i] = frozen[i];
        for (std::size_t i = 0; i < active.size(); ++i)
            full_state[active[i]] = sample[i];
        return full_si->isValid(full_state.get());
    };

    // Initially the validator uses kStartFull as the frozen pose.
    BOOST_CHECK_EQUAL(sub_si->isValid(sub_state.get()), fullCheck(kStartFull));

    // Swap in a different frozen pose; the validator must now agree with the
    // full-body check evaluated at the new frozen pose.
    subspace->setFrozenValues(toFrozen(kGoalFull));
    BOOST_CHECK_EQUAL(sub_si->isValid(sub_state.get()), fullCheck(kGoalFull));
}

// ---------------------------------------------------------------------------
// Compose with ConstrainedStateSpace — the design driver for inheriting
// from the state space rather than swapping the StateSampler.
// ---------------------------------------------------------------------------

namespace
{
    // Constrain the *second* active coordinate (joint 4 in ambient) to zero.
    // Linear, so kStartFull / kGoalFull (both joint-4 = 0) lie on the manifold
    // and linear interpolation stays on it.
    class SecondCoordZero : public ob::Constraint
    {
    public:
        explicit SecondCoordZero(unsigned int dim) : ob::Constraint(dim, 1)
        {
        }
        void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
        {
            out[0] = x[1];
        }
        void jacobian(const Eigen::Ref<const Eigen::VectorXd> & /*x*/, Eigen::Ref<Eigen::MatrixXd> out) const override
        {
            out.setZero();
            out(0, 1) = 1.0;
        }
    };

    // Nonlinear constraint: ‖x − center‖ = radius (a 3-sphere in 4D),
    // forcing the planner to walk along a curved manifold. Endpoints are
    // crafted to sit on this sphere.
    class WristSphere : public ob::Constraint
    {
    public:
        WristSphere(unsigned int dim, Eigen::VectorXd center, double radius)
          : ob::Constraint(dim, 1), center_(std::move(center)), radius_(radius)
        {
        }
        void function(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::VectorXd> out) const override
        {
            out[0] = (x - center_).norm() - radius_;
        }
        void jacobian(const Eigen::Ref<const Eigen::VectorXd> &x, Eigen::Ref<Eigen::MatrixXd> out) const override
        {
            const Eigen::VectorXd d = x - center_;
            const double n = d.norm();
            if (n > 0.0)
                out = (d / n).transpose();
            else
                out.setZero();
        }

    private:
        Eigen::VectorXd center_;
        double radius_;
    };

    struct ManifoldResult
    {
        ob::PlannerStatus status;
        double worst_residual;      // max ‖constraint(x)‖ over the path
        double worst_frozen_drift;  // max |ambient[i] - frozen[i]| over frozen indices i
    };

    // Plan over the wrist subgroup under a manifold constraint with the
    // requested constrained state-space flavor (Projected / Atlas / Tangent
    // Bundle). Returns the planning status, the worst constraint residual,
    // and the worst drift in any frozen ambient joint across the interpolated
    // solution path. The constraint residual confirms the trajectory follows
    // the manifold; the frozen drift confirms the subspace machinery never
    // perturbs the joints we promised not to move.
    template <typename CSS>
    auto planManifold(const Environment &env, const std::shared_ptr<ob::Constraint> &constraint,
                      const std::array<double, 4> &start_active, const std::array<double, 4> &goal_active,
                      double time_limit = 10.0) -> ManifoldResult
    {
        std::vector<std::size_t> active{3, 4, 5, 6};
        auto frozen = toFrozen(kStartFull);
        auto subspace = std::make_shared<ob::SubspaceStateSpace>(makePandaBounds(), active, frozen);
        auto css = std::make_shared<CSS>(subspace, constraint);
        auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(css);
        css->setup();
        // Only the state validity checker — the constrained SI provides
        // ConstrainedMotionValidator which discretizes along the manifold and
        // delegates per-step validity to our checker.
        csi->setStateValidityChecker(std::make_shared<ompl::vamp::VampSubgroupStateValidityChecker<Robot>>(csi, env));
        // Install our motion validator instead of the default
        // ConstrainedMotionValidator. Our validator detects the constrained
        // wrapper, walks the manifold via the constrained state space's own
        // ``discreteGeodesic``, and batch-validates the resulting samples
        // through VAMP's ``Robot::fkcc<rake>``, restoring the rake-wide SIMD
        // throughput that the default per-state path forfeits.
        csi->setMotionValidator(std::make_shared<ompl::vamp::VampSubgroupMotionValidator<Robot>>(csi, env));
        csi->setup();

        og::SimpleSetup ss(csi);
        ob::ScopedState<> start(css), goal(css);
        for (std::size_t i = 0; i < active.size(); ++i)
        {
            start[i] = start_active[i];
            goal[i] = goal_active[i];
        }
        // Atlas-based spaces need at least one chart anchored before sampling;
        // anchor at both endpoints to seed the manifold approximation.
        if (auto *atlas = dynamic_cast<ob::AtlasStateSpace *>(css.get()))
        {
            atlas->anchorChart(start.get());
            atlas->anchorChart(goal.get());
        }
        ss.setStartAndGoalStates(start, goal);
        ss.setPlanner(std::make_shared<og::RRTConnect>(csi));

        ManifoldResult result{ss.solve(time_limit), 0.0, 0.0};
        if (result.status == ob::PlannerStatus::EXACT_SOLUTION)
        {
            auto path = ss.getSolutionPath();
            path.interpolate();
            Eigen::VectorXd x(css->getDimension());
            Eigen::VectorXd r(constraint->getCoDimension());
            for (std::size_t i = 0; i < path.getStateCount(); ++i)
            {
                const auto *rv = path.getState(i)->as<ob::ConstrainedStateSpace::StateType>();
                for (unsigned int j = 0; j < css->getDimension(); ++j)
                    x[j] = (*rv)[j];
                constraint->function(x, r);
                result.worst_residual = std::max(result.worst_residual, r.norm());

                // Lift to ambient and confirm the frozen joints never moved.
                // ``subspace->expandToFull`` reads the active values from the
                // underlying real-vector state held by the wrapper.
                auto full = subspace->expandToFull(path.getState(i));
                for (std::size_t k = 0; k < full.size(); ++k)
                {
                    const bool is_active = std::find(active.begin(), active.end(), k) != active.end();
                    if (!is_active)
                        result.worst_frozen_drift = std::max(result.worst_frozen_drift, std::abs(full[k] - frozen[k]));
                }
            }
        }
        return result;
    }
}  // namespace

BOOST_AUTO_TEST_CASE(ManifoldPlanning_LinearConstraint_ProjectedStateSpace)
{
    auto env = makeEnvironment();
    auto constraint = std::make_shared<SecondCoordZero>(4);
    std::array<double, 4> start_a{kStartFull[3], kStartFull[4], kStartFull[5], kStartFull[6]};
    std::array<double, 4> goal_a{kGoalFull[3], kGoalFull[4], kGoalFull[5], kGoalFull[6]};
    auto r = planManifold<ob::ProjectedStateSpace>(env, constraint, start_a, goal_a);
    BOOST_REQUIRE_EQUAL(r.status, ob::PlannerStatus::EXACT_SOLUTION);
    BOOST_CHECK_LE(r.worst_residual, constraint->getTolerance());
    BOOST_CHECK_SMALL(r.worst_frozen_drift, 1e-9);
}

BOOST_AUTO_TEST_CASE(ManifoldPlanning_LinearConstraint_AtlasStateSpace)
{
    auto env = makeEnvironment();
    auto constraint = std::make_shared<SecondCoordZero>(4);
    std::array<double, 4> start_a{kStartFull[3], kStartFull[4], kStartFull[5], kStartFull[6]};
    std::array<double, 4> goal_a{kGoalFull[3], kGoalFull[4], kGoalFull[5], kGoalFull[6]};
    auto r = planManifold<ob::AtlasStateSpace>(env, constraint, start_a, goal_a);
    BOOST_REQUIRE_EQUAL(r.status, ob::PlannerStatus::EXACT_SOLUTION);
    BOOST_CHECK_LE(r.worst_residual, constraint->getTolerance());
    BOOST_CHECK_SMALL(r.worst_frozen_drift, 1e-9);
}

BOOST_AUTO_TEST_CASE(ManifoldPlanning_LinearConstraint_TangentBundleStateSpace)
{
    auto env = makeEnvironment();
    auto constraint = std::make_shared<SecondCoordZero>(4);
    std::array<double, 4> start_a{kStartFull[3], kStartFull[4], kStartFull[5], kStartFull[6]};
    std::array<double, 4> goal_a{kGoalFull[3], kGoalFull[4], kGoalFull[5], kGoalFull[6]};
    auto r = planManifold<ob::TangentBundleStateSpace>(env, constraint, start_a, goal_a);
    BOOST_REQUIRE_EQUAL(r.status, ob::PlannerStatus::EXACT_SOLUTION);
    // TangentBundle is lazy — its produced waypoints can sit slightly off the
    // manifold by design, so allow a small multiple of the constraint tolerance.
    BOOST_CHECK_LE(r.worst_residual, 10.0 * constraint->getTolerance());
    BOOST_CHECK_SMALL(r.worst_frozen_drift, 1e-9);
}

BOOST_AUTO_TEST_CASE(ManifoldPlanning_NonlinearConstraint_ProjectedStateSpace)
{
    auto env = makeEnvironment();
    // 3-sphere of radius 0.5 in the 4D active subspace centred at the start
    // wrist pose. Place start and goal on the sphere along two orthogonal
    // coordinate axes so they're geometrically far apart but feasible.
    Eigen::Vector4d center(kStartFull[3], kStartFull[4], kStartFull[5], kStartFull[6]);
    const double radius = 0.5;
    const Eigen::Vector4d start_vec = center + Eigen::Vector4d(radius, 0.0, 0.0, 0.0);
    const Eigen::Vector4d goal_vec = center + Eigen::Vector4d(0.0, 0.0, radius, 0.0);

    auto constraint = std::make_shared<WristSphere>(4, center, radius);
    std::array<double, 4> start_a{start_vec[0], start_vec[1], start_vec[2], start_vec[3]};
    std::array<double, 4> goal_a{goal_vec[0], goal_vec[1], goal_vec[2], goal_vec[3]};

    // Sanity-check: both endpoints lie on the sphere.
    Eigen::VectorXd rr(1);
    Eigen::VectorXd vs = start_vec, vg = goal_vec;
    constraint->function(vs, rr);
    BOOST_REQUIRE_LE(std::abs(rr[0]), constraint->getTolerance());
    constraint->function(vg, rr);
    BOOST_REQUIRE_LE(std::abs(rr[0]), constraint->getTolerance());

    auto r = planManifold<ob::ProjectedStateSpace>(env, constraint, start_a, goal_a, 15.0);
    BOOST_REQUIRE_EQUAL(r.status, ob::PlannerStatus::EXACT_SOLUTION);
    BOOST_CHECK_LE(r.worst_residual, constraint->getTolerance());
    BOOST_CHECK_SMALL(r.worst_frozen_drift, 1e-9);
}

BOOST_AUTO_TEST_SUITE_END()
