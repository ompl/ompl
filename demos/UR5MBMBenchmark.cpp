// demos/UR5MBMBenchmark.cpp
//
// The CBF rollout against ordinary collision-checked planning on a *standard* problem
// set: MotionBenchMaker's UR5 scenes, 689 problems over 7 scenes, as shipped by VAMP.
//
//     ./scripts/mbm_to_scenes.py /path/to/vamp/resources/ur5/problems.json scenes.txt
//     ./build/demos/demo_UR5MBMBenchmark scenes.txt [perScene] [seconds] [voxel] [stepSize]
//         [range] [margin] [buffer] [segmentFraction] [gamma] [maxStepScale] [selfMargin]
//         [pathPrefix]
//
// `pathPrefix` dumps the audited motions of both rows (`<prefix>.rrtc`, `<prefix>.cbf`) so
// they can be replayed against the real UR5 meshes in PyBullet:
//
//     ur5_experiments/scripts/audit_self_collision.py --path <prefix>.cbf
//
// That is the only check that does not share the sphere model with the `collide` column.
//
// The last two are the certified-step A/B: `maxStepScale 1` pins the rollout to a fixed
// `stepSize` however much room it has, and `gamma` scales how much of its clearance a
// step may spend, so it scales the certificate with it.
//
// Why this and not UR5CBFPlanningDemo's scene: that one is a pair of spheres placed to
// block the direct sweep, and `geometric::RRTConnect` solves it in six vertices. A
// method whose claim is "no wasted edges" cannot show anything on a problem with no
// wasted edges. These scenes are cluttered, externally defined, and widely reported, so
// they can neither be tuned to flatter the filter nor dismissed as a strawman.
//
// Every obstacle in the set is a box or a cylinder, both of which have an exact
// closed-form signed distance -- so the field the barrier reads is exact up to the grid,
// with no mesh, no FCL, and no sign ambiguity.
//
// ### What is compared
//
// - `rrtconnect`: stock geometric RRTConnect, straight-line edges, the SDF behind an
//   ordinary StateValidityChecker. The bar.
// - `cbf-rrtc`: the same planner over `cbf::FilteredStateSpace`, so every edge is a CBF
//   rollout and every intermediate state is certified as it is produced. No state
//   validity checker at all.
//
// Both see the same field, the same margin, the same start and goal.
//
// ### Feasibility is reported, not assumed
//
// MotionBenchMaker calls a problem valid when the *mesh* robot is collision free. Our
// robot is 40 spheres that do not enclose those meshes (see ClearanceBarrier) and the
// barrier adds a margin on top, so a problem can be valid upstream and still have its
// start or goal inside our margin. Those are counted and excluded rather than scored as
// failures -- and the count is itself a result, because a margin that rules out most of
// a standard benchmark is a finding about the margin.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/cbf/CBFControlFilter.h>
#include <ompl/cbf/ExecutedPath.h>
#include <ompl/cbf/FilteredMotionValidator.h>
#include <ompl/cbf/FilteredStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/util/Time.h>

#include "UR5SelfCollisionAudit.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;
using Barrier = ompl::cbf::ClearanceBarrier;
using Filter = ompl::cbf::CBFControlFilter;
using Space = ompl::cbf::FilteredStateSpace;
using UR5 = ompl::robots::UR5;
namespace sdf = ompl::sdf;

namespace
{
    constexpr int dimension = 6;

    /// Joint-space spacing both rows are audited at, in radians. Finer than the rollout
    /// step so the audit is not merely re-reading the filter's own decisions.
    constexpr double auditResolution = 0.02;

    /// One obstacle: a box (`halfExtents`) or a cylinder (`radius`, `halfLength` about
    /// the local z axis), posed in the world.
    struct Obstacle
    {
        enum class Kind
        {
            Box,
            Cylinder
        };

        Kind kind{Kind::Box};
        Eigen::Vector3d halfExtents{Eigen::Vector3d::Zero()};
        double radius{0.0};
        double halfLength{0.0};
        Eigen::Vector3d position{Eigen::Vector3d::Zero()};
        Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};

        /// Exact signed distance. Both formulas are the standard ones: reduce to the
        /// box's local frame, take the per-axis overshoot, then the norm of its positive
        /// part outside and the largest (negative) component inside.
        double distance(const Eigen::Vector3d &p) const
        {
            const Eigen::Vector3d q = rotation.transpose() * (p - position);
            if (kind == Kind::Box)
            {
                const Eigen::Vector3d d = q.cwiseAbs() - halfExtents;
                return d.cwiseMax(0.0).norm() + std::min(d.maxCoeff(), 0.0);
            }
            const Eigen::Vector2d d(q.head<2>().norm() - radius, std::abs(q.z()) - halfLength);
            return d.cwiseMax(0.0).norm() + std::min(d.maxCoeff(), 0.0);
        }
    };

    struct Problem
    {
        std::string scene;
        int index{0};
        UR5::Configuration start{UR5::Configuration::Zero()};
        UR5::Configuration goal{UR5::Configuration::Zero()};
        std::vector<Obstacle> obstacles;

        /// A union of solids is the min of their distances, which is still exact.
        sdf::DistanceFn field() const
        {
            const std::vector<Obstacle> solids = obstacles;
            return [solids](const Eigen::Vector3d &p)
            {
                double distance = std::numeric_limits<double>::infinity();
                for (const Obstacle &solid : solids)
                    distance = std::min(distance, solid.distance(p));
                return distance;
            };
        }
    };

    Eigen::Matrix3d rotationOf(double x, double y, double z, double w)
    {
        return Eigen::Quaterniond(w, x, y, z).normalized().toRotationMatrix();
    }

    std::vector<Problem> readProblems(const std::string &path)
    {
        std::ifstream in(path);
        if (!in)
            throw ompl::Exception("cannot open " + path +
                                  " -- generate it with scripts/mbm_to_scenes.py");

        std::vector<Problem> problems;
        std::string line;
        while (std::getline(in, line))
        {
            if (line.empty() || line[0] == '#')
                continue;
            std::istringstream fields(line);
            std::string tag;
            fields >> tag;

            if (tag == "problem")
            {
                problems.emplace_back();
                fields >> problems.back().scene >> problems.back().index;
            }
            else if (problems.empty())
            {
                continue;
            }
            else if (tag == "start" || tag == "goal")
            {
                UR5::Configuration q;
                for (int j = 0; j < dimension; ++j)
                    fields >> q[j];
                (tag == "start" ? problems.back().start : problems.back().goal) = q;
            }
            else if (tag == "box" || tag == "cyl")
            {
                Obstacle obstacle;
                double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
                if (tag == "box")
                {
                    obstacle.kind = Obstacle::Kind::Box;
                    fields >> obstacle.halfExtents[0] >> obstacle.halfExtents[1] >>
                        obstacle.halfExtents[2];
                }
                else
                {
                    obstacle.kind = Obstacle::Kind::Cylinder;
                    fields >> obstacle.radius >> obstacle.halfLength;
                }
                fields >> obstacle.position[0] >> obstacle.position[1] >> obstacle.position[2] >>
                    qx >> qy >> qz >> qw;
                obstacle.rotation = rotationOf(qx, qy, qz, qw);
                problems.back().obstacles.push_back(obstacle);
            }
        }
        return problems;
    }

    ob::RealVectorBounds jointBounds()
    {
        ob::RealVectorBounds bounds(dimension);
        for (int j = 0; j < dimension; ++j)
        {
            bounds.setLow(j, UR5::lowerBounds()[j]);
            bounds.setHigh(j, UR5::upperBounds()[j]);
        }
        return bounds;
    }

    struct Result
    {
        bool solved{false};
        double seconds{0.0};
        std::size_t evaluations{0};  ///< collision checks, or filter calls
        std::size_t vertices{0};
        std::size_t unsafeStates{0};
        std::size_t auditedStates{0};
        std::size_t misses{0};  ///< solution edges re-derived rather than replayed
        /// Joint-space radians per filter call, and the share of calls that ran past
        /// stepSize on a certificate. Meaningless for the baseline, whose "evaluation"
        /// is a collision check at a fixed resolution rather than a step.
        double radPerCall{0.0};
        double coarse{0.0};
        double minClearance{std::numeric_limits<double>::infinity()};
        /// The independent self-collision check -- see demos/UR5SelfCollisionAudit.h. The
        /// barrier carries self rows now, but only for the pairs the offline search kept,
        /// so this walks all of them and is the only thing able to say that search was
        /// wrong.
        double minSelfOverlap{std::numeric_limits<double>::infinity()};
        std::size_t selfColliding{0};
    };

    /// Audit a solution the way the CBF rows must be audited: expand it into the motion
    /// that would actually be executed, then evaluate the *unbuffered* barrier at every
    /// state of it.
    ///
    /// `record`, when given, collects exactly the states audited here, so an external mesh
    /// checker is handed the same motion at the same resolution that the `collide` column
    /// is computed from. Anything coarser would let a mesh contact hide between the states
    /// the sphere model was scored on.
    void audit(const og::PathGeometric &solution, const Barrier &barrier, Result &result,
               std::vector<UR5::Configuration> *record)
    {
        // executedPath() replays the rollout the planner recorded for each edge, so this
        // is the executed motion rather than a reconstruction of it -- and it honours the
        // requested resolution. PathGeometric::interpolate() would not: it budgets states
        // by the Euclidean distance() between waypoints, which is a lower bound on the arc
        // length of a deflected edge, so it under-samples exactly where the CBF was
        // working hardest. `misses` must be zero or the replay claim is void.
        //
        // The resolution is the one the baseline is audited at, so the clearance columns
        // mean the same thing in both rows. Note this genuinely samples *inside* a step
        // now; the old rollout quantised every fraction to a step boundary, so a fine
        // request silently came back at the step size.
        const og::PathGeometric path = ompl::cbf::executedPath(solution, auditResolution, &result.misses);
        result.auditedStates = path.getStateCount();
        for (std::size_t i = 0; i < path.getStateCount(); ++i)
        {
            const UR5::Configuration q = Space::configurationOf(path.getState(i));
            const double h = barrier.worstValue(q);
            result.minClearance = std::min(result.minClearance, h);
            if (h < 0.0)
                ++result.unsafeStates;

            const double own = ompl::demo::worstSelfOverlap(barrier.robot(), q);
            result.minSelfOverlap = std::min(result.minSelfOverlap, own);
            if (own < 0.0)
                ++result.selfColliding;

            if (record != nullptr)
                record->push_back(q);
        }
    }

    /// The bar: straight-line edges, the same SDF behind an ordinary validity checker.
    Result runCollisionChecked(const Problem &problem, const Barrier &barrier, double range,
                               double timeLimit, double segmentFraction,
                               std::vector<UR5::Configuration> *record)
    {
        auto space = std::make_shared<ob::RealVectorStateSpace>(dimension);
        space->setBounds(jointBounds());
        if (segmentFraction > 0.0)
            space->setLongestValidSegmentFraction(segmentFraction);
        auto si = std::make_shared<ob::SpaceInformation>(space);

        std::size_t checks = 0;
        si->setStateValidityChecker([&barrier, &checks](const ob::State *state)
                                    {
                                        ++checks;
                                        return barrier.isSafe(Space::configurationOf(state));
                                    });
        si->setup();

        ob::ScopedState<> start(space), goal(space);
        for (int j = 0; j < dimension; ++j)
        {
            start[j] = problem.start[j];
            goal[j] = problem.goal[j];
        }
        auto pdef = std::make_shared<ob::ProblemDefinition>(si);
        pdef->setStartAndGoalStates(start, goal, 0.05);

        auto planner = std::make_shared<og::RRTConnect>(si);
        planner->setRange(range);
        planner->setProblemDefinition(pdef);
        planner->setup();

        Result result;
        const ompl::time::point begin = ompl::time::now();
        const ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(timeLimit));
        result.seconds = ompl::time::seconds(ompl::time::now() - begin);
        result.solved = (status == ob::PlannerStatus::EXACT_SOLUTION);
        result.evaluations = checks;

        ob::PlannerData data(si);
        planner->getPlannerData(data);
        result.vertices = data.numVertices();

        if (result.solved)
        {
            // Audited at the same resolution the rollout uses, so the clearance columns
            // mean the same thing in both rows.
            og::PathGeometric path(*std::static_pointer_cast<og::PathGeometric>(
                pdef->getSolutionPath()));
            path.interpolate(static_cast<unsigned int>(path.length() / auditResolution));
            result.auditedStates = path.getStateCount();
            for (std::size_t i = 0; i < path.getStateCount(); ++i)
            {
                const UR5::Configuration q = Space::configurationOf(path.getState(i));
                const double h = barrier.worstValue(q);
                result.minClearance = std::min(result.minClearance, h);
                if (h < 0.0)
                    ++result.unsafeStates;

                // The baseline is audited for self-collision on the same terms as the
                // filtered row, and it has to be: its validity checker is this same
                // barrier's isSafe(), so it is subject to the self rows too, and a
                // comparison that audited only one of them would be measuring the audit.
                const double own = ompl::demo::worstSelfOverlap(barrier.robot(), q);
                result.minSelfOverlap = std::min(result.minSelfOverlap, own);
                if (own < 0.0)
                    ++result.selfColliding;

                if (record != nullptr)
                    record->push_back(q);
            }
        }
        return result;
    }

    /// The CBF rollout as the state space's interpolate(), with no collision checking
    /// anywhere: the barrier certifies each step as it is produced.
    Result runFiltered(const Problem &problem, const Barrier &audited, const Filter &filter,
                       double stepSize, double range, double timeLimit, double maxStepScale,
                       std::vector<UR5::Configuration> *record)
    {
        auto space = std::make_shared<Space>(filter, stepSize, UR5::velocityLimits());
        space->setBounds(jointBounds());
        if (maxStepScale > 0.0)
            space->setMaxStepScale(maxStepScale);

        auto si = std::make_shared<ob::SpaceInformation>(space);
        si->setStateValidityChecker(std::make_shared<ob::AllValidStateValidityChecker>(si));
        si->setMotionValidator(std::make_shared<ompl::cbf::FilteredMotionValidator>(si));
        si->setup();

        ob::ScopedState<> start(space), goal(space);
        for (int j = 0; j < dimension; ++j)
        {
            start[j] = problem.start[j];
            goal[j] = problem.goal[j];
        }
        auto pdef = std::make_shared<ob::ProblemDefinition>(si);
        // A rollout cannot be asked to land on an exact state next to an obstacle, so the
        // goal gets a tolerance. Kept small enough that "solved" still means solved.
        pdef->setStartAndGoalStates(start, goal, 0.1);

        auto planner = std::make_shared<og::RRTConnect>(si);
        planner->setRange(range);
        planner->setProblemDefinition(pdef);
        planner->setup();

        Result result;
        const ompl::time::point begin = ompl::time::now();
        const ob::PlannerStatus status = planner->solve(ob::timedPlannerTerminationCondition(timeLimit));
        result.seconds = ompl::time::seconds(ompl::time::now() - begin);
        result.solved = (status == ob::PlannerStatus::EXACT_SOLUTION);
        result.evaluations = space->statistics().steps;
        if (space->statistics().steps > 0)
        {
            const double calls = static_cast<double>(space->statistics().steps);
            result.radPerCall = space->statistics().travel / calls;
            result.coarse = static_cast<double>(space->statistics().coarse) / calls;
        }

        ob::PlannerData data(si);
        planner->getPlannerData(data);
        result.vertices = data.numVertices();

        if (result.solved)
            audit(*std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath()), audited,
                  result, record);
        return result;
    }

    struct Tally
    {
        int attempted{0};
        int skipped{0};  ///< start or goal inside our margin
        /// min(start, goal) sphere clearance at zero margin, per problem. VAMP validated
        /// this set against the same 40 spheres with no margin, so these should all be
        /// positive; how far above zero they sit is what decides which margins are
        /// affordable on a standard benchmark.
        std::vector<double> endpointClearance;
        int solved[2]{0, 0};
        std::vector<double> seconds[2];
        std::vector<double> evaluations[2];
        std::vector<double> vertices[2];
        std::vector<double> radPerCall[2];
        std::vector<double> coarse[2];
        std::size_t unsafe[2]{0, 0};
        std::size_t audited[2]{0, 0};
        std::size_t misses[2]{0, 0};
        std::size_t selfColliding[2]{0, 0};
        double worstClearance[2]{std::numeric_limits<double>::infinity(),
                                 std::numeric_limits<double>::infinity()};
        double worstSelf[2]{std::numeric_limits<double>::infinity(),
                            std::numeric_limits<double>::infinity()};

        void add(int row, const Result &result)
        {
            solved[row] += result.solved ? 1 : 0;
            seconds[row].push_back(result.seconds);
            evaluations[row].push_back(static_cast<double>(result.evaluations));
            vertices[row].push_back(static_cast<double>(result.vertices));
            radPerCall[row].push_back(result.radPerCall);
            coarse[row].push_back(result.coarse);
            unsafe[row] += result.unsafeStates;
            audited[row] += result.auditedStates;
            misses[row] += result.misses;
            selfColliding[row] += result.selfColliding;
            if (result.solved)
            {
                worstClearance[row] = std::min(worstClearance[row], result.minClearance);
                worstSelf[row] = std::min(worstSelf[row], result.minSelfOverlap);
            }
        }
    };

    double median(std::vector<double> values)
    {
        if (values.empty())
            return 0.0;
        std::sort(values.begin(), values.end());
        return values[values.size() / 2];
    }

    /// Append one problem's audited motion, preceded by the marker the Python auditor
    /// splits on. The marker matters: the file holds many unrelated motions, and
    /// interpolating across the seam between two of them invents states that belong to no
    /// trajectory the planner produced -- which on a self-collision count reads as the
    /// planner having folded the arm onto itself.
    void writeMotion(std::ofstream &out, const Problem &problem,
                     const std::vector<UR5::Configuration> &states)
    {
        if (!out.is_open() || states.empty())
            return;
        out << "# motion " << problem.scene << " " << problem.index << "\n";
        for (const UR5::Configuration &q : states)
        {
            for (int j = 0; j < dimension; ++j)
                out << (j ? " " : "") << q[j];
            out << "\n";
        }
    }

    void reportRow(const char *label, const Tally &tally, int row)
    {
        const int scored = tally.attempted - tally.skipped;
        std::printf("  %-11s %3d/%-4d %9.2f %10.0f %8.0f", label, tally.solved[row], scored,
                    1e3 * median(tally.seconds[row]), median(tally.evaluations[row]),
                    median(tally.vertices[row]));
        if (median(tally.radPerCall[row]) > 0.0)
            std::printf(" %8.4f %5.0f%%", median(tally.radPerCall[row]),
                        1e2 * median(tally.coarse[row]));
        else
            std::printf(" %8s %6s", "-", "-");
        if (tally.solved[row] > 0)
            std::printf(" %10.4f %6zu/%-7zu %6zu %10.4f %7zu\n", tally.worstClearance[row],
                        tally.unsafe[row], tally.audited[row], tally.misses[row],
                        tally.worstSelf[row], tally.selfColliding[row]);
        else
            std::printf(" %10s %14s %6s %10s %7s\n", "-", "-", "-", "-", "-");
    }
}  // namespace

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::printf("usage: %s scenes.txt [perScene] [seconds] [voxel] [stepSize] [range]\n"
                    "       [margin] [buffer] [segmentFraction] [gamma] [maxStepScale]\n"
                    "       [selfMargin] [pathPrefix]\n\n"
                    "Generate scenes.txt with scripts/mbm_to_scenes.py.\n",
                    argv[0]);
        return 1;
    }

    const std::string path = argv[1];
    const int perScene = argc > 2 ? std::atoi(argv[2]) : 10;
    const double timeLimit = argc > 3 ? std::atof(argv[3]) : 5.0;
    const double voxel = argc > 4 ? std::atof(argv[4]) : 0.03;
    const double stepSize = argc > 5 ? std::atof(argv[5]) : 0.05;
    const double range = argc > 6 ? std::atof(argv[6]) : 2.0;
    // The audited margin, and the extra the filter guards on top of it. MotionBenchMaker
    // endpoints are grasp poses sitting ~8 mm off the shelf, so the defaults
    // (0.06 + one voxel) rule out most of the set -- these exist to find out what does
    // fit, with the unsafe column as the check on having shrunk them too far.
    const double margin = argc > 7 ? std::atof(argv[7]) : Barrier::defaultMargin;
    const double buffer = argc > 8 ? std::atof(argv[8]) : -1.0;
    // Resolution the baseline checks its straight-line edges at, as a fraction of the
    // state space's maximum extent. OMPL defaults to 0.01, which on this space is 0.31
    // rad between samples -- far coarser than the audit, so the baseline is scored unsafe
    // at the default. Tightening this is what makes the comparison like for like: both
    // rows then have to be audit-clean, and the cost of being so is the thing to compare.
    const double segmentFractionArg = argc > 9 ? std::atof(argv[9]) : -1.0;
    // The CBF decay rate, which is also the fraction of its clearance a step is allowed
    // to spend -- so it scales the certified step directly. At 1.0 the barrier is a plain
    // collision condition and the certificate is as long as the clearance allows.
    const double gamma = argc > 10 ? std::atof(argv[10]) : 0.4;
    // Cap on the certified step, as a multiple of stepSize. 1.0 is the fixed-step A/B.
    const double maxStepScale = argc > 11 ? std::atof(argv[11]) : -1.0;
    // The self-collision margin. A large negative value is the A/B for the rows
    // themselves: every pair clearance becomes enormous, so none is ever screened in and
    // none can cap the certificate, which is what the barrier did before it modelled the
    // arm against itself. The independent audit column is unaffected either way, so it
    // says what turning them off costs.
    const double selfMargin = argc > 12 ? std::atof(argv[12]) : Barrier::defaultSelfMargin;
    // Where to dump the audited motions, one file per row (`<prefix>.cbf`, `<prefix>.rrtc`).
    // The `collide` column is a statement about the 40 spheres and nothing else; these
    // files are what lets ur5_experiments/scripts/audit_self_collision.py repeat the count
    // against the real meshes, which is the only thing that can say the sphere model is
    // too coarse to stand in for them.
    const std::string pathPrefix = argc > 13 ? argv[13] : std::string();

    // Tie the baseline's edge-checking spacing to the rollout's step unless told otherwise,
    // so neither row is scored at a resolution the other never saw. The rollout advances
    // `maxSpeed * stepSize` radians per filter call; OMPL states the baseline's resolution
    // as a fraction of the state space's maximum extent, so convert. A RealVectorStateSpace
    // over `dimension` joints spanning [lo, hi] has extent |hi - lo| * sqrt(dimension).
    const double rolloutStep = UR5::velocityLimits().maxCoeff() * stepSize;
    const double extent =
        (UR5::upperBounds() - UR5::lowerBounds()).norm();
    const double segmentFraction =
        segmentFractionArg > 0.0 ? segmentFractionArg : rolloutStep / extent;

    ompl::RNG::setSeed(1);
    ompl::msg::setLogLevel(ompl::msg::LOG_ERROR);

    const std::vector<Problem> problems = readProblems(path);
    const UR5 robot;

    Filter::Parameters parameters;
    parameters.gamma = gamma;
    parameters.maxSpeed = UR5::velocityLimits();
    parameters.respectJointLimits = true;

    std::printf("\nMotionBenchMaker UR5, %d problems loaded, up to %d per scene\n",
                static_cast<int>(problems.size()), perScene);
    std::printf("voxel %.3f m, margin %.4f m + %.4f m filter buffer, stepSize %.3f s, "
                "range %.2f rad, %.1f s limit\n\n",
                voxel, margin,
                buffer < 0.0 ? Barrier::interpolationBuffer(
                                   sdf::GridSDF(problems.front().field(),
                                                UR5::reachableBounds(), voxel))
                             : buffer,
                stepSize, range, timeLimit);
    std::printf("gamma %.2f, certified step %s, self-collision margin %.4f m over %d pairs%s\n",
                gamma, maxStepScale > 0.0 ? "capped" : "uncapped", selfMargin,
                static_cast<int>(UR5::nSelfPairs), selfMargin < 0.0 ? " (rows disabled)" : "");
    std::printf("baseline segment: %.6f of extent = %.4f rad, rollout step %.4f rad (%s)\n\n",
                segmentFraction, segmentFraction * extent, rolloutStep,
                segmentFractionArg > 0.0 ? "overridden" : "matched to the rollout");
    std::printf("  %-11s %8s %9s %10s %8s %8s %6s %10s %14s %6s %10s %7s\n", "planner", "solved",
                "ms", "evals", "vertices", "rad/call", "coarse", "worst clr", "unsafe/audited",
                "missed", "worst self", "collide");

    std::map<std::string, Tally> tallies;
    std::map<std::string, int> seen;
    Tally overall;

    std::ofstream baselineOut, filteredOut;
    if (!pathPrefix.empty())
    {
        baselineOut.open(pathPrefix + ".rrtc");
        filteredOut.open(pathPrefix + ".cbf");
        if (!baselineOut.is_open() || !filteredOut.is_open())
        {
            std::printf("cannot write %s.{rrtc,cbf}\n", pathPrefix.c_str());
            return 1;
        }
        for (std::ofstream *out : {&baselineOut, &filteredOut})
            *out << "# audited joint-space motions, one configuration per line, "
                 << auditResolution << " rad spacing\n"
                 << "# each motion begins with a '# motion <scene> <index>' marker\n";
    }

    for (const Problem &problem : problems)
    {
        if (seen[problem.scene]++ >= perScene)
            continue;

        const sdf::GridSDF field(problem.field(), UR5::reachableBounds(), voxel);
        // The barrier the assertions use, and the thicker one the filter guards so that
        // auditing against the first one passes. See ClearanceBarrier::guarding().
        const Barrier audited(robot, field, margin, selfMargin);
        const Barrier guard =
            Barrier::guarding(robot, field, margin,
                              buffer < 0.0 ? Barrier::interpolationBuffer(field) : buffer,
                              selfMargin);
        const Filter filter(guard, parameters);

        Tally &tally = tallies[problem.scene];
        ++tally.attempted;
        ++overall.attempted;

        const Barrier bare(robot, field, 0.0, selfMargin);
        const double endpoints =
            std::min(bare.worstValue(problem.start), bare.worstValue(problem.goal));
        tally.endpointClearance.push_back(endpoints);
        overall.endpointClearance.push_back(endpoints);

        // Valid upstream does not mean feasible here: our spheres do not enclose the
        // meshes MotionBenchMaker checked, and the margin sits on top of that.
        if (!audited.isSafe(problem.start) || !audited.isSafe(problem.goal))
        {
            ++tally.skipped;
            ++overall.skipped;
            continue;
        }

        std::vector<UR5::Configuration> checkedPath, rolledPath;
        const Result checked = runCollisionChecked(problem, audited, range, timeLimit,
                                                   segmentFraction,
                                                   pathPrefix.empty() ? nullptr : &checkedPath);
        const Result rolled = runFiltered(problem, audited, filter, stepSize, range, timeLimit,
                                          maxStepScale,
                                          pathPrefix.empty() ? nullptr : &rolledPath);
        writeMotion(baselineOut, problem, checkedPath);
        writeMotion(filteredOut, problem, rolledPath);
        tally.add(0, checked);
        tally.add(1, rolled);
        overall.add(0, checked);
        overall.add(1, rolled);
    }

    if (!pathPrefix.empty())
    {
        baselineOut.close();
        filteredOut.close();
        std::printf("wrote %s.rrtc and %s.cbf -- audit them against the meshes with\n"
                    "  ur5_experiments/scripts/audit_self_collision.py --path %s.cbf\n",
                    pathPrefix.c_str(), pathPrefix.c_str(), pathPrefix.c_str());
    }

    for (const auto &entry : tallies)
    {
        const Tally &tally = entry.second;
        std::printf("\n%s  (%d problems, %d skipped: start or goal inside the margin)\n",
                    entry.first.c_str(), tally.attempted, tally.skipped);
        reportRow("rrtconnect", tally, 0);
        reportRow("cbf-rrtc", tally, 1);
    }

    // The feasibility question, since it decides how much of the benchmark is usable.
    std::printf("\nEndpoint clearance at zero margin -- min(start, goal) over all spheres.\n");
    std::printf("  %-20s %8s %9s %9s %9s   %s\n", "scene", "min", "median", "max",
                "affordable", "problems by margin");
    for (const auto &entry : tallies)
    {
        std::vector<double> clearances = entry.second.endpointClearance;
        if (clearances.empty())
            continue;
        std::sort(clearances.begin(), clearances.end());
        const auto share = [&clearances](double margin)
        {
            std::size_t n = 0;
            for (const double c : clearances)
                n += (c >= margin) ? 1 : 0;
            return 100.0 * static_cast<double>(n) / static_cast<double>(clearances.size());
        };
        std::printf("  %-20s %8.4f %9.4f %9.4f %9.4f   0:%3.0f%% 0.02:%3.0f%% 0.04:%3.0f%% "
                    "0.06:%3.0f%% 0.09:%3.0f%%\n",
                    entry.first.c_str(), clearances.front(), median(clearances), clearances.back(),
                    clearances.front(), share(0.0), share(0.02), share(0.04), share(0.06),
                    share(0.09));
    }

    std::printf("\nall scenes  (%d problems, %d skipped)\n", overall.attempted, overall.skipped);
    reportRow("rrtconnect", overall, 0);
    reportRow("cbf-rrtc", overall, 1);
    std::printf("\n\"evals\" is collision checks for rrtconnect and filter calls for cbf-rrtc.\n"
                "\"unsafe/audited\" evaluates the unbuffered barrier at every state of the\n"
                "densified solution -- for cbf-rrtc that is the rollout the planner recorded\n"
                "for each edge, replayed, so it is the motion that would actually be executed,\n"
                "sampled inside each step rather than only at its boundaries. Non-zero unsafe\n"
                "invalidates a row however fast it was.\n"
                "\"missed\" counts solution edges that were not on file and had to be\n"
                "re-derived; it must be zero, or the audited motion is a different trajectory\n"
                "from the one the planner found.\n");
    return 0;
}
