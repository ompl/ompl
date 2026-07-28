#include "ompl/cbf/CBFControlFilter.h"

#include <algorithm>
#include <stdexcept>

#include <qpmad/solver.h>

namespace
{
    constexpr int nJoints = ompl::cbf::ClearanceBarrier::nJoints;
    constexpr int nSpheres = ompl::cbf::ClearanceBarrier::nSpheres;

    // Below this, a returned control counts as "the nominal, untouched". qpmad
    // returns the nominal bit-for-bit when nothing activates, so this only needs
    // to absorb rounding.
    constexpr double unchangedTolerance = 1e-12;
}  // namespace

/// All sizes are fixed at compile time, so a solve does no heap allocation.
struct ompl::cbf::CBFControlFilter::Solver
{
    using Backend = qpmad::SolverTemplate<double, nJoints, 1, nSpheres>;

    Backend backend;
    // qpmad factorizes the Hessian in place, so it gets a scratch copy each solve.
    Eigen::Matrix<double, nJoints, nJoints> hessian{Eigen::Matrix<double, nJoints, nJoints>::Zero()};
    Eigen::Matrix<double, nJoints, 1> objective;
    Eigen::Matrix<double, nSpheres, 1> rowLower;
    // One-sided rows: no upper limit on how fast clearance may grow.
    Eigen::Matrix<double, nSpheres, 1> rowUpper{
        Eigen::Matrix<double, nSpheres, 1>::Constant(std::numeric_limits<double>::infinity())};
    ClearanceBarrier::Evaluation evaluation;
};

ompl::cbf::CBFControlFilter::CBFControlFilter(const ClearanceBarrier &barrier)
  : CBFControlFilter(barrier, Parameters())
{
}

ompl::cbf::CBFControlFilter::CBFControlFilter(const ClearanceBarrier &barrier, const Parameters &parameters)
  : barrier_(barrier), parameters_(parameters), solver_(std::make_unique<Solver>())
{
}

ompl::cbf::CBFControlFilter::~CBFControlFilter() = default;

void ompl::cbf::CBFControlFilter::controlBounds(const Configuration &q, double duration, Control &lower,
                                                Control &upper) const
{
    const Configuration jointLower = robots::UR5::lowerBounds();
    const Configuration jointUpper = robots::UR5::upperBounds();
    const double dt = duration;

    for (Eigen::Index j = 0; j < nJoints; ++j)
    {
        lower[j] = -parameters_.maxSpeed[j];
        upper[j] = parameters_.maxSpeed[j];

        if (parameters_.respectJointLimits)
        {
            // u_j must keep q_j + u_j*dt inside [jointLower, jointUpper].
            lower[j] = std::max(lower[j], (jointLower[j] - q[j]) / dt);
            upper[j] = std::min(upper[j], (jointUpper[j] - q[j]) / dt);
        }

        // A joint already outside its limits gives an empty interval. Freezing it
        // keeps the QP well posed and lets the remaining joints still move.
        if (lower[j] > upper[j])
            lower[j] = upper[j] = 0.0;
    }
}

ompl::cbf::ControlFilter::Status ompl::cbf::CBFControlFilter::filter(const Configuration &q,
                                                                    const Control &nominal, double duration,
                                                                    Control &filtered) const
{
    Diagnostics ignored;
    return filter(q, nominal, duration, filtered, ignored);
}

ompl::cbf::ControlFilter::Status ompl::cbf::CBFControlFilter::filter(const Configuration &q,
                                                                    const Control &nominal, double duration,
                                                                    Control &filtered,
                                                                    Diagnostics &diagnostics) const
{
    // A non-positive step has no meaningful CBF condition, and backward
    // propagation through a projection is not defined.
    if (duration <= 0.0)
    {
        filtered.setZero();
        return Status::Blocked;
    }

    Solver &solver = *solver_;
    ClearanceBarrier::Evaluation &evaluation = solver.evaluation;
    barrier_.evaluate(q, evaluation);

    diagnostics.worstValue = evaluation.values[static_cast<Eigen::Index>(evaluation.worst)];
    diagnostics.worstSphere = evaluation.worst;
    diagnostics.inBounds = evaluation.inBounds;
    diagnostics.solverIterations = 0;

    // Outside the baked field, GridSDF clamps and over-reports clearance, so the
    // barrier cannot be trusted. Refusing to move is the only safe answer.
    if (!evaluation.inBounds)
    {
        filtered.setZero();
        return Status::Blocked;
    }

    Control lower;
    Control upper;
    controlBounds(q, duration, lower, upper);

    // minimize 0.5 u^T W u - (W uNom)^T u, i.e. H = W and objective = -W uNom.
    solver.hessian.diagonal() = parameters_.weights;
    solver.objective = -parameters_.weights.cwiseProduct(nominal);

    // Discrete-time CBF: (dh_i/dq) u >= -gamma h_i / dt.
    solver.rowLower = evaluation.values * (-parameters_.gamma / duration);

    try
    {
        const auto status = solver.backend.solve(filtered, solver.hessian, solver.objective, lower, upper,
                                                 evaluation.rows, solver.rowLower, solver.rowUpper);
        diagnostics.solverIterations = solver.backend.getNumberOfInequalityIterations();
        if (status != Solver::Backend::OK)
        {
            filtered.setZero();
            return Status::Blocked;
        }
    }
    catch (const std::exception &)
    {
        // qpmad signals infeasibility by throwing: the robot is cornered, and no
        // control satisfies every clearance row.
        filtered.setZero();
        return Status::Blocked;
    }

    return (filtered - nominal).norm() <= unchangedTolerance ? Status::Unchanged : Status::Filtered;
}
