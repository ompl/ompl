#pragma once

#include <algorithm>
#include <cstddef>
#include <limits>

#include <ompl/robots/UR5.h>

namespace ompl::demo
{
    /// Worst sphere-sphere overlap on the UR5, measured independently of the barrier.
    ///
    /// This exists to be able to *contradict* `cbf::ClearanceBarrier`. The barrier now
    /// carries self-collision rows, but only for the pairs in `UR5::selfPairs()`, and that
    /// table is the product of a sampled search (`scripts/generate_ur5_self_pairs.py`) that
    /// drops the great majority of pairs. If the search was wrong — a pair dropped as
    /// never-approaching that does in fact approach — nothing inside the barrier can
    /// notice, because the barrier is not looking at that pair any more.
    ///
    /// So this walks every pair, from the sphere table alone, and must stay that way. The
    /// moment it is rewritten in terms of `selfPairs()` it stops being evidence and starts
    /// agreeing with whatever it was meant to check.
    ///
    /// Returns the most negative `|p_i - p_j| - (r_i + r_j)`; >= 0 means the sphere model
    /// is clear of itself. Spheres less than two frames apart are skipped: those are
    /// rigidly attached or share an axis, so they touch by construction and no planner can
    /// do anything about it. That is the same rule the PyBullet side applies to link pairs,
    /// and it is deliberately cruder than the barrier's own — it makes no use of the band,
    /// so a pair the band discarded is still audited here.
    inline double worstSelfOverlap(const robots::UR5 &robot, const robots::UR5::Configuration &q)
    {
        const robots::UR5::SphereCenters centers = robot.sphereCenters(q);
        const auto &spheres = robots::UR5::spheres();

        double worst = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < robots::UR5::nSpheres; ++i)
            for (std::size_t j = i + 1; j < robots::UR5::nSpheres; ++j)
            {
                const std::size_t a = spheres[i].frame;
                const std::size_t b = spheres[j].frame;
                const std::size_t gap = a > b ? a - b : b - a;
                if (gap < 2)
                    continue;
                const double distance =
                    (centers.col(static_cast<Eigen::Index>(i)) - centers.col(static_cast<Eigen::Index>(j)))
                        .norm() -
                    (spheres[i].radius + spheres[j].radius);
                worst = std::min(worst, distance);
            }
        return worst;
    }
}  // namespace ompl::demo
