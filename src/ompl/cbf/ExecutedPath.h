#pragma once

#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include <ompl/cbf/FilteredStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/util/Exception.h>

namespace ompl::cbf
{
    /// The motion that would actually be executed for \p path.
    ///
    /// A solution path out of a geometric planner is a list of tree nodes; the motion
    /// between two of them is the CBF rollout that produced the edge, which
    /// `FilteredStateSpace` kept. This replaces every edge with that recorded
    /// trajectory, so the result is a waypoint list a robot can follow rather than a
    /// polyline through states it cannot reach in a straight line.
    ///
    /// Use this instead of `geometric::PathGeometric::interpolate()` for anything that
    /// consumes the motion -- auditing it, exporting it, driving it. `interpolate()`
    /// asks the state space for fractions of each edge, which is answered from the same
    /// records, so it gives the same states; but it budgets them by `distance()`, which
    /// is Euclidean and therefore a lower bound on the arc length of a deflected edge,
    /// so it under-samples exactly where the motion is most interesting.
    ///
    /// \p resolution is the maximum joint-space spacing, in radians, between emitted
    /// states. Each recorded step is subdivided linearly, which is exact rather than
    /// approximate: the rollout holds its control constant over a step, so the segment
    /// between consecutive waypoints *is* the executed motion. A value <= 0 emits step
    /// boundaries only.
    ///
    /// \p misses, if given, receives the number of edges that were not on file and had
    /// to be re-rolled. **It should be zero.** A non-zero value means part of the result
    /// is a re-derivation rather than a replay -- a different trajectory that happens to
    /// be safe -- and the usual cause is ledger eviction. Callers that report on the
    /// path should report this too.
    inline geometric::PathGeometric executedPath(const geometric::PathGeometric &path,
                                                 double resolution = 0.0,
                                                 std::size_t *misses = nullptr)
    {
        using Configuration = FilteredStateSpace::Configuration;

        const base::SpaceInformationPtr &si = path.getSpaceInformation();
        const auto *space = dynamic_cast<const FilteredStateSpace *>(si->getStateSpace().get());
        if (space == nullptr)
            throw Exception("executedPath requires a FilteredStateSpace");

        if (misses != nullptr)
            *misses = 0;

        geometric::PathGeometric out(si);
        if (path.getStateCount() < 2)
        {
            out = path;
            return out;
        }

        base::State *scratch = si->allocState();
        std::vector<Configuration> edge;

        for (std::size_t i = 0; i + 1 < path.getStateCount(); ++i)
        {
            const Configuration from = FilteredStateSpace::configurationOf(path.getState(i));
            const Configuration to = FilteredStateSpace::configurationOf(path.getState(i + 1));

            edge.clear();
            if (const FilteredStateSpace::EdgeRecord record = space->recordedEdge(from, to))
            {
                edge.reserve(record.size());
                for (std::size_t k = 0; k < record.size(); ++k)
                    edge.push_back(record[k]);
            }
            else
            {
                // Not on file. Re-deriving is the best that can be done, and it is what
                // every consumer of this path used to get unconditionally, but it is a
                // different trajectory from the one the planner validated -- so say so.
                if (misses != nullptr)
                    ++*misses;
                FilteredStateSpace::Rollout rollout = space->roll(from, to, 1.0);
                edge = std::move(rollout.waypoints);
                if (edge.empty())
                    edge.push_back(from);
                edge.back() = to;
            }

            if (i == 0)
            {
                FilteredStateSpace::setState(scratch, edge.front());
                out.append(scratch);
            }

            for (std::size_t k = 0; k + 1 < edge.size(); ++k)
            {
                const Configuration &a = edge[k];
                const Configuration &b = edge[k + 1];
                const double span = (b - a).norm();
                const auto parts =
                    resolution > 0.0 && span > resolution
                        ? static_cast<std::size_t>(std::ceil(span / resolution))
                        : std::size_t{1};

                for (std::size_t part = 1; part < parts; ++part)
                {
                    const double t = static_cast<double>(part) / static_cast<double>(parts);
                    FilteredStateSpace::setState(scratch, a + t * (b - a));
                    out.append(scratch);
                }
                FilteredStateSpace::setState(scratch, b);
                out.append(scratch);
            }
        }

        si->freeState(scratch);
        return out;
    }
}  // namespace ompl::cbf
