#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "ompl/base/State.h"

namespace ompl::binding::base
{
    // OMPL fills the geodesic even when it fails, so the states are handed back owned either way.
    template <typename Space>
    std::pair<bool, std::vector<std::shared_ptr<ompl::base::State>>> ownedGeodesic(const std::shared_ptr<Space> &space,
                                                                                   const ompl::base::State *from,
                                                                                   const ompl::base::State *to,
                                                                                   bool interpolate)
    {
        std::vector<ompl::base::State *> geodesic;
        bool ok = false;
        try
        {
            ok = space->discreteGeodesic(from, to, interpolate, &geodesic);
        }
        catch (...)
        {
            for (ompl::base::State *state : geodesic)
                space->freeState(state);
            throw;
        }

        std::vector<std::shared_ptr<ompl::base::State>> owned;
        owned.reserve(geodesic.size());
        for (ompl::base::State *state : geodesic)
            owned.emplace_back(state, [space](ompl::base::State *s) { space->freeState(s); });
        return {ok, std::move(owned)};
    }
}  // namespace ompl::binding::base
