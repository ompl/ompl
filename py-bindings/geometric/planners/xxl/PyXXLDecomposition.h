#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/trampoline.h>

#include <cstddef>
#include <utility>
#include <vector>

#include "ompl/base/State.h"

namespace ompl::binding::geometric
{
    namespace nb = nanobind;

    /// Out-parameter overloads call Python methods that return the value, so each overload needs its own name.
    template <typename Base, std::size_t ExtraSlots = 0>
    class PyXXLDecompositionT : public Base
    {
    public:
        NB_TRAMPOLINE(Base, 5 + ExtraSlots);

        int numLayers() const override
        {
            NB_OVERRIDE_PURE(numLayers);
        }

        bool sampleFromRegion(int r, ompl::base::State *s, const ompl::base::State *seed) const override
        {
            NB_OVERRIDE_PURE(sampleFromRegion, r, s, seed);
        }

        bool sampleFromRegion(int r, ompl::base::State *s, const ompl::base::State *seed, int layer) const override
        {
            NB_OVERRIDE_PURE_NAME("sampleFromRegionLayer", sampleFromRegion, r, s, seed, layer);
        }

        void project(const ompl::base::State *s, std::vector<double> &coord, int layer) const override
        {
            coord = callPure<std::vector<double>>("project", s, layer);
            if (coord.size() != static_cast<std::size_t>(this->getDimension()))
                throw nb::value_error("project() must return getDimension() coordinates");
        }

        void project(const ompl::base::State *s, std::vector<int> &layers) const override
        {
            layers = callPure<std::vector<int>>("projectLayers", s);
            if (layers.size() != static_cast<std::size_t>(this->numLayers()))
                throw nb::value_error("projectLayers() must return one region per layer");
        }

    protected:
        template <typename R, typename... Args>
        R callPure(const char *name, Args &&...args) const
        {
            nb::detail::ticket nb_ticket(nb_trampoline, name, true);
            return nb::cast<R>(nb_trampoline.base().attr(nb_ticket.key)(std::forward<Args>(args)...));
        }
    };
}  // namespace ompl::binding::geometric
