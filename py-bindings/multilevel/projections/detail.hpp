#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include <utility>

#include "ompl/base/StateSpace.h"
#include "ompl/multilevel/datastructures/projections/FiberedProjection.h"

namespace ompl::binding::multilevel::detail
{
    template <typename ProjectionT>
    void bindFiberedProjection(nanobind::module_ &m, const char *name)
    {
        nanobind::class_<ProjectionT, ompl::multilevel::FiberedProjection>(m, name).def(
            "__init__",
            [](ProjectionT *self, ompl::base::StateSpacePtr bundleSpace, ompl::base::StateSpacePtr baseSpace)
            {
                new (self) ProjectionT(std::move(bundleSpace), std::move(baseSpace));
                // ProjectionFactory does this after construction; without it lift() uses a null fiber sampler.
                self->makeFiberSpace();
            },
            nanobind::arg("bundleSpace"), nanobind::arg("baseSpace"));
    }
}  // namespace ompl::binding::multilevel::detail
