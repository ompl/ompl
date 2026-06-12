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

#define BOOST_TEST_MODULE SubspaceStateSpaceTest
#include <boost/test/unit_test.hpp>

#include <Eigen/Core>

#include <memory>
#include <vector>

#include <ompl/base/ConstrainedSpaceInformation.h>
#include <ompl/base/Constraint.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/SubspaceStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include <ompl/util/Exception.h>

namespace ob = ompl::base;

namespace
{
    // Reference ambient bounds for a hypothetical 7-dimensional space.
    auto makeAmbientBounds() -> ob::RealVectorBounds
    {
        ob::RealVectorBounds b(7);
        for (unsigned int i = 0; i < 7; ++i)
        {
            b.setLow(i, -static_cast<double>(i + 1));
            b.setHigh(i, static_cast<double>(i + 1));
        }
        return b;
    }

    // Trivial constraint used to verify SubspaceStateSpace composes with
    // ProjectedStateSpace: pins the first active coordinate to 0.
    class FirstCoordZero : public ob::Constraint
    {
    public:
        explicit FirstCoordZero(unsigned int ambient_dim) : ob::Constraint(ambient_dim, 1)
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

BOOST_AUTO_TEST_SUITE(SubspaceStateSpace)

BOOST_AUTO_TEST_CASE(Construct_RejectsEmptyActiveIndices)
{
    BOOST_CHECK_THROW(ob::SubspaceStateSpace(makeAmbientBounds(), {}, std::vector<double>(7, 0.0)), ompl::Exception);
}

BOOST_AUTO_TEST_CASE(Construct_RejectsDuplicateActiveIndices)
{
    BOOST_CHECK_THROW(ob::SubspaceStateSpace(makeAmbientBounds(), {1, 2, 2}, std::vector<double>(7, 0.0)),
                      ompl::Exception);
}

BOOST_AUTO_TEST_CASE(Construct_RejectsOutOfRangeActiveIndex)
{
    BOOST_CHECK_THROW(ob::SubspaceStateSpace(makeAmbientBounds(), {0, 7}, std::vector<double>(7, 0.0)),
                      ompl::Exception);
}

BOOST_AUTO_TEST_CASE(Construct_RejectsFrozenLengthMismatch)
{
    BOOST_CHECK_THROW(ob::SubspaceStateSpace(makeAmbientBounds(), {0, 1}, std::vector<double>(6, 0.0)),
                      ompl::Exception);
}

BOOST_AUTO_TEST_CASE(Bounds_ProjectedOntoActiveIndicesInOrder)
{
    // Active indices given out of order — the subspace bounds must follow the same order.
    auto subspace = ob::SubspaceStateSpace(makeAmbientBounds(), {5, 2, 6}, std::vector<double>(7, 0.0));

    BOOST_CHECK_EQUAL(subspace.getDimension(), 3u);
    BOOST_CHECK_EQUAL(subspace.getAmbientDimension(), 7u);

    BOOST_CHECK_CLOSE(subspace.getBounds().low[0], -6.0, 1e-9);
    BOOST_CHECK_CLOSE(subspace.getBounds().high[0], 6.0, 1e-9);
    BOOST_CHECK_CLOSE(subspace.getBounds().low[1], -3.0, 1e-9);
    BOOST_CHECK_CLOSE(subspace.getBounds().high[1], 3.0, 1e-9);
    BOOST_CHECK_CLOSE(subspace.getBounds().low[2], -7.0, 1e-9);
    BOOST_CHECK_CLOSE(subspace.getBounds().high[2], 7.0, 1e-9);
}

BOOST_AUTO_TEST_CASE(ExpandToFull_WritesActiveSlotsAndPreservesFrozen)
{
    std::vector<double> frozen{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
    std::vector<std::size_t> active{1, 4, 6};
    auto subspace = std::make_shared<ob::SubspaceStateSpace>(makeAmbientBounds(), active, frozen);

    ob::ScopedState<> state(subspace);
    state[0] = -1.5;  // → ambient[1]
    state[1] = 2.5;   // → ambient[4]
    state[2] = -3.5;  // → ambient[6]

    std::vector<double> full;
    subspace->expandToFull(state.get(), full);

    BOOST_REQUIRE_EQUAL(full.size(), 7u);
    BOOST_CHECK_CLOSE(full[0], 0.1, 1e-9);
    BOOST_CHECK_CLOSE(full[1], -1.5, 1e-9);
    BOOST_CHECK_CLOSE(full[2], 0.3, 1e-9);
    BOOST_CHECK_CLOSE(full[3], 0.4, 1e-9);
    BOOST_CHECK_CLOSE(full[4], 2.5, 1e-9);
    BOOST_CHECK_CLOSE(full[5], 0.6, 1e-9);
    BOOST_CHECK_CLOSE(full[6], -3.5, 1e-9);

    // Allocating overload must agree with the in-place form.
    auto full2 = subspace->expandToFull(state.get());
    BOOST_CHECK_EQUAL_COLLECTIONS(full.begin(), full.end(), full2.begin(), full2.end());
}

BOOST_AUTO_TEST_CASE(SetFrozenValues_UpdatesExpansion)
{
    std::vector<double> frozen(7, 0.0);
    std::vector<std::size_t> active{0, 2};
    auto subspace = std::make_shared<ob::SubspaceStateSpace>(makeAmbientBounds(), active, frozen);

    std::vector<double> updated{9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0};
    subspace->setFrozenValues(updated);

    BOOST_CHECK_EQUAL_COLLECTIONS(subspace->getFrozenValues().begin(), subspace->getFrozenValues().end(),
                                  updated.begin(), updated.end());

    ob::ScopedState<> s(subspace);
    s[0] = -0.25;
    s[1] = 0.75;
    auto full = subspace->expandToFull(s.get());
    BOOST_CHECK_CLOSE(full[0], -0.25, 1e-9);  // active
    BOOST_CHECK_CLOSE(full[1], 8.0, 1e-9);    // frozen (post-update)
    BOOST_CHECK_CLOSE(full[2], 0.75, 1e-9);   // active
    BOOST_CHECK_CLOSE(full[3], 6.0, 1e-9);    // frozen (post-update)

    BOOST_CHECK_THROW(subspace->setFrozenValues(std::vector<double>(6, 0.0)), ompl::Exception);
}

BOOST_AUTO_TEST_CASE(ComposesWith_ProjectedStateSpace)
{
    // The sampler-based subgroup prototype could not compose with
    // ConstrainedStateSpace; inheriting from RealVectorStateSpace must.
    // ConstrainedStateSpace::setup() requires a SpaceInformation be
    // associated first — ConstrainedSpaceInformation's constructor does that
    // wiring, so we go through it like real planning code would.
    auto subspace = std::make_shared<ob::SubspaceStateSpace>(makeAmbientBounds(), std::vector<std::size_t>{0, 1, 2},
                                                             std::vector<double>(7, 0.0));
    auto constraint = std::make_shared<FirstCoordZero>(subspace->getDimension());
    auto projected = std::make_shared<ob::ProjectedStateSpace>(subspace, constraint);
    auto csi = std::make_shared<ob::ConstrainedSpaceInformation>(projected);

    BOOST_REQUIRE_NO_THROW(projected->setup());
    BOOST_CHECK_EQUAL(projected->getDimension(), subspace->getDimension());
}

BOOST_AUTO_TEST_SUITE_END()
