/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Rice University
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
 *   * Neither the name of the Rice University nor the names of its
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

/* Author: Clayton W. Ramsey */

#ifndef OMPL_BASE_SPACES_FLAT_CHART_
#define OMPL_BASE_SPACES_FLAT_CHART_

#include "ompl/base/StateSpace.h"
#include "ompl/util/ClassForward.h"

#include <Eigen/Core>

#include <vector>

namespace ompl::base
{
    /// @cond IGNORE
    OMPL_CLASS_FORWARD(FlatChart);
    /// @endcond

    /** \brief Vector coordinates around a point of a flat output space, used for computing motions between states.

        In differentially flat systems, the primitive motion between states is a polynomial.
        A polynomial needs a vector space to run in, and a flat output space such as SO(2) isn't one.
        The chart at a state \e from lays a vector space over the flat output space with \e from at the
        origin.
        \ref difference reads off the coordinates of a second state in that chart, and \ref advance maps
        coordinates back to a state.

        Charts for ompl::base::RealVectorStateSpace and ompl::base::SO2StateSpace are exact, and so is any
        compound of them, such as ompl::base::SE2StateSpace or ompl::base::TorusStateSpace.
        \ref allocFlatChart builds one of those for a flat output space.
    */
    class FlatChart
    {
    public:
        /** \brief Construct a chart laying \e dimension coordinates over the flat output space, none of
            which wrap. */
        explicit FlatChart(unsigned int dimension) : dimension_(dimension), periods_(dimension, 0.)
        {
        }

        virtual ~FlatChart() = default;

        /** \brief The number of coordinates the chart lays over the flat output space. */
        unsigned int getDimension() const
        {
            return dimension_;
        }

        /** \brief The period of each coordinate, which is zero for a coordinate that doesn't wrap.

            Adding a whole number of periods to a coordinate reaches the same state, so steering can pick
            whichever of those copies of the target costs least to reach.
        */
        const std::vector<double> &getPeriods() const
        {
            return periods_;
        }

        /** \brief Write the coordinates of \e to in the chart at \e from into \e out, which needs one entry
            per coordinate.

            Where several coordinates reach \e to, this picks the one nearest the origin.
        */
        virtual void difference(const State *from, const State *to, Eigen::Ref<Eigen::VectorXd> out) const = 0;

        /** \brief Write the state sitting at coordinates \e delta in the chart at \e from into \e out.

            \e out is allowed to alias \e from.
        */
        virtual void advance(const State *from, const Eigen::Ref<const Eigen::VectorXd> &delta, State *out) const = 0;

    protected:
        /** \brief The dimension of the chart. */
        unsigned int dimension_;

        /** \brief The period of each coordinate, which is zero for a coordinate that doesn't wrap. */
        std::vector<double> periods_;
    };

    /** \brief The chart over an ompl::base::RealVectorStateSpace, whose coordinates are the displacement
        from the origin of the chart. */
    class RealVectorFlatChart : public FlatChart
    {
    public:
        /** \brief Construct a chart over a space of \e dimension dimensions. */
        explicit RealVectorFlatChart(unsigned int dimension) : FlatChart(dimension)
        {
        }

        void difference(const State *from, const State *to, Eigen::Ref<Eigen::VectorXd> out) const override;

        void advance(const State *from, const Eigen::Ref<const Eigen::VectorXd> &delta, State *out) const override;
    };

    /** \brief The chart over an ompl::base::SO2StateSpace, whose coordinate is the angle turned from the
        origin of the chart.

        Coordinates past a half turn either way wrap back into the space when they get advanced to.
    */
    class SO2FlatChart : public FlatChart
    {
    public:
        /** \brief Construct the chart, whose one coordinate wraps every full turn. */
        SO2FlatChart();

        /** \brief Write the signed angle turned from \e from to \e to the short way round, which lies
            from minus pi up to pi, into \e out. */
        void difference(const State *from, const State *to, Eigen::Ref<Eigen::VectorXd> out) const override;

        void advance(const State *from, const Eigen::Ref<const Eigen::VectorXd> &delta, State *out) const override;
    };

    /** \brief The chart over an ompl::base::CompoundStateSpace, stacking the coordinates of one chart per
        component in component order. */
    class CompoundFlatChart : public FlatChart
    {
    public:
        /** \brief Construct a chart stacking \e components, holding one chart for each component of the
            compound in component order. */
        explicit CompoundFlatChart(std::vector<FlatChartPtr> components);

        /** \brief The chart for each component, in component order. */
        const std::vector<FlatChartPtr> &getComponents() const
        {
            return components_;
        }

        void difference(const State *from, const State *to, Eigen::Ref<Eigen::VectorXd> out) const override;

        void advance(const State *from, const Eigen::Ref<const Eigen::VectorXd> &delta, State *out) const override;

    private:
        /** \brief The chart for each component, in component order. */
        std::vector<FlatChartPtr> components_;

        /** \brief The index of the first coordinate of each component. */
        std::vector<Eigen::Index> offsets_;
    };

    /** \brief Build the chart over \e space.

        Anything deriving from ompl::base::RealVectorStateSpace or ompl::base::SO2StateSpace gets the chart
        for it, and a compound gets an ompl::base::CompoundFlatChart over the charts of its components.
        Any other space makes this throw.
    */
    FlatChartPtr allocFlatChart(const StateSpacePtr &space);
}  // namespace ompl::base

#endif
