/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014 Rice University
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

/* Author: Zachary Kingston, Caleb Voss */

#include "ompl/base/spaces/constraint/AtlasStateSpace.h"
#include "ompl/base/spaces/constraint/AtlasChart.h"

#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Exception.h"

/// AtlasStateSampler

/// Public

ompl::base::AtlasStateSampler::AtlasStateSampler(const AtlasStateSpace *space) : StateSampler(space), atlas_(space)
{
}

void ompl::base::AtlasStateSampler::sampleUniform(State *state)
{
    auto astate = state->as<AtlasStateSpace::StateType>();

    const std::size_t k = atlas_->getManifoldDimension();
    Eigen::VectorXd ru(k);

    AtlasChart *c;

    // Sampling a point on the manifold.
    unsigned int tries = ompl::magic::ATLAS_STATE_SPACE_SAMPLES;
    do
    {
        // Rejection sampling to find a point inside a chart's polytope.
        do
        {
            // Pick a chart.
            c = atlas_->sampleChart();

            // Sample a point within rho_s of the center. This is done by
            // sampling uniformly on the surface and multiplying by a dist
            // whose distribution is biased according to spherical volume.
            for (std::size_t i = 0; i < k; ++i)
                ru[i] = rng_.gaussian01();

            ru *= atlas_->getRho_s() * std::pow(rng_.uniform01(), 1.0 / k) / ru.norm();
        } while (tries-- > 0 && !c->inPolytope(ru));

        // Project. Will need to try again if this fails.
    } while (tries > 0 && !c->psi(ru, *astate));

    if (tries == 0)
    {
        // Consider decreasing rho and/or the exploration paramter if this
        // becomes a problem.
        OMPL_WARN("ompl::base::AtlasStateSpace::sampleUniform(): "
                  "Took too long; returning center of a random chart.");
        atlas_->copyState(astate, c->getOrigin());
    }

    space_->enforceBounds(state);

    // Extend polytope of neighboring chart wherever point is near the border.
    c->psiInverse(*astate, ru);
    c->borderCheck(ru);
    astate->setChart(atlas_->owningChart(astate));
}

void ompl::base::AtlasStateSampler::sampleUniformNear(State *state, const State *near, const double dist)
{
    // Find the chart that the starting point is on.
    auto astate = state->as<AtlasStateSpace::StateType>();
    auto anear = near->as<AtlasStateSpace::StateType>();

    const std::size_t k = atlas_->getManifoldDimension();

    Eigen::VectorXd ru(k), uoffset(k);

    AtlasChart *c = atlas_->getChart(anear, true);
    if (c == nullptr)
    {
        OMPL_ERROR("ompl::base::AtlasStateSpace::sampleUniformNear(): "
                   "Sampling failed because chart creation failed! Falling back to uniform sample.");
        sampleUniform(state);
        return;
    }

    // Sample a point from the starting chart.
    c->psiInverse(*anear, ru);
    unsigned int tries = ompl::magic::ATLAS_STATE_SPACE_SAMPLES;
    do
    {
        // Sample within dist
        for (std::size_t i = 0; i < k; ++i)
            uoffset[i] = ru[i] + rng_.gaussian01();

        uoffset *= dist * std::pow(rng_.uniform01(), 1.0 / k) / uoffset.norm();
    } while (--tries > 0 && !c->psi(uoffset, *astate));  // Try again if we can't project.

    if (tries == 0)
    {
        // Consider decreasing the dist argument if this becomes a
        // problem. Check planner code to see how it gets chosen.
        OMPL_WARN("ompl::base:::AtlasStateSpace::sampleUniformNear(): "
                  "Took too long; returning initial point.");
        atlas_->copyState(state, near);
    }

    space_->enforceBounds(state);

    c->psiInverse(*astate, ru);
    if (!c->inPolytope(ru))
        c = atlas_->getChart(astate, true);
    else
        c->borderCheck(ru);

    astate->setChart(c);
}

void ompl::base::AtlasStateSampler::sampleGaussian(State *state, const State *mean, const double stdDev)
{
    auto astate = state->as<AtlasStateSpace::StateType>();
    auto amean = mean->as<AtlasStateSpace::StateType>();

    const std::size_t k = atlas_->getManifoldDimension();
    Eigen::VectorXd ru(k), rand(k);

    AtlasChart *c = atlas_->getChart(amean, true);
    if (c == nullptr)
    {
        OMPL_ERROR("ompl::base::AtlasStateSpace::sampleGaussian(): "
                   "Sampling failed because chart creation failed! Falling back to uniform sample.");
        sampleUniform(state);
        return;
    }

    c->psiInverse(*amean, ru);

    // Sample a point in a normal distribution on the starting chart.
    unsigned int tries = ompl::magic::ATLAS_STATE_SPACE_SAMPLES;

    do
    {
        for (std::size_t i = 0; i < k; i++)
            rand[i] = ru[i] + rng_.gaussian(0, stdDev);
    } while (--tries > 0 && !c->psi(rand, *astate));  // Try again if we can't project.

    if (tries == 0)
    {
        OMPL_WARN("ompl::base::AtlasStateSpace::sampleUniforGaussian(): "
                  "Took too long; returning initial point.");
        atlas_->copyState(state, mean);
    }

    space_->enforceBounds(state);

    c->psiInverse(*astate, ru);
    if (!c->inPolytope(ru))
        c = atlas_->getChart(astate, true);
    else
        c->borderCheck(ru);

    astate->setChart(c);
}

/// AtlasStateSpace

/// Public

ompl::base::AtlasStateSpace::AtlasStateSpace(const StateSpacePtr &ambientSpace, const ConstraintPtr &constraint,
                                             bool separate)
  : ConstrainedStateSpace(ambientSpace, constraint)
  , biasFunction_([](AtlasChart *) -> double { return 1; })
  , separate_(separate)
{
    setRho(delta_ * ompl::magic::ATLAS_STATE_SPACE_RHO_MULTIPLIER);
    setAlpha(ompl::magic::ATLAS_STATE_SPACE_ALPHA);
    setExploration(ompl::magic::ATLAS_STATE_SPACE_EXPLORATION);

    setName("Atlas" + space_->getName());

    chartNN_.setDistanceFunction(
        [&](const NNElement &e1, const NNElement &e2) -> double { return distance(e1.first, e2.first); });
}

ompl::base::AtlasStateSpace::~AtlasStateSpace()
{
    // Delete anchors first so clear does no reinitialization.
    for (auto anchor : anchors_)
        freeState(anchor);

    anchors_.clear();
    clear();
}

void ompl::base::AtlasStateSpace::clear()
{
    // Delete the non-anchor charts
    for (auto chart : charts_)
        delete chart;
    charts_.clear();

    std::vector<NNElement> nnList;
    chartNN_.list(nnList);
    for (auto &chart : nnList)
    {
        const State *state = chart.first;
        freeState(const_cast<State *>(state));
    }

    chartNN_.clear();
    chartPDF_.clear();

    // Reinstate the anchor charts
    for (auto anchor : anchors_)
        newChart(anchor);

    ConstrainedStateSpace::clear();
}

ompl::base::AtlasChart *ompl::base::AtlasStateSpace::anchorChart(const ompl::base::State *state) const
{
    auto anchor = cloneState(state)->as<StateType>();
    anchors_.push_back(anchor);

    // This could fail with an exception. We cannot recover if that happens.
    AtlasChart *chart = newChart(anchor);
    if (chart == nullptr)
        throw ompl::Exception("ompl::base::AtlasStateSpace::anchorChart(): "
                              "Initial chart creation failed. Cannot proceed.");

    return chart;
}

ompl::base::AtlasChart *ompl::base::AtlasStateSpace::newChart(const StateType *state) const
{
    AtlasChart *chart;
    StateType *cstate = nullptr;

    try
    {
        cstate = cloneState(state)->as<StateType>();
        chart = new AtlasChart(this, cstate);
    }
    catch (ompl::Exception &e)
    {
        OMPL_ERROR("ompl::base::AtlasStateSpace::newChart(): "
                   "Failed because manifold looks degenerate here.");

        if (cstate != nullptr)
            freeState(cstate);

        return nullptr;
    }

    // Ensure all charts respect boundaries of the new one, and vice versa, but
    // only look at nearby ones (within 2*rho).
    if (separate_)
    {
        std::vector<NNElement> nearbyCharts;
        chartNN_.nearestR(std::make_pair(cstate, 0), 2 * rho_s_, nearbyCharts);

        for (auto &&near : nearbyCharts)
        {
            AtlasChart *other = charts_[near.second];
            AtlasChart::generateHalfspace(other, chart);

            chartPDF_.update(chartPDF_.getElements()[near.second], biasFunction_(other));
        }
    }

    chartNN_.add(std::make_pair(cstate, charts_.size()));
    charts_.push_back(chart);
    chartPDF_.add(chart, biasFunction_(chart));

    return chart;
}

ompl::base::AtlasChart *ompl::base::AtlasStateSpace::sampleChart() const
{
    if (charts_.empty())
        throw ompl::Exception("ompl::base::AtlasStateSpace::sampleChart(): "
                              "Atlas sampled before any charts were made. Use AtlasStateSpace::anchorChart() first.");

    return chartPDF_.sample(rng_.uniform01());
}

ompl::base::AtlasChart *ompl::base::AtlasStateSpace::getChart(const StateType *state, bool force, bool *created) const
{
    AtlasChart *c = state->getChart();
    if (c == nullptr || force)
    {
        c = owningChart(state);

        if (c == nullptr)
        {
            c = newChart(state);
            if (created != nullptr)
                *created = true;
        }

        if (c != nullptr)
            state->setChart(c);
    }

    return c;
}

ompl::base::AtlasChart *ompl::base::AtlasStateSpace::owningChart(const StateType *state) const
{
    Eigen::VectorXd u_t(k_);
    auto temp = allocState()->as<StateType>();

    std::vector<NNElement> nearby;
    chartNN_.nearestR(std::make_pair(state, 0), rho_, nearby);

    double best = epsilon_;
    AtlasChart *chart = nullptr;
    for (auto & near : nearby)
    {
        // The point must lie in the chart's validity region and polytope
        auto owner = charts_[near.second];
        owner->psiInverse(*state, u_t);
        owner->phi(u_t, *temp);

        double far;
        if (owner->inPolytope(u_t)                       // in polytope
            && (far = distance(state, temp)) < epsilon_  // within epsilon
            && far < best)
        {
            best = far;
            chart = owner;
        }
    }

    freeState(temp);
    return chart;
}

bool ompl::base::AtlasStateSpace::discreteGeodesic(const State *from, const State *to, bool interpolate,
                                                   std::vector<ompl::base::State *> *geodesic) const
{
    auto &&svc = si_->getStateValidityChecker();

    // We can't traverse the manifold if we don't start on it.
    if (!constraint_->isSatisfied(from) || !(interpolate || svc->isValid(from)))
        return false;

    auto afrom = from->as<StateType>();
    auto ato = to->as<StateType>();

    // Try to get starting chart from `from` state.
    AtlasChart *c = getChart(afrom);
    if (c == nullptr)
        return false;

    // Save a copy of the from state.
    if (geodesic != nullptr)
    {
        geodesic->clear();
        geodesic->push_back(cloneState(from));
    }

    // No need to traverse the manifold if we are already there
    const double tolerance = delta_;
    const double distTo = distance(from, to);
    if (distTo <= tolerance)
        return true;

    // Traversal stops if the ball of radius distMax centered at x_from is left
    const double distMax = lambda_ * distTo;

    // Create a scratch state to use for movement.
    auto scratch = cloneState(from)->as<StateType>();
    auto temp = allocState()->as<StateType>();

    // Project from and to points onto the chart
    Eigen::VectorXd u_j(k_), u_b(k_);
    c->psiInverse(*scratch, u_j);
    c->psiInverse(*ato, u_b);

    bool done = false;
    std::size_t chartsCreated = 0;
    double dist = 0;

    double factor = 1;

    do
    {
        if (factor < delta_)
            break;

        // Take a step towards the final state
        u_j += factor * delta_ * (u_b - u_j).normalized();

        // will also bork on non-finite numbers
        const bool onManifold = c->psi(u_j, *temp);
        if (!onManifold)
        {
            done = false;
            break;
        }

        const double step = distance(scratch, temp);

        if (step < std::numeric_limits<double>::epsilon())
            break;

        const bool exceedStepSize = step >= lambda_ * delta_;
        if (exceedStepSize)
        {
            factor *= backoff_;
            continue;
        }

        dist += step;

        // Update state
        copyState(scratch, temp);
        scratch->setChart(c);

        if (!(interpolate || svc->isValid(scratch))     // not valid
            || distance(from, scratch) > distMax        // exceed max dist
            || dist > distMax                           // exceed wandering
            || chartsCreated > maxChartsPerExtension_)  // exceed chart limit
        {
            done = false;
            break;
        }

        // Check if we left the validity region or polytope of the chart.
        c->phi(u_j, *temp);

        // Find or make a new chart if new state is off of current chart
        if (distance(scratch, temp) > epsilon_  // exceeds epsilon
            || delta_ / step < cos_alpha_       // exceeds angle
            || !c->inPolytope(u_j))             // outside polytope
        {
            bool created = false;
            if ((c = getChart(scratch, true, &created)) == nullptr)
            {
                OMPL_ERROR("Treating singularity as an obstacle.");
                done = false;
                break;
            }
            chartsCreated += created;

            // Re-project onto the next chart.
            c->psiInverse(*scratch, u_j);
            c->psiInverse(*ato, u_b);
        }

        done = distance(scratch, to) <= delta_;
        factor = 1;

        // Keep the state in a list, if requested.
        if (geodesic != nullptr)
            geodesic->push_back(cloneState(scratch));

    } while (!done);

    const bool ret = done && distance(to, scratch) <= delta_;
    freeState(scratch);
    freeState(temp);

    return ret;
}

double ompl::base::AtlasStateSpace::estimateFrontierPercent() const
{
    double frontier = 0;
    for (const AtlasChart *c : charts_)
        frontier += c->estimateIsFrontier() ? 1 : 0;

    return (100 * frontier) / charts_.size();
}

void ompl::base::AtlasStateSpace::printPLY(std::ostream &out) const
{
    std::stringstream v, f;
    std::size_t vcount = 0;
    std::size_t fcount = 0;
    std::vector<Eigen::VectorXd> vertices;
    for (AtlasChart *c : charts_)
    {
        vertices.clear();
        c->toPolygon(vertices);

        std::stringstream poly;
        std::size_t fvcount = 0;
        for (Eigen::VectorXd &vert : vertices)
        {
            v << vert.transpose() << "\n";
            poly << vcount++ << " ";
            fvcount++;
        }

        if (fvcount > 2)
        {
            f << fvcount << " " << poly.str() << "\n";
            fcount += 1;
        }
    }
    vertices.clear();

    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "element vertex " << vcount << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "element face " << fcount << "\n";
    out << "property list uint uint vertex_index\n";
    out << "end_header\n";
    out << v.str() << f.str();
}
