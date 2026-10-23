/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, Open Motion Planning Library contributors
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
 *   * Neither the name of the copyright holder nor the names of its
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

/* Author: Elias (OMPL contribution) */

#ifndef OMPL_DEMO_PLANNER_COMPARISON_GEOMETRY_
#define OMPL_DEMO_PLANNER_COMPARISON_GEOMETRY_

#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>

// Analytic rectangles/circles extruded over additional real-vector coordinates.
struct PlannerComparisonWorld
{
    std::vector<std::array<double, 4>> rectangles;
    std::vector<std::array<double, 3>> circles;

    explicit PlannerComparisonWorld(const std::string &scenario)
    {
        if (scenario == "passage" || scenario == "narrow" || scenario == "passage6d")
        {
            const double low = scenario == "passage" ? 0.45 : 0.53;
            const double high = scenario == "passage" ? 0.7 : 0.56;
            rectangles = {{-0.12, 0.12, -1.0, low}, {-0.12, 0.12, high, 1.0}};
        }
        else if (scenario == "maze")
            for (unsigned int wall = 0; wall < 3; ++wall)
            {
                const double x = -0.45 + 0.45 * wall;
                const double gap = wall == 1 ? -0.65 : 0.65;
                rectangles.push_back({x - 0.04, x + 0.04, -1.0, gap - 0.08});
                rectangles.push_back({x - 0.04, x + 0.04, gap + 0.08, 1.0});
            }
        else if (scenario == "surround")
            rectangles = {{-0.44, -0.36, -0.54, 0.6}, {0.36, 0.44, -0.54, 0.6}, {-0.44, 0.44, -0.54, -0.46}};
        else if (scenario == "blocked")
            rectangles = {{-0.12, 0.12, -1.0, 1.0}};
        else if (scenario == "clutter")
            for (double x : {-0.6, -0.3, 0.0, 0.3, 0.6})
                for (double y : {-0.7, -0.35, 0.0, 0.35, 0.7})
                    circles.push_back({x, y, 0.115});
    }

    bool valid(const ompl::base::State *state) const
    {
        const auto *v = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        for (const auto &r : rectangles)
            if (v[0] >= r[0] && v[0] <= r[1] && v[1] >= r[2] && v[1] <= r[3])
                return false;
        for (const auto &c : circles)
            if ((v[0] - c[0]) * (v[0] - c[0]) + (v[1] - c[1]) * (v[1] - c[1]) <= c[2] * c[2])
                return false;
        return true;
    }

    bool segmentValid(const ompl::base::State *a, const ompl::base::State *b) const
    {
        const auto *p = a->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        const auto *q = b->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        for (const auto &r : rectangles)
        {
            double enter = 0.0, leave = 1.0;
            bool hit = true;
            for (unsigned int axis = 0; axis < 2; ++axis)
            {
                const double delta = q[axis] - p[axis];
                if (delta == 0.0)
                {
                    if (p[axis] < r[2 * axis] || p[axis] > r[2 * axis + 1])
                        hit = false;
                }
                else
                {
                    double first = (r[2 * axis] - p[axis]) / delta;
                    double last = (r[2 * axis + 1] - p[axis]) / delta;
                    if (first > last)
                        std::swap(first, last);
                    enter = std::max(enter, first);
                    leave = std::min(leave, last);
                }
            }
            if (hit && enter <= leave)
                return false;
        }
        const double dx = q[0] - p[0], dy = q[1] - p[1];
        const double squaredLength = dx * dx + dy * dy;
        for (const auto &c : circles)
        {
            const double fraction = squaredLength == 0.0 ?
                                        0.0 :
                                        std::clamp(((c[0] - p[0]) * dx + (c[1] - p[1]) * dy) / squaredLength, 0.0, 1.0);
            const double x = p[0] + fraction * dx - c[0], y = p[1] + fraction * dy - c[1];
            if (x * x + y * y <= c[2] * c[2])
                return false;
        }
        return true;
    }
};

class PlannerComparisonMotionValidator : public ompl::base::MotionValidator
{
public:
    PlannerComparisonMotionValidator(const ompl::base::SpaceInformationPtr &si, const PlannerComparisonWorld &world,
                                     std::size_t &checks)
      : MotionValidator(si), world_(world), checks_(checks)
    {
    }

    bool checkMotion(const ompl::base::State *a, const ompl::base::State *b) const override
    {
        ++checks_;
        const bool result = si_->satisfiesBounds(b) && world_.valid(b) && world_.segmentValid(a, b);
        if (result)
            ++valid_;
        else
            ++invalid_;
        return result;
    }

    bool checkMotion(const ompl::base::State *a, const ompl::base::State *b,
                     std::pair<ompl::base::State *, double> &lastValid) const override
    {
        if (checkMotion(a, b))
            return true;
        auto *scratch = si_->allocState();
        double low = 0.0, high = 1.0;
        for (unsigned int i = 0; i < 40; ++i)
        {
            const double middle = 0.5 * (low + high);
            si_->getStateSpace()->interpolate(a, b, middle, scratch);
            if (si_->satisfiesBounds(scratch) && world_.segmentValid(a, scratch))
                low = middle;
            else
                high = middle;
        }
        lastValid.second = low;
        if (lastValid.first)
            si_->getStateSpace()->interpolate(a, b, low, lastValid.first);
        si_->freeState(scratch);
        return false;
    }

private:
    const PlannerComparisonWorld &world_;
    std::size_t &checks_;
};

#endif
