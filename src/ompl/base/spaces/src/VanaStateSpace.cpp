/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, Metron, Inc.
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
 *   * Neither the name of the Metron, Inc. nor the names of its
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

/* Author: Mark Moll */

#include "ompl/base/spaces/VanaStateSpace.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/tools/config/MagicConstants.h"
#include "ompl/util/Exception.h"
#include <queue>

using namespace ompl::base;

namespace
{
    // max # iterations for doubling turning radius for initial solution path
    constexpr unsigned int MAX_ITER = 32;
}  // namespace

VanaStateSpace::VanaStateSpace(double turningRadius, double maxPitch)
  : VanaStateSpace(turningRadius, {-maxPitch, maxPitch})
{
}
VanaStateSpace::VanaStateSpace(double turningRadius, std::pair<double, double> pitchRange)
  : rho_(turningRadius), minPitch_(pitchRange.first), maxPitch_(pitchRange.second), dubinsSpace_(turningRadius)
{
    setName("Vana" + getName());
    type_ = STATE_SPACE_VANA;
    auto space = std::make_shared<RealVectorStateSpace>(4);
    ompl::base::RealVectorBounds pitchBounds(4);
    pitchBounds.setLow(3, minPitch_);
    pitchBounds.setHigh(3, maxPitch_);
    space->setBounds(pitchBounds);
    addSubspace(space, 1.0);
    addSubspace(std::make_shared<SO2StateSpace>(), 0.5);
    lock();
}

State *VanaStateSpace::allocState() const
{
    auto *state = new StateType();
    allocStateComponents(state);
    return state;
}

void VanaStateSpace::registerProjections()
{
    class VanaDefaultProjection : public ProjectionEvaluator
    {
    public:
        VanaDefaultProjection(const StateSpace *space) : ProjectionEvaluator(space)
        {
        }

        unsigned int getDimension() const override
        {
            return 3;
        }

        void defaultCellSizes() override
        {
            cellSizes_.resize(3);
            bounds_ = space_->as<VanaStateSpace>()->getBounds();
            bounds_.resize(3);
            cellSizes_[0] = (bounds_.high[0] - bounds_.low[0]) / magic::PROJECTION_DIMENSION_SPLITS;
            cellSizes_[1] = (bounds_.high[1] - bounds_.low[1]) / magic::PROJECTION_DIMENSION_SPLITS;
            cellSizes_[2] = (bounds_.high[2] - bounds_.low[2]) / magic::PROJECTION_DIMENSION_SPLITS;
        }

        void project(const State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
        {
            projection = Eigen::Map<const Eigen::VectorXd>(
                state->as<StateType>()->as<RealVectorStateSpace::StateType>(0)->values, 3);
        }
    };

    registerDefaultProjection(std::make_shared<VanaDefaultProjection>(this));
}

DubinsStateSpace::StateType *VanaStateSpace::get2DPose(double x, double y, double yaw) const
{
    auto state = dubinsSpace_.allocState()->as<DubinsStateSpace::StateType>();
    state->setXY(x, y);
    state->setYaw(yaw);
    return state;
}

bool VanaStateSpace::decoupled(const State *state1, const State *state2, double radius, VanaPath &result,
                               DubinsStateSpace::StateType *endSZ) const
{
    auto s1 = state1->as<StateType>();
    auto s2 = state2->as<StateType>();

    result.horizontalRadius_ = radius;
    // note that we are exploiting properties of the memory layout of state types
    result.pathXY_ = dubinsSpace_.dubins(state1, state2, radius);

    result.verticalRadius_ = 1. / std::sqrt(1. / (rho_ * rho_) - 1. / (radius * radius));
    if (!std::isfinite(result.verticalRadius_))
    {
        return false;
    }

    result.startSZ_->setXY(0., (*s1)[2]);
    result.startSZ_->setYaw(s1->pitch());
    endSZ->setXY(radius * result.pathXY_.length(), (*s2)[2]);
    endSZ->setYaw(s2->pitch());
    result.pathSZ_ = dubinsSpace_.dubins(result.startSZ_, endSZ, result.verticalRadius_);

    // in three cases the result is invalid:
    // 1. path of type CCC (i.e., RLR or LRL)
    // 2. pitch smaller than minPitch_
    // 3. pitch greater than maxPitch_
    if ((result.pathSZ_.type_[1] != DubinsStateSpace::DUBINS_STRAIGHT) ||
        (result.pathSZ_.type_[0] == DubinsStateSpace::DUBINS_RIGHT &&
         s1->pitch() - result.pathSZ_.length_[0] < minPitch_) ||
        (result.pathSZ_.type_[0] == DubinsStateSpace::DUBINS_LEFT &&
         s1->pitch() + result.pathSZ_.length_[0] > maxPitch_))
    {
        return false;
    }

    return true;
}

std::optional<VanaStateSpace::VanaPath> VanaStateSpace::vanaPath(const State *state1, const State *state2) const
{
    double radiusMultiplier = 2.;
    auto scratch = dubinsSpace_.allocState()->as<DubinsStateSpace::StateType>();
    VanaPath path;

    unsigned int iter = 0;
    while (!decoupled(state1, state2, rho_ * radiusMultiplier, path, scratch) && iter++ < MAX_ITER)
    {
        radiusMultiplier *= 2.;
    }
    if (iter >= MAX_ITER)
    {
        OMPL_ERROR("Maximum number of iterations exceeded in VanaStateSpace::VanaPath");
        dubinsSpace_.freeState(scratch);
        return {};
    }

    // Local optimalization between horizontal and vertical radii
    double step = .1, radiusMultiplier2;
    VanaPath path2;
    while (std::abs(step) > tolerance_)
    {
        radiusMultiplier2 = std::max(1., radiusMultiplier + step);
        if (decoupled(state1, state2, rho_ * radiusMultiplier2, path2, scratch) && path2.length() < path.length())
        {
            radiusMultiplier = radiusMultiplier2;
            path = path2;
            step *= 2.;
        }
        else
            step *= -.1;
    }

    dubinsSpace_.freeState(scratch);
    return path;
}

double VanaStateSpace::distance(const State *state1, const State *state2) const
{
    if (auto path = vanaPath(state1, state2))
        return path->length();
    return getMaximumExtent();
}

unsigned int VanaStateSpace::validSegmentCount(const State *state1, const State *state2) const
{
    return StateSpace::validSegmentCount(state1, state2);
}

void VanaStateSpace::interpolate(const State *from, const State *to, const double t, State *state) const
{
    if (auto path = vanaPath(from, to))
        interpolate(from, to, t, *path, state);
    else
        if (from != state)
            copyState(state, from);
}

void VanaStateSpace::interpolate(const State *from, const State *to, const double t, VanaPath &path,
                                 State *state) const
{
    if (t >= 1.)
    {
        if (to != state)
            copyState(state, to);
        return;
    }
    if (t <= 0.)
    {
        if (from != state)
            copyState(state, from);
        return;
    }

    interpolate(from, path, t, state);
}

void VanaStateSpace::interpolate(const State *from, const VanaPath &path, double t, State *state) const
{
    auto intermediate = (from==state) ? dubinsSpace_.allocState() : state;
    auto s = state->as<StateType>();
    auto i = intermediate->as<DubinsStateSpace::StateType>();
    // This is exploiting internal properties of compound state spaces like DubinsStateSpace
    // and VanaStateSpace
    dubinsSpace_.interpolate(path.startSZ_, path.pathSZ_, t, intermediate, path.verticalRadius_);
    (*s)[2] = i->getY();
    s->pitch() = i->getYaw();
    dubinsSpace_.interpolate(from, path.pathXY_, t, state, path.horizontalRadius_);
    if (from==state)
        dubinsSpace_.freeState(intermediate);
}

VanaStateSpace::VanaPath::~VanaPath()
{
    static const DubinsStateSpace dubinsStateSpace_;
    dubinsStateSpace_.freeState(startSZ_);
}

VanaStateSpace::VanaPath::VanaPath()
{
    static const DubinsStateSpace dubinsStateSpace_;
    startSZ_ = dubinsStateSpace_.allocState()->as<DubinsStateSpace::StateType>();
}

VanaStateSpace::VanaPath::VanaPath(const VanaStateSpace::VanaPath &path)
  : horizontalRadius_(path.horizontalRadius_)
  , verticalRadius_(path.verticalRadius_)
  , pathXY_(path.pathXY_)
  , pathSZ_(path.pathSZ_)
{
    static const DubinsStateSpace dubinsStateSpace_;
    startSZ_ = dubinsStateSpace_.allocState()->as<DubinsStateSpace::StateType>();
    dubinsStateSpace_.copyState(startSZ_, path.startSZ_);
}

VanaStateSpace::VanaPath &VanaStateSpace::VanaPath::operator=(const VanaStateSpace::VanaPath &path)
{
    static const DubinsStateSpace dubinsStateSpace_;
    horizontalRadius_ = path.horizontalRadius_;
    verticalRadius_ = path.verticalRadius_;
    pathXY_ = path.pathXY_;
    pathSZ_ = path.pathSZ_;
    dubinsStateSpace_.copyState(startSZ_, path.startSZ_);
    return *this;
}

double VanaStateSpace::VanaPath::length() const
{
    return verticalRadius_ * pathSZ_.length();
}

namespace ompl::base
{
    std::ostream &operator<<(std::ostream &os, const VanaStateSpace::VanaPath &path)
    {
        static const DubinsStateSpace dubinsStateSpace;

        os << "VanaPath[ length = " << path.length() << ", XY=" << path.pathXY_ << ", SZ=" << path.pathSZ_
           << ", rh = " << path.horizontalRadius_ << ", rv = " << path.verticalRadius_ << ", startSZ=";
        dubinsStateSpace.printState(path.startSZ_, os);
        os << " ]";
        return os;
    }
}  // namespace ompl::base

void ompl::base::VanaMotionValidator::defaultSettings()
{
    stateSpace_ = dynamic_cast<VanaStateSpace *>(si_->getStateSpace().get());
    if (stateSpace_ == nullptr)
        throw Exception("No state space for motion validator");
}

bool ompl::base::VanaMotionValidator::checkMotion(const State *s1, const State *s2,
                                                  std::pair<State *, double> &lastValid) const
{
    auto path = stateSpace_->vanaPath(s1, s2);
    if (!path)
        return false;

    /* assume motion starts in a valid configuration so s1 is valid */
    bool result = true;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    if (nd > 1)
    {
        /* temporary storage for the checked state */
        State *test = si_->allocState();

        for (int j = 1; j < nd; ++j)
        {
            stateSpace_->interpolate(s1, *path, (double)j / (double)nd, test);
            if (!si_->isValid(test))
            {
                lastValid.second = (double)(j - 1) / (double)nd;
                if (lastValid.first != nullptr)
                    stateSpace_->interpolate(s1, *path, lastValid.second, lastValid.first);
                result = false;
                break;
            }
        }
        si_->freeState(test);
    }

    if (result)
        if (!si_->isValid(s2))
        {
            lastValid.second = (double)(nd - 1) / (double)nd;
            if (lastValid.first != nullptr)
                stateSpace_->interpolate(s1, *path, lastValid.second, lastValid.first);
            result = false;
        }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}

bool ompl::base::VanaMotionValidator::checkMotion(const State *s1, const State *s2) const
{
    /* assume motion starts in a valid configuration so s1 is valid */
    if (!si_->isValid(s2))
        return false;
    auto path = stateSpace_->vanaPath(s1, s2);
    if (!path)
        return false;

    bool result = true;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    /* initialize the queue of test positions */
    std::queue<std::pair<int, int>> pos;
    if (nd >= 2)
    {
        pos.emplace(1, nd - 1);

        /* temporary storage for the checked state */
        State *test = si_->allocState();

        /* repeatedly subdivide the path segment in the middle (and check the middle) */
        while (!pos.empty())
        {
            std::pair<int, int> x = pos.front();

            int mid = (x.first + x.second) / 2;
            stateSpace_->interpolate(s1, *path, (double)mid / (double)nd, test);

            if (!si_->isValid(test))
            {
                result = false;
                break;
            }

            pos.pop();

            if (x.first < mid)
                pos.emplace(x.first, mid - 1);
            if (x.second > mid)
                pos.emplace(mid + 1, x.second);
        }

        si_->freeState(test);
    }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}
