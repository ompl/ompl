/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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

/* Author: Nikhil P, Vrushabh Zinage */

#include "ompl/geometric/planners/GSE/GSE.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/tools/config/SelfConfig.h"

ompl::geometric::GSE::GSE(const base::SpaceInformationPtr &si, uint dim)
    : base::Planner(si, "GSE"), resolution_(0), dim_(dim)
{
    specs_.approximateSolutions = true;
}

ompl::geometric::GSE::~GSE()
{
}

ompl::base::PlannerStatus ompl::geometric::GSE::solve(const base::PlannerTerminationCondition &ptc)
{
    // make sure the planner is configured correctly; ompl::base::Planner::checkValidity
    // ensures that there is at least one input state and a ompl::base::Goal object specified
    checkValidity();
    base::State *rState = si_->allocState();
    Eigen::VectorXd xState(dim_);
    Eigen::VectorXd xN(dim_);
    uint idx;
    std::vector<int> solution;

    // get a handle to the Goal from the ompl::base::ProblemDefinition member, pdef_
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalState *>(goal);

    if (vertices_.size() == 0)
    {
        projector_->project(goal_s->getState(), xState);
        addVertex(xState);
    }

    // get input states with PlannerInputStates helper, pis_
    while (pis_.haveMoreStartStates())
    {
        projector_->project(pis_.nextStart(), xState);
        addVertex(xState);
    }

    if (vertices_.size() <= 1)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(),
                vertices_.size());

    while (!ptc())
    {
        sampler_->sampleUniform(rState);
        projector_->project(rState, xState);
        idx = nn_->nearest(std::make_pair(xState, -1)).second;
        vertices_[idx]->steer(xState);
        idx = addVertex(xState);

        for (uint i = 0; i < idx; i++)
        {
            xN = vertices_[i]->eigenState_;
            vertices_[idx]->steer(xN);
            if (vertices_[i]->shape(xN))
            {
                xN = vertices_[idx]->eigenState_;
                vertices_[i]->steer(xN);
                if (vertices_[idx]->shape(xN))
                {
                    double dist = (vertices_[idx]->eigenState_ - vertices_[i]->eigenState_).norm();
                    adj_.addEdge(i, idx, dist);
                    adj_.addEdge(idx, i, dist);
                }
            }
        }

        if (adj_.inSameComponent(0, 1))
        {
            adj_.dijkstra(1, 0, solution);
            break;
        }
    }

    si_->freeState(rState);
    OMPL_INFORM("%s: Created %u states", getName().c_str(), vertices_.size());

    if (solution.size() != 0)
    {
        auto path(std::make_shared<PathGeometric>(si_));
        for (auto x : solution)
        {
            path->append(vertices_[x]->state_);
        }
        pdef_->addSolutionPath(path, true, 0, getName());
        return base::PlannerStatus::EXACT_SOLUTION;
    }
    return base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::GSE::clear(void)
{
    Planner::clear();
    // clear the data structures here
    sampler_.reset();
    if (nn_)
        nn_->clear();
    vertices_.clear();
    adj_.clear();
}

void ompl::geometric::GSE::setup(void)
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());

    if (projector_->getDimension() != dim_)
    {
        OMPL_ERROR("%s: Projector dimension do not match planner dimension: expected dimension %d but got "
                    "%d",
                    getName().c_str(), dim_, projector_->getDimension());
        OMPL_INFORM("The GSE Algorithm works on the euclidean vector space (R^n). To use GSE with OMPL, "
                    "the statespace should have a default projector which maps to R^dim");
    }

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<std::pair<Eigen::VectorXd, int>>(this));
    nn_->setDistanceFunction(
        [this](const std::pair<Eigen::VectorXd, int> &a, const std::pair<Eigen::VectorXd, int> &b) {
            return distanceFcn(a, b);
        });
}

void ompl::geometric::GSE::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    data.addGoalVertex(base::PlannerDataVertex(vertices_[0]->state_));
    data.addStartVertex(base::PlannerDataVertex(vertices_[1]->state_));

    for (uint i = 0; i < vertices_.size(); i++)
    {
        std::vector<int> nbrs;
        adj_.getNeighbors(i, nbrs);
        for (auto j : nbrs)
            data.addEdge(base::PlannerDataVertex(vertices_[i]->state_),
                            base::PlannerDataVertex(vertices_[j]->state_));
    }
}

void ompl::geometric::GSE::addObstacle(Eigen::MatrixXd &obs)
{
    if (obs.rows() != dim_)
    {
        OMPL_ERROR("%s: Obstacle Dimension is not equal to planner dimension, Aborting Obstacle Addition",
                    getName().c_str());
        return;
    }
    obstacles_.push_back(obs);
}

void ompl::geometric::GSE::setProjector(base::ProjectionEvaluatorPtr projector)
{
    projector_ = projector;
}

void ompl::geometric::GSE::setResolution(double resolution){
    resolution_=resolution;
}

uint ompl::geometric::GSE::addVertex(const Eigen::VectorXd state)
{
    vertices_.push_back(std::move(std::make_unique<Node>(state, obstacles_, si_,resolution_)));
    uint idx = adj_.addVertex();
    nn_->add(std::make_pair(state, idx));
    return idx;
}

ompl::geometric::GSE::Node::Node(const Eigen::VectorXd &state, const std::vector<Eigen::MatrixXd> &obstacles,
        base::SpaceInformationPtr &si, double resolution)
    : si_(si), eigenState_(state), obsSize(obstacles.size())
{
    state_ = si_->allocState();
    std::vector<double> vecState(eigenState_.size());
    Eigen::VectorXd::Map(&vecState[0], eigenState_.size()) = eigenState_;
    if (si_->getStateSpace()->isCompound())
    {
        auto *compund_state = state_->as<ompl::base::CompoundStateSpace::StateType>();
        auto *realvector_state = compund_state->as<ompl::base::RealVectorStateSpace::StateType>(0);
        si_->getStateSpace()->as<ompl::base::CompoundStateSpace>()->getSubspace(0)->copyFromReals(
            realvector_state, vecState);
        si_->getStateSpace()->enforceBounds(state_);
    }
    else
        si_->getStateSpace()->copyFromReals(state_, vecState);

    Eigen::MatrixXd::Index minIndex;
    Eigen::MatrixXd obsRel;
    Eigen::VectorXd normal;
    double rmin, theta;
    for (auto &obs : obstacles)
    {
        obsRel = obs.colwise() - state;
        rmin = std::max(sqrt(obsRel.colwise().squaredNorm().minCoeff(&minIndex)) - resolution, 0.0);
        obsRel.colwise().normalize();
        normal = obsRel.col(minIndex);
        obsRel = normal.transpose() * obsRel;
        theta = acos((obsRel.array() > 0).select(obsRel, 10).minCoeff());
        shapeParam_.insert(std::move(std::make_unique<obsParam>(rmin, normal, theta)));
    }
}

bool ompl::geometric::GSE::Node::shape(Eigen::VectorXd &state)
{
    bool flag = false;
    std::vector<int> r;
    r.reserve(obsSize);
    std::vector<int> f;
    f.reserve(obsSize);
    Eigen::VectorXd rel = state - eigenState_;

    auto sat = [](double x) -> int { return (x > 0) ? 1 : 0; };
    for (auto &x : shapeParam_)
    {
        r.push_back(sat(acos(rel.transpose().normalized() * x->normal_) - x->theta_));
        if (r.back() == 0)
            flag = true;
        f.push_back(sat(x->rmin_ - rel.norm()));
    }

    if (flag)
    {
        int sum = r[0] + 1;
        if (sum == f[0])
            return 1;
        for (uint i = 1; i < obsSize; i++)
        {
            sum += r[i] - 2 * r[i - 1] + 1;
            if (sum == f[i])
                return 1;
        }
        return 0;
    }
    else
        return 1;
}

void ompl::geometric::GSE::Node::steer(Eigen::VectorXd &state)
{
    if (shape(state))
        return;
    Eigen::VectorXd slope = (state - eigenState_).normalized();
    for (auto it = shapeParam_.rbegin(); it != shapeParam_.rend(); it++)
    {
        state = eigenState_ + slope * (it->get()->rmin_);
        if (shape(state))
            return;
    }
}

ompl::geometric::GSE::Node::~Node()
{
    si_->freeState(state_);
}