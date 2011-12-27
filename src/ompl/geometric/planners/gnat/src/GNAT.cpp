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

/* Author: Ioan Sucan */

#include "ompl/geometric/planners/gnat/GNAT.h"
#include "ompl/base/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <cassert>

ompl::geometric::GNAT::GNAT(const base::SpaceInformationPtr &si,
		bool useProjectedDistance,
		unsigned int degree, unsigned int minDegree,
		unsigned int maxDegree, unsigned int maxNumPtsPerLeaf,
		unsigned int removedCacheSize
    ) : base::Planner(si, "GNAT")
{
	_nng = boost::shared_ptr<gnatSampler<ompl::geometric::GNAT::compactState> >(new gnatSampler<ompl::geometric::GNAT::compactState>(degree,minDegree,maxDegree,maxNumPtsPerLeaf,removedCacheSize));
	if(useProjectedDistance)
		_nng->setDistanceFunction(defaultProjectionDistanceFunction);
	else
		_nng->setDistanceFunction(defaultDistanceFunction);
	goalBias_ = 0.05;
	specs_.approximateSolutions = true;
	maxDistance_ = 0.0;
  _borderFraction = 0.0;

  Planner::declareParam<double>("range", this, &GNAT::setRange, &GNAT::getRange);
  Planner::declareParam<double>("goal_bias", this, &GNAT::setGoalBias, &GNAT::getGoalBias);
}

ompl::geometric::GNAT::~GNAT(void)
{
    freeMemory();
}

void ompl::geometric::GNAT::setRebuildRadius(double radius)
{
  _nng->setRebuildRadius(radius);
}

void ompl::geometric::GNAT::setup(void)
{
    Planner::setup();
    SelfConfig sc(si_, getName());
    sc.configureProjectionEvaluator(projectionEvaluator_);
    sc.configurePlannerRange(maxDistance_);
}

void ompl::geometric::GNAT::clear(void)
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    _nng->clear();
}

void ompl::geometric::GNAT::freeMemory(void)
{
  _nng->clear();
}

bool ompl::geometric::GNAT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal                   *goal = pdef_->getGoal().get();
    base::GoalSampleableRegion *goal_s = dynamic_cast<base::GoalSampleableRegion*>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        addMotion(motion);
    }

    flushMotions();
    if (_nng->size() == 0)
    {
      msg_.error("There are no valid initial states!");
      return false;
    }

    if (!sampler_)
      sampler_ = si_->allocValidStateSampler();

    msg_.inform("Starting with %u states", _nng->size());

    Motion *solution  = NULL;
    Motion *approxsol = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();
    base::State *xstate = si_->allocState();

    while (ptc() == false)
    {
      /* Decide on a state to expand from */
      Motion *existing = selectMotion();
      assert(existing);

      /* sample random state (with goal biasing) */
      if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
        goal_s->sampleGoal(xstate);
      else
        if (!sampler_->sampleNear(xstate, existing->state, maxDistance_))
          continue;

      if (si_->checkMotion(existing->state, xstate))
      {
        /* create a motion */
        Motion *motion = new Motion(si_);
        si_->copyState(motion->state, xstate);
        motion->parent = existing;

        addMotion(motion);
        double dist = 0.0;
        bool solved = goal->isSatisfied(motion->state, &dist);
        if (solved)
        {
          approxdif = dist;
          solution = motion;
          break;
        }
        if (dist < approxdif)
        {
          approxdif = dist;
          approxsol = motion;
        }
      }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == NULL)
    {
      solution = approxsol;
      approximate = true;
    }

    if (solution != NULL)
    {
      /* construct the solution path */
      std::vector<Motion*> mpath;
      while (solution != NULL)
      {
        mpath.push_back(solution);
        solution = solution->parent;
      }

      /* set the solution path */
      PathGeometric *path = new PathGeometric(si_);
      for (int i = mpath.size() - 1 ; i >= 0 ; --i)
        path->states.push_back(si_->cloneState(mpath[i]->state));
      goal->addSolutionPath(base::PathPtr(path), approximate, approxdif);
      solved = true;
    }

    si_->freeState(xstate);

    msg_.inform("Created %u states", _nng->size());

    return solved;
}

void ompl::geometric::GNAT::addMotion(Motion *motion)
{
  //std::cout<<"Adding state "<<_insertionQueue.size()<<std::endl;
  _insertionQueue.push_back(ompl::geometric::GNAT::compactState(motion,si_,projectionEvaluator_));
  //_nng->add(ompl::geometric::GNAT::compactState(motion,si_,projectionEvaluator_));
}

void ompl::geometric::GNAT::flushMotions() const
{
  if(_insertionQueue.size())
  {
    //std::cout<<"Flushing states"<<std::endl;
    if(_insertionQueue.size() == 1)
      _nng->add(_insertionQueue.front());
    else
      _nng->add(_insertionQueue); 
    _insertionQueue.clear();
  }
}

ompl::geometric::GNAT::Motion* ompl::geometric::GNAT::selectMotion(void)
{
  flushMotions();
  return _nng->sample(rng_.uniform01() < _borderFraction).getMotion();
}
void ompl::geometric::GNAT::getPlannerData(base::PlannerData &data) const
{
  flushMotions();
  Planner::getPlannerData(data);

  std::vector<compactState> motions;
  _nng->list(motions);
  for (std::vector<compactState>::iterator it=motions.begin(); it!=motions.end(); it++)
    data.recordEdge(it->getMotion()->parent ? it->getMotion()->parent->state : NULL, it->getMotion()->state);
}

double ompl::geometric::GNAT::defaultDistanceFunction(const ompl::geometric::GNAT::compactState &A, const ompl::geometric::GNAT::compactState &B)
{
  return A.distance(B);
}

double ompl::geometric::GNAT::defaultProjectionDistanceFunction(const ompl::geometric::GNAT::compactState &A, const ompl::geometric::GNAT::compactState &B)
{
  base::ProjectionEvaluatorPtr p = A.getProjEv();
  size_t N = p->getDimension();
  ompl::base::EuclideanProjection eA(N), eB(N);
  p->project(A.getMotion()->state,eA); p->project(B.getMotion()->state,eB);
  double d = 0.0;
  for(size_t k=0; k<eA.size(); k++)
  {
    d+= (eA[k] - eB[k])*(eA[k] - eB[k]);
  }
  return sqrt(d);
}
