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

/* Author: Dave Coleman */

#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_TRRT_
#define OMPL_GEOMETRIC_PLANNERS_RRT_TRRT_

#include "ompl/geometric/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"

/*
  TODO: change these variables back to short style to match other planners:
  nearest_neighbors_ -> nn_
  planner_termination_condition -> ptc

  NOTES: Inherited Member Variables Key
  si_ -> SpaceInformation
  pdef_ -> ProblemDefinition
  pis_ -> PlannerInputStates - Utility class to extract valid input states

*/


namespace ompl
{

namespace geometric
{

/**
   @anchor TRRT
   @par Short description //TODO
   T-RRT is a tree-based motion planner that uses the following
   idea: T-RRT samples a random state @b qr in the state space,
   then finds the state @b qc among the previously seen states
   that is closest to @b qr and expands from @b qc towards @b
   qr, until a state @b qm is reached. @b qm is then added to
   the exploration tree.
   @par External documentation
   L. Jaillet, J. Cortés, T. Siméon, Sampling-Based Path Planning on Configuration-Space Costmaps, in <em>IEEE TRANSACTIONS ON ROBOTICS, VOL. 26, NO. 4, AUGUST 2010</em>. DOI: <a href="http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=5477164&url=http%3A%2F%2Fieeexplore.ieee.org%2Fxpls%2Fabs_all.jsp%3Farnumber%3D5477164">IEEE</a><br />
   <a href="http://homepages.laas.fr/nic/Papers/10TRO.pdf">[PDF]</a>
*/

/** \brief Transition-based Rapidly-exploring Random Trees */
class TRRT : public base::Planner
{
public:

  /** \brief Constructor */
  TRRT(const base::SpaceInformationPtr &si);

  virtual ~TRRT(void);

  virtual void getPlannerData(base::PlannerData &data) const;

  virtual base::PlannerStatus solve(const base::PlannerTerminationCondition &planner_termination_condition);

  virtual void clear(void);

  /** \brief Set the goal bias

      In the process of randomly selecting states in
      the state space to attempt to go towards, the
      algorithm may in fact choose the actual goal state, if
      it knows it, with some probability. This probability
      is a real number between 0.0 and 1.0; its value should
      usually be around 0.05 and should not be too large. It
      is probably a good idea to use the default value. */
  void setGoalBias(double goalBias)
  {
    goalBias_ = goalBias;
  }

  /** \brief Get the goal bias the planner is using */
  double getGoalBias(void) const
  {
    return goalBias_;
  }

  /** \brief Set the range the planner is supposed to use.

      This parameter greatly influences the runtime of the
      algorithm. It represents the maximum length of a
      motion to be added in the tree of motions. */
  void setRange(double distance)
  {
    maxDistance_ = distance;
  }

  /** \brief Get the range the planner is using */
  double getRange(void) const
  {
    return maxDistance_;
  }

  /** \brief Set a different nearest neighbors datastructure */
  template<template<typename T> class NN>
  void setNearestNeighbors(void)
  {
    nearest_neighbors_.reset(new NN<Motion*>());
  }

  virtual void setup(void);

protected:


  /** \brief Representation of a motion

      This only contains pointers to parent motions as we
      only need to go backwards in the tree. */
  class Motion
  {
  public:

    Motion(void) : state(NULL), parent(NULL)
    {
    }

    /** \brief Constructor that allocates memory for the state */
    Motion(const base::SpaceInformationPtr &si) : state(si->allocState()), parent(NULL)
    {
    }

    ~Motion(void)
    {
    }

    /** \brief The state contained by the motion */
    base::State       *state;

    /** \brief The parent motion in the exploration tree */
    Motion            *parent;

    /** \brief The cost of the motion, cached for optimization */
    double            distance;

    /** \brief Cost of the state */
    double            cost;

  };

  /** \brief Free the memory allocated by this planner */
  void freeMemory(void);

  /** \brief Compute distance between motions (actually distance between contained states) */
  double distanceFunction(const Motion* a, const Motion* b) const
  {
    return si_->distance(a->state, b->state);
  }

  /** \brief Filter irrelevant configuration regarding the search of low-cost paths before inserting into tree */
  bool transitionTest( Motion *motion );

  /** \brief Use ratio to prefer frontier nodes to nonfrontier ones */
  bool minExpansionControl( double rand_motion_distance );

  /** \brief State sampler */
  base::StateSamplerPtr                          sampler_;

  /** \brief A nearest-neighbors datastructure containing the tree of motions */
  boost::shared_ptr< NearestNeighbors<Motion*> > nearest_neighbors_;

  /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
  double                                         goalBias_;

  /** \brief The maximum length of a motion to be added to a tree */
  double                                         maxDistance_;

  /** \brief The random number generator */
  RNG                                            rng_;

  /** \brief The most recent goal motion.  Used for PlannerData computation */
  Motion                                         *lastGoalMotion_;


  // *******************************************************************************************************************
  // Custom TRRT Member Variables
  // *******************************************************************************************************************

  // Transition Test -----------------------------------------------------

  // TODO: for my application this is the same as an obstacle, not sure how to rectify this...
  //  static const double cost_max = 1000000; // TODO: move this elsewhere and give better value

  // Max number of rejections allowed
  //  static const int MAX_NUM_FAILED = 100; // the tempered version
  static const int MAX_NUM_FAILED = 10; // the greedy version

  // Failure temperature factor used when MAX_NUM_FAILED failures occur
  static const double FAILED_FACTOR = 2; // little alpha

  // Prevent temperature from dropping too far
  static const double MIN_TEMPERATURE = 10e-10;

  // A very low value at initialization to authorize very easy positive slopes
  static const double INIT_TEMPERATURE = 10e-6;

  // Failure counter for which states are rejected
  int num_failed_;

  // Temperature parameter used to control the difficulty level of transition tests. Low temperatures
  // limit the expansion to a slightly positive slopes, high temps enable to climb the steeper slopes.
  // Dynamically tuned according to the information acquired during exploration
  double temp_;


  // Minumum Expansion Control ---------------------------------------------

  // How far is a frontier node considered to be
  static const double EXPANSION_STEP = 5; // little delta

  // Target ratio of nonfrontier nodes to frontier nodes. rho
  static const double NONFRONTIER_NODE_RATIO = 1/10; // 1 nonfrontier for every 10 frontier

  // Ratio counters for space exploration
  double nonfrontier_count_;
  double frontier_count_; // init to 1 to prevent division by zero error




};

}
}

#endif
