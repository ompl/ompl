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
   @par Short description
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
    max_distance_ = distance;
  }

  /** \brief Get the range the planner is using */
  double getRange(void) const
  {
    return max_distance_;
  }

  /** \brief Set the maximum number of states that can be rejected before the temperature starts to rise */
  void setMaxStatesFailed( double max_states_failed )
  {
    max_states_failed_ = max_states_failed;
  }

  /** \brief Get the maximum number of states that can be rejected before the temperature starts to rise */
  double getMaxStatesFailed( void ) const
  {
    return max_states_failed_;
  }

  /** \brief Set the factor by which the temperature rises or falls based on current acceptance/rejection rate */
  void setTempChangeFactor( double temp_change_factor )
  {
    temp_change_factor_ = temp_change_factor;
  }

  /** \brief Get the factor by which the temperature rises or falls based on current acceptance/rejection rate */
  double getTempChangeFactor( void ) const
  {
    return temp_change_factor_;
  }

  /** \brief Set the minimum the temperature can drop to before being floored at that value */
  void setMinTemperature( double min_temperature )
  {
    min_temperature_ = min_temperature;
  }

  /** \brief Get the minimum the temperature can drop to before being floored at that value */
  double getMinTemperature( void ) const
  {
    return min_temperature_;
  }

  /** \brief Set the initial temperature at the beginning of the algorithm. Should be low */
  void setInitTemperature( double init_temperature )
  {
    init_temperature_ = init_temperature;
  }

  /** \brief Get the initial temperature at the beginning of the algorithm. Should be low */
  double getInitTemperature( void ) const
  {
    return init_temperature_;
  }

  /** \brief Set the distance between a new state and the nearest neighbor
      that qualifies that state as being a frontier */
  void setFrontierThreshold( double frontier_threshold )
  {
    frontier_threshold_ = frontier_threshold;
  }

  /** \brief Get the distance between a new state and the nearest neighbor
      that qualifies that state as being a frontier */
  double getFrontierThreshold( void ) const
  {
    return frontier_threshold_;
  }

  /** \brief Set the ratio between adding nonfrontier nodes to frontier nodes,
      for example .1 is 1/10 or one nonfrontier node for every 10 frontier nodes added */
  void setFrontierNodeRatio( double frontier_node_ratio )
  {
    frontier_node_ratio_ = frontier_node_ratio;
  }

  /** \brief Get the ratio between adding nonfrontier nodes to frontier nodes,
      for example .1 is 1/10 or one nonfrontier node for every 10 frontier nodes added */
  double getFrontierNodeRatio( void ) const
  {
    return frontier_node_ratio_;
  }

  /** \brief Set the constant value used to normalize the expression */
  void setKConstant( double k_constant )
  {
    k_constant_ = k_constant;
  }

  /** \brief Get the constant value used to normalize the expression */
  double getKConstant( void ) const
  {
    return k_constant_;
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

    /** \brief The distance between parent state and this state, cached for optimization */
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

  /** \brief Filter irrelevant configuration regarding the search of low-cost paths before inserting into tree
      \param child_cost - cost of current state
      \param parent_cost - cost of its ancestor parent state
      \param distance - distance between parent and child
  */
  bool transitionTest( double child_cost, double parent_cost, double distance );

  /** \brief Use ratio to prefer frontier nodes to nonfrontier ones */
  bool minExpansionControl( double rand_motion_distance );

  /** \brief State sampler */
  base::StateSamplerPtr                          sampler_;

  /** \brief A nearest-neighbors datastructure containing the tree of motions */
  boost::shared_ptr< NearestNeighbors<Motion*> > nearest_neighbors_;

  /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is available) */
  double                                         goalBias_;

  /** \brief The maximum length of a motion to be added to a tree */
  double                                         max_distance_;

  /** \brief The random number generator */
  RNG                                            rng_;

  /** \brief The most recent goal motion.  Used for PlannerData computation */
  Motion                                         *lastGoalMotion_;

  /** \brief Output debug info */
  bool                                                                             verbose_;

  // *********************************************************************************************************
  // TRRT-Specific Variables
  // *********************************************************************************************************

  // Transtion Test -----------------------------------------------------------------------

  // Temperature parameter used to control the difficulty level of transition tests. Low temperatures
  // limit the expansion to a slightly positive slopes, high temps enable to climb the steeper slopes.
  // Dynamically tuned according to the information acquired during exploration
  double                                          temp_;

  // Constant value used to normalize expression. Based on order of magnitude of the considered costs.
  // Average cost of the query configurtaions since they are the only cost values known at the
  // beginning of the search process.
  double                                                                                        k_constant_;

  // Max number of rejections allowed
  unsigned int                                    max_states_failed_;

  // Failure temperature factor used when max_num_failed_ failures occur
  double                                          temp_change_factor_;

  // Prevent temperature from dropping too far
  double                                          min_temperature_;

  // A very low value at initialization to authorize very easy positive slopes
  double                                          init_temperature_;

  // Failure counter for states that are rejected
  unsigned int                                    num_states_failed_;


  // Minimum Expansion Control --------------------------------------------------------------

  // Ratio counters for nodes that expand the search space versus those that do not
  double                                          nonfrontier_count_;
  double                                          frontier_count_;

  // The distance between an old state and a new state that qualifies it as a frontier state
  double                                          frontier_threshold_;

  // Target ratio of nonfrontier nodes to frontier nodes. rho
  double                                          frontier_node_ratio_;



};

}
}

#endif
