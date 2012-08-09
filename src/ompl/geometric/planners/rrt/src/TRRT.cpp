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

#include "ompl/geometric/planners/rrt/TRRT.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/datastructures/NearestNeighborsGNAT.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <stdio.h>

// *********************************************************************************************************************
// Constructor
// *********************************************************************************************************************
ompl::geometric::TRRT::TRRT(const base::SpaceInformationPtr &si) : base::Planner(si, "TRRT")
{
  specs_.approximateSolutions = true;
  specs_.directed = true;

  goalBias_ = 0.05;
  maxDistance_ = 0.0;
  lastGoalMotion_ = NULL;

  Planner::declareParam<double>("range", this, &TRRT::setRange, &TRRT::getRange);
  Planner::declareParam<double>("goal_bias", this, &TRRT::setGoalBias, &TRRT::getGoalBias);

  // TRRT Specific Variables
  nonfrontier_count_ = 1;
  frontier_count_ = 1; // init to 1 to prevent division by zero error
}

// *********************************************************************************************************************
// Destructor
// *********************************************************************************************************************
ompl::geometric::TRRT::~TRRT(void)
{
  freeMemory();
}

// *********************************************************************************************************************
// Clear/Reset
// *********************************************************************************************************************
void ompl::geometric::TRRT::clear(void)
{
  Planner::clear();
  sampler_.reset();
  freeMemory();
  if (nearest_neighbors_)
    nearest_neighbors_->clear();
  lastGoalMotion_ = NULL;

  // Clear TRRT specific variables ---------------------------------------------------------
  num_states_failed_ = 0;
}

// *********************************************************************************************************************
// Setup
// *********************************************************************************************************************
void ompl::geometric::TRRT::setup(void)
{
  Planner::setup();
  tools::SelfConfig self_config(si_, getName());

  self_config.configurePlannerRange(maxDistance_);
  std::cout << "MAX DISTANCE: " << maxDistance_ << std::endl;

  if (!nearest_neighbors_)
    nearest_neighbors_.reset(new NearestNeighborsGNAT<Motion*>());
  nearest_neighbors_->setDistanceFunction(boost::bind(&TRRT::distanceFunction, this, _1, _2));

  // Setup TRRT specific variables ---------------------------------------------------------
  num_states_failed_ = 0;
}

// *********************************************************************************************************************
// Free Memory
// *********************************************************************************************************************
void ompl::geometric::TRRT::freeMemory(void)
{
  if (nearest_neighbors_)
  {
    std::vector<Motion*> motions;
    nearest_neighbors_->list(motions);
    for (unsigned int i = 0 ; i < motions.size() ; ++i)
    {
      if (motions[i]->state)
        si_->freeState(motions[i]->state);
      delete motions[i];
    }
  }
}

// *********************************************************************************************************************
// Solve - the main algorithm
// *********************************************************************************************************************
ompl::base::PlannerStatus
ompl::geometric::TRRT::solve(const base::PlannerTerminationCondition &planner_termination_condition)
{
  // Basic error checking
  checkValidity();

  // Goal information
  base::Goal                 *goal   = pdef_->getGoal().get();
  base::GoalSampleableRegion *goal_region = dynamic_cast<base::GoalSampleableRegion*>(goal);

  // Object for getting the cost of a state
  const base::StateValidityCheckerPtr &state_validity_checker = si_->getStateValidityChecker();


  // Input States ---------------------------------------------------------------------------------

  // Loop through valid input states and add to tree
  while (const base::State *state = pis_.nextStart())
  {
    // Allocate memory for a new start state motion based on the "space-information"-size
    Motion *motion = new Motion(si_);

    // Copy destination <= source (backwards???)
    si_->copyState(motion->state, state);

    // Set cost for this start state
    motion->cost = state_validity_checker->cost( motion->state );

    // Add start motion to the tree
    nearest_neighbors_->add(motion);
  }

  // Check that input states exist
  if (nearest_neighbors_->size() == 0)
  {
    logError("There are no valid initial states!");
    return base::PlannerStatus::INVALID_START;
  }

  // Create state sampler if this is TRRT's first run
  if (!sampler_)
    sampler_ = si_->allocStateSampler();

  // Debug
  logInform("Starting with %u states", nearest_neighbors_->size());


  // Solver variables ------------------------------------------------------------------------------------

  // the final solution
  Motion *solution  = NULL;
  // the approximate solution, returned if no final solution found
  Motion *approx_solution = NULL;
  // track the distance from goal to closest solution yet found
  double  approx_difference = std::numeric_limits<double>::infinity();

  // distance between states - the intial state and the interpolated state (may be the same)
  double rand_motion_distance;
  double motion_distance;

  // Create random motion and a pointer (for optimization) to its state
  Motion *rand_motion   = new Motion(si_);
  Motion *near_motion;

  // STATES
  // The random state
  base::State *rand_state = rand_motion->state;
  // The new state that is generated between states *to* and *from*
  base::State *interpolated_state = si_->allocState(); // Allocates "space information"-sized memory for a state
  // The chosen state btw rand_state and interpolated_state
  base::State *new_state;

  // Begin sampling --------------------------------------------------------------------------------------
  while (planner_termination_condition() == false)
  {
    //// q_rand <-- SampleConf( config_space ) ------------------------------
    {
      // Sample random state (with goal biasing probability)
      if (goal_region && rng_.uniform01() < goalBias_ && goal_region->canSample())
      {
        //std::cout << "Sample biased toward GOAL" << std::endl;
        // Bias sample towards goal
        goal_region->sampleGoal(rand_state);
      }
      else
      {
        //std::cout << "NOT BIASED" << std::endl;
        // Uniformly Sample
        sampler_->sampleUniform(rand_state);
      }
    }

    //// q_near <-- NEAREST_NEIGHBOR(q_rand, T) -----------------------------
    {
      // Find closest state in the tree
      near_motion = nearest_neighbors_->nearest( rand_motion );
    }


    //// q_new <-- Extend( T, q_rand, q_near ) ------------------------------
    {
      // Distance from near state q_n to a random state
      rand_motion_distance = si_->distance( near_motion->state, rand_state );

      // Check if the rand_state is too far away
      if( rand_motion_distance > maxDistance_ )
      {

        //std::cout << "\n\n Motion distance is greater than maxDistance = " << maxDistance_ << std::endl << std::endl;

        // Computes the state that lies at time t in [0, 1] on the segment that connects *from* state to *to* state.
        // The memory location of *state* is not required to be different from the memory of either *from* or *to*.
        //                    interpolate(const State *from, const State *to, const double t, State *state) const
        si_->getStateSpace()->interpolate( near_motion->state, rand_state,
                                           maxDistance_ / rand_motion_distance, interpolated_state);

        // Update the distance between near and new with the interpolated_state
        motion_distance = si_->distance( near_motion->state, interpolated_state );

        // Use the interpolated state as the new state
        new_state = interpolated_state;
      }
      else
      {
        //std::cout << "Random state is close enough, max = " << maxDistance_ << std::endl;

        // Random state is close enough
        new_state = rand_state;

        // Copy the distance
        motion_distance = rand_motion_distance;
      }
    }

    //// if( q_new != NULL ) -----------------------------------------------
    // this stage integrates collision detections in the presence of obstacles and checks for collisions

    /** bool checkMotion(const State *s1, const State *s2, std::pair<State*, double> &lastValid) const
        \brief Incrementally check if the path between two motions is valid. Also compute the last state that was
        valid and the time of that state. The time is used to parametrize the motion from s1 to s2, s1 being at t =
        0 and s2 being at t = 1. This function assumes s1 is valid.
        \param s1 start state of the motion to be checked (assumed to be valid)
        \param s2 final state of the motion to be checked
    */
    if( !si_->checkMotion( near_motion->state, new_state ))
      continue; // try a new sample

    //// and TransitionTest( C(q_near), C(q_new), dist_near-new ) ---------

    // Create a motion
    Motion *motion = new Motion(si_);
    si_->copyState(motion->state, new_state);
    motion->parent = near_motion; // link q_new to q_near as an edge
    motion->distance = motion_distance; // cache the distance btw parent and state
    motion->cost = state_validity_checker->cost( motion->state );

    // Only add this motion to the tree if the tranistion test accepts it
    if( !transitionTest( motion ) )
      continue; // give up on this one and try a new sample


    //// and minExpandControl( T, q_near, q_rand )

    // Minimum Expansion Control
    // A possible side effect may appear when the tree expansion toward unexplored regions remains slow, and the
    // new nodes contribute only to refine already explored regions.
    if( !minExpansionControl( rand_motion_distance ) )
      continue; // give up on this one and try a new sample


    //// AddNewEdge(T, q_near, q_new) -------------------------------------
    //// AddNewNode(T, q_new) ---------------------------------------------

    // Add motion to data structure
    nearest_neighbors_->add(motion);

    //// StopCondition(T, q_goal) ----------------------------------------------------------------

    // Check if this motion is the goal
    double dist_to_goal = 0.0;
    bool is_satisfied = goal->isSatisfied(motion->state, &dist_to_goal);
    if (is_satisfied)
    {
      approx_difference = dist_to_goal; // the tolerated error distance btw state and goal
      solution = motion; // set the final solution
      break;
    }

    // Is this the closest solution we've found so far
    if (dist_to_goal < approx_difference)
    {
      approx_difference = dist_to_goal;
      approx_solution = motion;
    }

  } // end of solver sampling loop


  // Finish solution processing --------------------------------------------------------------------

  bool solved = false;
  bool approximate = false;

  // Substitute an empty solution with the best approximation
  if (solution == NULL)
  {
    solution = approx_solution;
    approximate = true;
  }

  // Generate solution path for real/approx solution
  if (solution != NULL)
  {
    lastGoalMotion_ = solution;

    // construct the solution path
    std::vector<Motion*> mpath;
    while (solution != NULL)
    {
      mpath.push_back(solution);
      solution = solution->parent;
    }

    // set the solution path
    PathGeometric *path = new PathGeometric(si_);
    for (int i = mpath.size() - 1 ; i >= 0 ; --i)
      path->append(mpath[i]->state);
    pdef_->addSolutionPath(base::PathPtr(path), approximate, approx_difference);
    solved = true;
  }

  // Clean up ---------------------------------------------------------------------------------------

  si_->freeState(interpolated_state);
  if (rand_motion->state)
    si_->freeState(rand_motion->state);
  delete rand_motion;

  logInform("Created %u states", nearest_neighbors_->size());

  return base::PlannerStatus(solved, approximate);
}

// *********************************************************************************************************************
// Planner Data
// *********************************************************************************************************************
void ompl::geometric::TRRT::getPlannerData(base::PlannerData &data) const
{
  Planner::getPlannerData(data);

  std::vector<Motion*> motions;
  if (nearest_neighbors_)
    nearest_neighbors_->list(motions);

  if (lastGoalMotion_)
    data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

  for (unsigned int i = 0 ; i < motions.size() ; ++i)
  {
    if (motions[i]->parent == NULL)
      data.addStartVertex(base::PlannerDataVertex(motions[i]->state));
    else
      data.addEdge(base::PlannerDataVertex(motions[i]->parent->state),
                   base::PlannerDataVertex(motions[i]->state));
  }
}

// *********************************************************************************************************************
// Filter irrelevant configuration regarding the search of low-cost paths before inserting into tree
// *********************************************************************************************************************
bool ompl::geometric::TRRT::transitionTest( Motion *motion )
{
  /* KEY
     c_i = motion->parent->cost = cost of parent
     c_j = motion->cost = cost of child
  */


  // Temperature parameter used to control the difficulty level of transition tests. Low temperatures
  // limit the expansion to a slightly positive slopes, high temps enable to climb the steeper slopes.
  // Dynamically tuned according to the information acquired during exploration
  static double T = INIT_TEMPERATURE;


  // Variables --------------------------------------------------------------------------------

  double child_cost = motion->cost;
  double parent_cost = motion->parent->cost;

  // Basic Checks ------------------------------------------------------------------------------

  // Threshold check
  /*  if( child_cost > cost_max )
      {
      return false;
      }*/

  // Always accept if new state has lower cost than old state
  if( child_cost < parent_cost )
  {
    //    std::cout << "Child has lower cost than parent, accepting" << std::endl;
    return true;
  }

  // Transition Test ------------------------------------------------------------------------------

  // Difference in cost
  double cost_slope = ( child_cost - parent_cost ) / ( motion->distance ); // TODO: is distance always calculated here?

  // Constant value used to normalize expression. Based on order of magnitude of the considered costs.
  // Average cost of the query configurtaions (start?) since they are the only cost values known at the
  // beginning of the search process.
  double K = 128; // TODO: calculate dynamically

  // The probability of acceptance of a new configuration is defined by comparing its cost c_j
  // relatively to the cost c_i of its parent in the tree. Baased on the Metropolis criterion.
  double transition_probability = 1; // if cost_slope is <= 0, probabilty is 1

  // Only return at end
  bool result = false;

  // Calculate tranision probabilty
  if( cost_slope > 0 )
  {
    transition_probability = exp( -cost_slope / (K * T) );
  }

  // Check if we can accept it
  if( rng_.uniform01() <= transition_probability )
  {
    T = T / FAILED_FACTOR;

    // Prevent T from getting too small
    if( T < MIN_TEMPERATURE )
    {
      std::cout << "Temp too low --------------------------------------------------------- " << std::endl;
      T = MIN_TEMPERATURE;
    }

    num_states_failed_ = 0;

    result = true;
  }
  else
  {
    // State has failed
    if( num_states_failed_ >= MAX_NUM_FAILED )
    {
      T = T * FAILED_FACTOR;
      num_states_failed_ = 0;
    }
    else
    {
      ++num_states_failed_;
    }

  }

  // Increase temperature artifically
  static int counter = 0;
  ++counter;
  if( counter > 3000 )
  {
    T = T * 1.6;
  }


  if( true )
  {
    // Debug output
    std::cout << "Parent Cost " << std::setw(12)
              << "| Child Cost" << std::setw(12)
              << "| Diff " << std::setw(10)
              << "| Distance " << std::setw(20)
              << "| CostSlope" << std::setw(20)
              << "| Temperature " << std::setw(20)
              << "| exp( X )  " << std::setw(20)
              << "| Prob    " << std::setw(20)
              << "| Num Fail" << std::setw(20)
              << "| Result \n";
    std::cout << (result ? "" : "\033[0;31m") << std::setw(12)
              << parent_cost << std::setw(12)
              << child_cost << std::setw(10)
              << child_cost-parent_cost << std::setw(20)
              << motion->distance << std::setw(20)
              << cost_slope << std::setw(20)
              << T << std::setw(20)
              << -cost_slope / (K * T) << std::setw(20)
              << transition_probability << std::setw(20)
              << double(num_states_failed_) << std::setw(20)
              << (result ? "ACCEPT" : "REJECT\033[0m")
              << "   " << counter
              << std::endl;
  }

  return result;
}

// *********************************************************************************************************************
// Use ratio to prefer frontier nodes to nonfrontier ones
// *********************************************************************************************************************
bool ompl::geometric::TRRT::minExpansionControl( double rand_motion_distance )
{
  // Decide to accept or not
  if( rand_motion_distance > EXPANSION_STEP )
  {
    // participates in the tree expansion
    ++frontier_count_;
    std::cout << "MIN_EXPAND_CONTROL: accepted bc larger than step" << std::endl;

    return true;
  }
  else
  {
    // participates in the tree refinement

    // check our ratio first before accepting it
    if( nonfrontier_count_ / frontier_count_ > NONFRONTIER_NODE_RATIO )
    {
      std::cout << "MIN_EXPAND_CONTROL: \033[0;31mREJECTED\033[0m bc bad ratio" << std::endl;

      // Increment so that the temperature rises faster
      ++num_states_failed_;

      // reject this node as being too much refinement
      return false;
    }
    else
    {
      std::cout << "MIN_EXPAND_CONTROL: accepted as within ratio" << std::endl;

      ++nonfrontier_count_;
      return true;
    }
  }
}


