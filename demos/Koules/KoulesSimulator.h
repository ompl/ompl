/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2013, Rice University
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

/* Author: Beck Chen, Mark Moll */

#ifndef DEMOS_KOULES_SIMULATOR_
#define DEMOS_KOULES_SIMULATOR_

#include "KoulesConfig.h"
#include <ompl/control/StatePropagator.h>
#include <tuple>
#include <queue>

// State propagator for KoulesSetup.
class KoulesSimulator
{
public:
    KoulesSimulator(const ompl::control::SpaceInformation* si);

    // A propagate step.
    void step(const ompl::base::State *start, const ompl::control::Control* control,
        const double t, ompl::base::State *result);

protected:
    // A tuple containing the time and id's of two objects colliding
    using CollisionEvent = std::tuple<double, unsigned int, unsigned int>;
    // A priority queue of events, s.t. the top element is the collision
    // that will happen first.
    using CollisionEventQueue = std::priority_queue<CollisionEvent,
        std::vector<CollisionEvent>, std::greater<CollisionEvent>>;

    // Compute the collision events based on current positions and velocities.
    // Push objects apart if they are slightly overlapping.
    void initCollisionEvents();
    // Return time when i will return with horizontal (dim==0) or vertical
    // (dim==1) walls.
    double wallCollideEvent(unsigned int i, int dim);
    // Compute the collision response velocities when i and j collide.
    void elasticCollision(unsigned int i, unsigned int j);
    // Compute time if/when i and j will collide. If it happens before
    // endTime_, insert a collision event in the queue.
    void computeCollisionEvent(unsigned int i, unsigned int j);
    // Advance to the system to time t assuming no collision happen
    // between time_ and t.
    void advance(double t);
    // Mark object i as dead. The koules have id's 1,..,numKoules_, while
    // the ship has id 0.
    void markAsDead(unsigned int i);
    // Analytic solution for ship's motion from time 0 to t.
    void updateShip(const ompl::control::Control* control, double t);

    // Pointer to Koules' SpaceInformation.
    const ompl::control::SpaceInformation* si_;
    // Number of dimensions in state space.
    unsigned int numDimensions_;
    // Number of koules.
    unsigned int numKoules_;
    // Scratch space holding the current state.
    std::vector<double> qcur_;
    // Scrath space holding the next state after integration.
    std::vector<double> qnext_;
    // A vector of flags indicating which objects are dead.
    std::vector<bool> dead_;
    // The current time in the simulation.
    double time_;
    // The time to stop the simulation.
    double endTime_;
    // A queue of collision events.
    CollisionEventQueue collisionEvents_;
};

#endif
