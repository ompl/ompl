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

/** \author Ioan Sucan */


/**
   @mainpage
   
   
   @b OMPL (Open Motion Planning Library) consists of a set of motion
   planning algorithms and nothing more. There is no environment
   specification, there is no collision detection or
   visualisation. There are only motion planners. The purpose of this
   library is to be extensible and freely available. It currently only
   contains a set of kinematic sampling-based motion planners, but
   this will be extended in the near future.

   The overall structure can be observed by looking at the @b base/
   directory. There are two main components: 
   - the space information
   - the planner

   The base space information class contains only basic definitions of
   states, paths and goals. Implementations of this space information
   class may contain additional functionality. 

   A planner can be instantiated given an instance of the space
   information. The space information contains the starting states and
   goal definition. The planner reads the neccessary data from the
   space information and fills in a path for the given goal, if it
   finds one.

   <hr> 

   The code in this library is meant to be thread safe. All static,
   non-member or const member functions are thread safe. Calling
   member functions that are not const in multiple threads
   simultaneously is unsafe and locks should be used.
   
   <hr>

   @section sampling_planners Sampling-based motion planners
   
   This class of motion planners typically needs the ability to sample
   the state (configuration) space of the robot(s) planning is done
   for. To allow this, an implementation of StateValidityChecker must
   be provided. This implementation will most likely depend on a
   collision detector.

   @subsection kinematic_planners Kinematic motion planners

   - @ref RRT
   - @ref LazyRRT
   - @ref SBL
   - @ref EST
   - @ref KPIECE
   
   @subsection kinodynamic_planners Kinodynamic motion planners
   
   - None implemented yet

   @section grid_planners Grid-based motion planners

   - Not included yet

 */

#ifndef OMPL_BASE_GENERAL_
#define OMPL_BASE_GENERAL_

/** Macro for forward class declarations */
#define ForwardClassDeclaration(A) class A; typedef A * A##_t

/** Macro for forward struct declarations */
#define ForwardStructDeclaration(A) struct A; typedef A * A##_t

#endif
