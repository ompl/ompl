/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2019, University of Stuttgart
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
*   * Neither the name of the University of Stuttgart nor the names
*     of its contributors may be used to endorse or promote products
*     derived from this software without specific prior written
*     permission.
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

/* Author: Andreas Orthey */
#include "PlannerdataVertexAnnotated.h"

using FeasibilityType = PlannerDataVertexAnnotated::FeasibilityType;

PlannerDataVertexAnnotated::PlannerDataVertexAnnotated(const ob::State *st,
                                                       int tag)
    : ob::PlannerDataVertex(st, tag) {}

PlannerDataVertexAnnotated::PlannerDataVertexAnnotated(
    const PlannerDataVertexAnnotated &rhs)
    : ob::PlannerDataVertex(rhs.state_, rhs.tag_) {
  level = rhs.GetLevel();
  max_level = rhs.GetMaxLevel();
  component = rhs.GetComponent();
  state_quotient_space = rhs.getQuotientState();
}

ob::PlannerDataVertex *PlannerDataVertexAnnotated::clone() const {
  return new PlannerDataVertexAnnotated(*this);
}

//##############################################################################
void PlannerDataVertexAnnotated::SetComponent(uint component_) {
  component = component_;
}
uint PlannerDataVertexAnnotated::GetComponent() const { return component; }

//##############################################################################
void PlannerDataVertexAnnotated::SetLevel(uint level_) { level = level_; }
uint PlannerDataVertexAnnotated::GetLevel() const { return level; }

//##############################################################################
void PlannerDataVertexAnnotated::SetMaxLevel(uint level_) {
  max_level = level_;
}
uint PlannerDataVertexAnnotated::GetMaxLevel() const { return max_level; }

//##############################################################################
const ob::State *PlannerDataVertexAnnotated::getState() const { return state_; }
const ob::State *PlannerDataVertexAnnotated::getQuotientState() const {
  return state_quotient_space;
}
void PlannerDataVertexAnnotated::setQuotientState(const ob::State *s) {
  state_quotient_space = s;
}
void PlannerDataVertexAnnotated::setState(ob::State *s) { state_ = s; }

std::ostream &operator<<(std::ostream &out,
                         const PlannerDataVertexAnnotated &v) {
  out << "AnnotatedVertex";
  out << " ->level " << v.GetLevel() << "/" << v.GetMaxLevel();
  out << " ->component " << v.GetComponent();
  out << std::endl;
  return out;
}
