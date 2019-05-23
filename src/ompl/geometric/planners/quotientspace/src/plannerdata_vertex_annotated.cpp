#include "plannerdata_vertex_annotated.h"

using FeasibilityType = PlannerDataVertexAnnotated::FeasibilityType;

PlannerDataVertexAnnotated::PlannerDataVertexAnnotated(const ob::State *st, int tag):
      ob::PlannerDataVertex(st,tag)
{
}

PlannerDataVertexAnnotated::PlannerDataVertexAnnotated (const PlannerDataVertexAnnotated &rhs): 
  ob::PlannerDataVertex(rhs.state_, rhs.tag_)
{
  level = rhs.GetLevel();
  max_level = rhs.GetMaxLevel();
  component = rhs.GetComponent();
  state_quotient_space = rhs.getQuotientState();
}

ob::PlannerDataVertex *PlannerDataVertexAnnotated::clone() const 
{
  return new PlannerDataVertexAnnotated(*this);
}

//##############################################################################
void PlannerDataVertexAnnotated::SetComponent(uint component_)
{
  component = component_;
}
uint PlannerDataVertexAnnotated::GetComponent() const
{
  return component;
}

//##############################################################################
void PlannerDataVertexAnnotated::SetLevel(uint level_)
{
  level = level_;
}
uint PlannerDataVertexAnnotated::GetLevel() const
{
  return level;
}

//##############################################################################
void PlannerDataVertexAnnotated::SetMaxLevel(uint level_)
{
  max_level = level_;
}
uint PlannerDataVertexAnnotated::GetMaxLevel() const
{
  return max_level;
}

//##############################################################################
const ob::State *PlannerDataVertexAnnotated::getState() const 
{
  return state_;
}
const ob::State *PlannerDataVertexAnnotated::getQuotientState() const
{
  return state_quotient_space;
}
void PlannerDataVertexAnnotated::setQuotientState(const ob::State *s)
{
  state_quotient_space = s;
}
void PlannerDataVertexAnnotated::setState(ob::State *s)
{
  state_ = s;
}

std::ostream& operator<< (std::ostream& out, const PlannerDataVertexAnnotated& v)
{
  out << "AnnotatedVertex";
  out << " ->level " << v.GetLevel() << "/" << v.GetMaxLevel();
  out << " ->component " << v.GetComponent();
  out << std::endl;
  return out;
}
