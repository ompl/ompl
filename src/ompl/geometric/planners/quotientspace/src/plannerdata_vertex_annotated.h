#pragma once
#include <ompl/base/PlannerData.h>
#include <boost/serialization/export.hpp>
namespace ob = ompl::base;

class PlannerDataVertexAnnotated: public ob::PlannerDataVertex
{
  //If new elements are added, you need to update the clone/getstate functions!
  public:
    enum class FeasibilityType{FEASIBLE, INFEASIBLE, SUFFICIENT_FEASIBLE};

    PlannerDataVertexAnnotated(const ob::State *st, int tag=0);
    PlannerDataVertexAnnotated (const PlannerDataVertexAnnotated &rhs);
    virtual PlannerDataVertex *clone() const override;

    void SetOpenNeighborhoodDistance(double d_);
    double GetOpenNeighborhoodDistance() const;

    void SetLevel(uint level_);
    uint GetLevel() const;

    void SetMaxLevel(uint level_);
    uint GetMaxLevel() const;

    void SetComponent(uint component_);
    uint GetComponent() const;

    void setState(ob::State *s);
    void setQuotientState(const ob::State *s);
    virtual const ob::State *getState() const override;
    virtual const ob::State *getQuotientState() const;

    virtual bool operator==(const PlannerDataVertex &rhs) const override
    {
      const PlannerDataVertexAnnotated &v = static_cast<const PlannerDataVertexAnnotated&>(rhs);
      return (level == v.GetLevel() &&
              state_ == v.getState());
    }

    friend std::ostream& operator<< (std::ostream&, const PlannerDataVertexAnnotated&);

  protected:

    bool infeasible{false};
    uint level{0};
    uint max_level{1};

    uint component{0};
    const ob::State *state_quotient_space{nullptr};

};

//BOOST_CLASS_EXPORT(PlannerDataVertexAnnotated);
