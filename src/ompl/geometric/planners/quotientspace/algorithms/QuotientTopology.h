#pragma once
#include <ompl/geometric/planners/quotientspace/datastructures/QuotientSpaceGraphSparse.h>
#include <ompl/datastructures/PDF.h>
#include <ompl/control/Control.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/control/DirectedControlSampler.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
//namespace oc = ompl::control;
//need to set propagationStepSize?

namespace ompl
{
  namespace base
  {
      OMPL_CLASS_FORWARD(OptimizationObjective);
  }
  namespace geometric
  {
    //QuotientTopology 
    class QuotientTopology: public og::QuotientSpaceGraphSparse{

      typedef og::QuotientSpaceGraphSparse BaseT;
      public:

        QuotientTopology(const ob::SpaceInformationPtr &si, QuotientSpace *parent_);
        virtual ~QuotientTopology() override;
        virtual void grow() override;
        virtual void growGeometric();
        virtual void growControl();
        virtual bool getSolution(ob::PathPtr &solution) override;

        virtual void setup() override;
        virtual void clear() override;

        void setGoalBias(double goalBias);
        double getGoalBias() const;
        void setRange(double distance);
        double getRange() const;

        Configuration *q_random{nullptr};
        ompl::control::Control* c_random{nullptr};
        ob::State* s_random;
        ompl::control::StatePropagatorPtr prop;
        ompl::control::DirectedControlSamplerPtr dCSampler;
	

      protected:

        int numberOfControlSamples{10};
        double propStepSize;
        int controlDuration{200};
        double maxDistance{.0};
        double goalBias{.05};
        double epsilon{.0};

    };

  };
};
