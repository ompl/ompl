#ifndef OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_DATASTRUCTURES_PARAMETER_
#define OMPL_MULTILEVEL_PLANNERS_BUNDLESPACE_DATASTRUCTURES_PARAMETER_

#include <ompl/util/Exception.h>

namespace ompl
{
    class Parameter
    {
    public:
        Parameter() = default;

        Parameter(double valueInit) : 
          valueInit_(valueInit)
        {
        };

        Parameter(double valueInit, double valueTarget) : 
          valueInit_(valueInit), 
          valueTarget_(valueTarget)
        {
        };

        void setValueInit(double valueInit)
        {
            valueInit_ = valueInit;
        }
        void setValueTarget(double valueTarget)
        {
            valueTarget_ = valueTarget;
        }
        void setCounterTarget(unsigned long long counterTarget)
        {
            counterTarget_ = counterTarget;
        }
        void setCounterInit(unsigned long long counterInit)
        {
            counterInit_ = counterInit;
        }

        double getValueInit()
        {
            return valueInit_;
        }
        double getValueTarget()
        {
            return valueTarget_;
        }
        unsigned long long getCounterTarget()
        {
            return counterTarget_;
        }
        unsigned long long getCounterInit()
        {
            return counterInit_;
        }

        double operator()(void)
        {
            if(counter_ > counterTarget_) return valueTarget_;

            //map to [0,1]
            double t = (counter_ - counterInit_)/(double)(counterTarget_ - counterInit_);

            double d = valueInit_ + t*(valueTarget_ - valueInit_);
            counter_++;
            return d;
        }

        void reset()
        {
            counter_ = 0;
        }

        unsigned long long getCounter()
        {
            return counter_;
        }
        void incrementCounter()
        {
            counter_++;
        }

    private:
        double valueInit_{0.0};
        double valueTarget_{1.0};

        unsigned long long counter_{0};

        unsigned long long counterInit_{0};
        unsigned long long counterTarget_{100};
    };
}
#endif

