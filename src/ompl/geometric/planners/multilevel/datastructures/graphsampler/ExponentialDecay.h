#ifndef OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_DATASTRUCTURES_GRAPHSAMPLER_EXPONENTIAL_DECAY__
#define OMPL_GEOMETRIC_PLANNERS_QUOTIENTSPACE_DATASTRUCTURES_GRAPHSAMPLER_EXPONENTIAL_DECAY__

class ExponentialDecay
{
  public:
    ExponentialDecay() = default;
    ExponentialDecay(double lambda):
      lambda_(lambda){};
    ExponentialDecay(double lambda, double lowerBound):
      lambda_(lambda), lowerBound_(lowerBound){};
    ExponentialDecay(double lambda, double lowerBound, double upperBound):
      lambda_(lambda), lowerBound_(lowerBound), upperBound_(upperBound){};

    void setLowerBound(double lowerBound)
    {
        lowerBound_ = lowerBound;
    }
    void setUpperBound(double upperBound)
    {
        upperBound_ = upperBound;
    }
    void setLambda(double lambda)
    {
        lambda_ = lambda;
    }

    double operator()(void)
    {
        return (upperBound_ - lowerBound_) * exp(-lambda_ * counter_++) 
        + lowerBound_;
    }

  private:
    double lambda_{0.1};
    double lowerBound_{0.0};
    double upperBound_{1.0};
    unsigned long long counter_{0};
};
#endif

