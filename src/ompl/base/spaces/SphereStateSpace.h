#ifndef OMPL_BASE_SPACES_SPHERE_STATE_SPACE_
#define OMPL_BASE_SPACES_SPHERE_STATE_SPACE_

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/SO2StateSpace.h"

namespace ompl
{
    namespace base
    {
        class SphereStateSampler : public ompl::base::StateSampler
        {
        public:
            SphereStateSampler(const StateSpace *space);

            void sampleUniform(State *state) override;

            void sampleUniformNear(
                State *state, 
                const State *near, 
                double distance) override;

            void sampleGaussian(
                State *state, 
                const State *mean, 
                double stdDev) override;

        };

        class SphereStateSpace : public CompoundStateSpace
        {
        public:
            class StateType : public CompoundStateSpace::StateType
            {
            public:
                StateType() = default;

                double getTheta() const
                {
                    return as<SO2StateSpace::StateType>(0)->value;
                }
                double getPhi() const
                {
                    return as<SO2StateSpace::StateType>(1)->value;
                }

                void setTheta(double theta)
                {
                    as<SO2StateSpace::StateType>(0)->value = theta;
                }
                void setPhi(double phi)
                {
                    as<SO2StateSpace::StateType>(1)->value = phi;
                }
                void setThetaPhi(double theta, double phi)
                {
                    setTheta(theta);
                    setPhi(phi);
                }
            };

            SphereStateSpace();

            virtual ~SphereStateSpace() override = default;

            StateSamplerPtr allocDefaultStateSampler() const override;

            double getMeasure() const override;

            double distance(const State *state1, const State *state2) const override;

            State *allocState() const override;

            void freeState(State *state) const override;
        protected:
            double radius_{1.0};
        };
    }
}

#endif


