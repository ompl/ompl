#ifndef OMPL_BASE_SPACES_TORUS_STATE_SPACE_
#define OMPL_BASE_SPACES_TORUS_STATE_SPACE_

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

namespace ompl
{
    namespace base
    {
        class TorusStateSampler : public StateSampler
        {
        public:
            TorusStateSampler(const StateSpace *space);

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

        class TorusStateSpace : public CompoundStateSpace
        {
        public:
            class StateType : public CompoundStateSpace::StateType
            {
            public:
                StateType() = default;

                double getS1() const
                {
                    return as<SO2StateSpace::StateType>(0)->value;
                }
                double getS2() const
                {
                    return as<SO2StateSpace::StateType>(1)->value;
                }

                void setS1(double s)
                {
                    as<SO2StateSpace::StateType>(0)->value = s;
                }
                void setS2(double s)
                {
                    as<SO2StateSpace::StateType>(1)->value = s;
                }
                void setS1S2(double s, double t)
                {
                    setS1(s);
                    setS2(t);
                }
            };

            TorusStateSpace(double majorRadius = 1, double minorRadius = 0.5);

            virtual ~TorusStateSpace() override = default;

            StateSamplerPtr allocDefaultStateSampler() const override;

            double distance(const State *state1, const State *state2) const override;

            State *allocState() const override;

            void freeState(State *state) const override;

            double getMajorRadius() const;

            double getMinorRadius() const;

        private:

            double majorRadius_;

            double minorRadius_;
        };
    }
}

#endif


