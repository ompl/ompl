#ifndef OMPL_BASE_SPACES_TORUS_STATE_SPACE_
#define OMPL_BASE_SPACES_TORUS_STATE_SPACE_

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ompl
{
    namespace base
    {
        class MobiusStateSpace : public CompoundStateSpace
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
                double getR1() const
                {
                    return as<RealVectorStateSpace::StateType>(1)->values[0];
                }

                void setS1(double s)
                {
                    as<SO2StateSpace::StateType>(0)->value = s;
                }
                void setR1(double s)
                {
                    as<RealVectorStateSpace::StateType>(1)->values[0] = s;
                }
                void setS1R1(double s, double t)
                {
                    setS1(s);
                    setR1(t);
                }
            };

            MobiusStateSpace(double intervalMax);

            virtual ~MobiusStateSpace() override = default;

            double distance(const State *state1, const State *state2) const override;

            virtual void interpolate(
                const State *from, 
                const State *to, 
                double t, 
                State *state) const override;

            State *allocState() const override;

            void freeState(State *state) const override;
        };
    }
}

#endif


