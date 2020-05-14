// State space in three dimentions with only yaw for rotation. To be used for Dubins Airplane state space.

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

namespace ompl
{
    namespace base
    {
        class SimpleSE3StateSpace : public CompoundStateSpace
        {
        public:
            class StateType : public CompoundStateSpace::StateType
            {
            public:
                StateType() = default;

                double getX() const
                {
                    return as<RealVectorStateSpace::StateType>(0)->values[0];
                }

                double getY() const
                {
                    return as<RealVectorStateSpace::StateType>(0)->values[1];
                }

                double getZ() const
                {
                    return as<RealVectorStateSpace::StateType>(0)->values[2];
                }

                double getYaw() const
                {
                    return as<SO2StateSpace::StateType>(1)->value;
                }

                void setX(double x)
                {
                    as<RealVectorStateSpace::StateType>(0)->values[0] = x;
                }

                void setY(double y)
                {
                    as<RealVectorStateSpace::StateType>(0)->values[1] = y;
                }

                void setZ(double z)
                {
                    as<RealVectorStateSpace::StateType>(0)->values[2] = z;
                }

                void setYaw(double yaw)
                {
                    as<SO2StateSpace::StateType>(1)->value = yaw;
                }

                void setXY(double x, double y)
                {
                    setX(x);
                    setY(y);
                }

                void setXYZ(double x, double y, double z)
                {
                    setX(x);
                    setY(y);
                    setZ(z);
                }

                void setXYYaw(double x, double y, double yaw)
                {
                    setXY(x, y);
                    setYaw(yaw);
                }

                void setXYZYaw(double x, double y, double z, double yaw)
                {
                    setXYZ(x, y, z);
                    setYaw(yaw);
                }
            };

            SimpleSE3StateSpace()
            {
                setName("SimpleSE3" + getName());
                type_ = STATE_SPACE_SE3;
                addSubspace(std::make_shared<RealVectorStateSpace>(3), 1.0);
                addSubspace(std::make_shared<SO2StateSpace>(), 1.0);
                lock();
            }

            ~SimpleSE3StateSpace() override = default;

            void setBounds(const RealVectorBounds &bounds)
            {
                as<RealVectorStateSpace>(0)->setBounds(bounds);
            }

            const RealVectorBounds &getBounds() const
            {
                return as<RealVectorStateSpace>(0)->getBounds();
            }

            State *allocState() const override;
            void freeState(State *state) const override;

            void registerProjections() override;
        };
    }  // namespace base
}  // namespace ompl
