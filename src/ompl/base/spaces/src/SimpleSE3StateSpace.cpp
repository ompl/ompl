// State space in three dimentions with only yaw for rotation. To be used for Dubins Airplane state space.

#include "ompl/base/spaces/SimpleSE3StateSpace.h"
#include "ompl/tools/config/MagicConstants.h"

namespace om = ompl::magic;
namespace ob = ompl::base;

namespace ompl
{
    namespace base
    {
        State *SimpleSE3StateSpace::allocState() const
        {
            auto *state = new StateType();
            allocStateComponents(state);
            return state;
            // return CompoundStateSpace::allocState();
        }

        void SimpleSE3StateSpace::freeState(State *state) const
        {
            CompoundStateSpace::freeState(state);
        }

        void SimpleSE3StateSpace::registerProjections()
        {
            class SimpleSE3DefaultProjection : public ProjectionEvaluator
            {
            public:
                SimpleSE3DefaultProjection(const StateSpace *space) : ProjectionEvaluator(space)
                {
                }

                unsigned int getDimension() const override
                {
                    return 3;
                }

                void defaultCellSizes() override
                {
                    cellSizes_.resize(3);
                    bounds_ = space_->as<SimpleSE3StateSpace>()->getBounds();
                    cellSizes_[0] = (bounds_.high[0] - bounds_.low[0]) / om::PROJECTION_DIMENSION_SPLITS;
                    cellSizes_[1] = (bounds_.high[1] - bounds_.low[1]) / om::PROJECTION_DIMENSION_SPLITS;
                    cellSizes_[2] = (bounds_.high[2] - bounds_.low[2]) / om::PROJECTION_DIMENSION_SPLITS;
                }

                void project(const State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
                {
                    projection = Eigen::Map<const Eigen::VectorXd>(
                        state->as<SimpleSE3StateSpace::StateType>()->as<RealVectorStateSpace::StateType>(0)->values, 3);
                }
            };
            registerDefaultProjection(std::make_shared<SimpleSE3DefaultProjection>(this));
        }
    }  // namespace base
}  // namespace ompl
