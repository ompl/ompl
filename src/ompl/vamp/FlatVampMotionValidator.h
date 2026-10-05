#pragma once

#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/flat/FlatChart.h>
#include <ompl/base/spaces/flat/FlatMotion.h>
#include <ompl/base/spaces/flat/FlatStateSpace.h>
#include <ompl/util/Exception.h>

#include <vamp/collision/environment.hh>
#include <vamp/vector.hh>

#include <boost/container/small_vector.hpp>

#include <array>
#include <cstdint>
#include <optional>
#include <utility>

namespace ompl::vamp
{
    namespace ob = ompl::base;

    /** \brief A flat motion converted to single precision so VAMP can check \e rake samples at once.
     */
    template <std::size_t dimension, std::size_t rake = ::vamp::FloatVectorWidth>
    class FlatVampMotion
    {
    public:
        using Vector = ::vamp::FloatVector<rake>;

        /** \brief One row per flat output dimension and one lane per sample. */
        using Block = ::vamp::FloatVector<rake, dimension>;

        /** \brief Construct a flat VAMP motion from derivative levels 0 through \e levels minus one of \e motion. */
        FlatVampMotion(const ob::FlatMotion &motion, unsigned int levels) : levels_(levels)
        {
            if (motion.outputDimension() != dimension)
                throw Exception("FlatVampMotion needs a motion with one column per flat output dimension");

            const Eigen::MatrixXd &coefficients = motion.coefficients();
            const auto rows = static_cast<unsigned int>(coefficients.rows());

            // Differentiating l times brings the falling factorial (q + l)! / q! down onto the coefficient
            // of power q + l, which becomes the coefficient of power q at level l.
            offsets_.push_back(0u);
            for (unsigned int level = 0; level < levels; ++level)
            {
                const unsigned int powers = rows > level ? rows - level : 1u;
                for (unsigned int power = 0; power < powers; ++power)
                {
                    double scale = 1.;
                    for (unsigned int factor = power + 1u; factor <= power + level; ++factor)
                        scale *= factor;

                    for (std::size_t axis = 0; axis < dimension; ++axis)
                    {
                        const double value = power + level < rows ? scale * coefficients(power + level, axis) : 0.;
                        coefficients_.push_back(static_cast<float>(value));
                    }
                }
                offsets_.push_back(offsets_.back() + powers);
            }
        }

        /** \brief The number of derivative levels this evaluates, counting the flat output itself. */
        unsigned int levels() const
        {
            return levels_;
        }

        /** \brief Write derivative level \e level of the flat output into \e out, with one lane per time
            in \e time, without clamping any time to the duration.

            The level has to be less than \ref levels.
        */
        void evaluate(const Vector &time, unsigned int level, Block &out) const
        {
            const std::size_t first = offsets_[level];
            const std::size_t top = offsets_[level + 1u] - 1u;
            for (std::size_t axis = 0; axis < dimension; ++axis)
            {
                Vector value = Vector::fill(coefficients_[top * dimension + axis]);
                for (std::size_t power = top; power-- > first;)
                    value = value * time + coefficients_[power * dimension + axis];
                out[axis] = value;
            }
        }

    private:
        /** \brief The number of derivative levels this evaluates. */
        unsigned int levels_;

        /** \brief The row of \ref coefficients_ where each level starts, followed by one past the last row. */
        boost::container::small_vector<std::size_t, 8> offsets_;

        /** \brief Ascending-power coefficients in time, level after level, with one row per power and one
            entry per flat output dimension in each row.
            This stays on the stack through order 3. */
        boost::container::small_vector<float, 16 * dimension> coefficients_;
    };

    /** \brief A motion validator for ompl::base::FlatStateSpace over a VAMP robot, which checks \e rake samples in
        one SIMD collision check.

        The flat output is the joint configuration of \e Robot, so the flat output space has to derive from
        ompl::base::RealVectorStateSpace with one dimension per joint and the chart has to be an
        ompl::base::RealVectorFlatChart.

        A sample is valid when every derivative level lies inside the bounds of its component, flat output
        included, and the robot at the flat output collides with nothing in the environment.
        The check reads bounds from the ompl::base::RealVectorBounds on each component, so a component
        enforcing some other shape of bound needs ompl::base::FlatMotionValidator instead.
    */
    template <typename Robot, std::size_t rake = ::vamp::FloatVectorWidth>
    class FlatVampMotionValidator : public ob::MotionValidator
    {
    public:
        using Environment = ::vamp::collision::Environment<::vamp::FloatVector<rake>>;

        using Vector = ::vamp::FloatVector<rake>;

        /** \brief One row per joint and one lane per sample. */
        using Block = typename Robot::template ConfigurationBlock<rake>;

        /** \brief Validate motions in the flat state space \e si plans in against \e env, which has to
            outlive this validator. */
        FlatVampMotionValidator(ob::SpaceInformation *si, const Environment &env) : ob::MotionValidator(si), env_(env)
        {
            defaultSettings();
        }

        /** \brief Validate motions in the flat state space \e si plans in against \e env, which has to
            outlive this validator. */
        FlatVampMotionValidator(const ob::SpaceInformationPtr &si, const Environment &env)
          : ob::MotionValidator(si), env_(env)
        {
            defaultSettings();
        }

        ~FlatVampMotionValidator() override = default;

        /** \brief Whether every sample along the motion from \e s1 to \e s2 is valid, assuming \e s1
            already is. */
        bool checkMotion(const ob::State *s1, const ob::State *s2) const override
        {
            const bool result = sweep(s1, s2);
            if (result)
                valid_++;
            else
                invalid_++;
            return result;
        }

        /** \brief Whether every sample along the motion from \e s1 to \e s2 is valid, assuming \e s1
            already is.

            On a false result \e lastValid.second comes out holding the fraction of the duration of the last
            sample before the first rejected batch, and \e lastValid.first, when it isn't null, comes out
            holding the state at that fraction.
        */
        bool checkMotion(const ob::State *s1, const ob::State *s2,
                         std::pair<ob::State *, double> &lastValid) const override
        {
            const bool result = march(s1, s2, lastValid);
            if (result)
                valid_++;
            else
                invalid_++;
            return result;
        }

    protected:
        /** \brief Whether the robot in every configuration of \e block collides with nothing in the
            environment. */
        virtual bool checkCollisions(const Block &block) const
        {
            return env_.attachments ? Robot::template fkcc_attach<rake>(env_, block) :
                                      Robot::template fkcc<rake>(env_, block);
        }

        /** \brief Take the flat state space out of the space information, throwing unless it plans for
            \e Robot. */
        void defaultSettings()
        {
            stateSpace_ = dynamic_cast<ob::FlatStateSpace *>(si_->getStateSpace().get());
            if (stateSpace_ == nullptr)
                throw Exception("FlatVampMotionValidator needs a FlatStateSpace");
            if (stateSpace_->getOutputDimension() != Robot::dimension)
                throw Exception("FlatVampMotionValidator needs one flat output dimension per joint");
            if (dynamic_cast<const ob::RealVectorStateSpace *>(stateSpace_->getOutputSpace().get()) == nullptr)
                throw Exception("FlatVampMotionValidator needs a flat output space deriving from "
                                "RealVectorStateSpace");
            if (dynamic_cast<const ob::RealVectorFlatChart *>(stateSpace_->getChart().get()) == nullptr)
                throw Exception("FlatVampMotionValidator needs a RealVectorFlatChart");
        }

        /** \brief The flat state space the motions run through. */
        ob::FlatStateSpace *stateSpace_{nullptr};

        /** \brief The obstacles the robot has to avoid. */
        const Environment &env_;

    private:
        /** \brief One value per derivative level and joint, held on the stack through order 3. */
        using LevelValues = boost::container::small_vector<float, 3 * Robot::dimension>;

        /** \brief The values of derivative level \e level of \e state, one per joint. */
        const double *levelValues(const ob::State *state, unsigned int level) const
        {
            const auto *flat = state->as<ob::FlatStateSpace::StateType>();
            if (level == 0u)
                return flat->output()->template as<ob::RealVectorStateSpace::StateType>()->values;
            return flat->derivative(level)->values;
        }

        /** \brief The bounds of derivative level \e level. */
        const ob::RealVectorBounds &levelBounds(unsigned int level) const
        {
            if (level == 0u)
                return stateSpace_->getOutputSpace()->as<ob::RealVectorStateSpace>()->getBounds();
            return stateSpace_->getDerivativeSpace(level)->getBounds();
        }

        /** \brief One edge ready for batched evaluation. */
        struct Edge
        {
            /** \brief The motion in single precision. */
            FlatVampMotion<Robot::dimension, rake> motion;

            /** \brief The joints of the start state, which the motion is displaced from. */
            std::array<float, Robot::dimension> start;

            /** \brief The number of batches to search through the interior. */
            std::uint64_t batches;

            /** \brief The time between consecutive interior samples. */
            float step;
        };

        /** \brief Write the bounds of every derivative level into \e low and \e high, level after level. */
        void readBounds(LevelValues &low, LevelValues &high) const
        {
            for (unsigned int level = 0; level < stateSpace_->getOrder(); ++level)
            {
                const ob::RealVectorBounds &bounds = levelBounds(level);
                for (std::size_t joint = 0; joint < Robot::dimension; ++joint)
                {
                    low.push_back(static_cast<float>(bounds.low[joint]));
                    high.push_back(static_cast<float>(bounds.high[joint]));
                }
            }
        }

        /** \brief Determine whether \e state lies inside every bound and collides with nothing.

            The check reads \e state straight rather than through a polynomial, so a state sitting on a
            bound doesn't get rounded out of it.
        */
        bool isValidState(const ob::State *state, const LevelValues &low, const LevelValues &high) const
        {
            for (unsigned int level = 0; level < stateSpace_->getOrder(); ++level)
            {
                const double *values = levelValues(state, level);
                for (std::size_t joint = 0; joint < Robot::dimension; ++joint)
                {
                    const std::size_t i = level * Robot::dimension + joint;
                    const auto value = static_cast<float>(values[joint]);
                    if (!(value >= low[i] && value <= high[i]))
                        return false;
                }
            }

            Block block;
            const double *values = levelValues(state, 0u);
            for (std::size_t joint = 0; joint < Robot::dimension; ++joint)
                block[joint] = Vector::fill(static_cast<float>(values[joint]));
            return checkCollisions(block);
        }

        /** \brief Prepare \e motion from \e s1 for batched evaluation, with its interior rounded up to
            whole batches.
        */
        Edge prepare(const ob::State *s1, const ob::FlatMotion &motion) const
        {
            // The far end gets checked on its own, so only the interior is left.
            const unsigned int interior = stateSpace_->validSegmentCount(motion) - 1u;
            const std::uint64_t batches = (interior + rake - 1u) / rake;

            // The motion runs in the chart at s1, so its flat output is the displacement from the joints of
            // s1, and adding those joints back gives the configuration.
            const double *startValues = levelValues(s1, 0u);
            std::array<float, Robot::dimension> start{};
            for (std::size_t joint = 0; joint < Robot::dimension; ++joint)
                start[joint] = static_cast<float>(startValues[joint]);

            return {FlatVampMotion<Robot::dimension, rake>(motion, stateSpace_->getOrder()), start, batches,
                    static_cast<float>(motion.duration() / static_cast<double>(rake * batches + 1u))};
        }

        /** \brief Whether every lane of \e edge at the sample numbers in \e sample is valid. */
        bool isValidBatch(const Edge &edge, const Vector &sample, const LevelValues &low, const LevelValues &high,
                          Block &block, Block &derivative) const
        {
            const Vector time = sample * edge.step;
            Vector inside = time == time;
            for (unsigned int level = 0; level < stateSpace_->getOrder(); ++level)
            {
                Block &values = level == 0u ? block : derivative;
                edge.motion.evaluate(time, level, values);
                for (std::size_t joint = 0; joint < Robot::dimension; ++joint)
                {
                    const std::size_t i = level * Robot::dimension + joint;
                    if (level == 0u)
                        values[joint] = values[joint] + edge.start[joint];
                    inside = inside & (values[joint] >= low[i]) & (values[joint] <= high[i]);
                }
            }
            return inside.all() && checkCollisions(block);
        }

        /** \brief Determine whether every sample along the motion from \e s1 to \e s2 is valid, checking the far end
            first and the interior strided.

            Lane j takes the run of samples from j batches + 1 through (j + 1) batches, starting at the top
            and stepping down one sample per batch, so the first batch spreads over the whole motion.
        */
        bool sweep(const ob::State *s1, const ob::State *s2) const
        {
            // Most rejected motions end somewhere invalid, and checking s2 first catches those without
            // paying for a steer.
            LevelValues low, high;
            readBounds(low, high);
            if (!isValidState(s2, low, high))
                return false;

            const std::optional<ob::FlatMotion> motion = stateSpace_->steer(s1, s2);
            if (!motion.has_value())
                return true;

            const Edge edge = prepare(s1, *motion);
            alignas(Vector::S::Alignment) std::array<float, rake> tops{};
            for (std::size_t lane = 0; lane < rake; ++lane)
                tops[lane] = static_cast<float>((lane + 1u) * edge.batches);
            const Vector top(tops.data());

            Block block, derivative;
            for (std::uint64_t batch = 0; batch < edge.batches; ++batch)
                if (!isValidBatch(edge, top - static_cast<float>(batch), low, high, block, derivative))
                    return false;
            return true;
        }

        /** \brief Whether every sample along the motion from \e s1 to \e s2 is valid, walking the interior
            from \e s1 onward and checking the far end last, and recording where the motion stopped being
            valid.

            Batch b holds samples b rake + 1 through (b + 1) rake, so the first rejected batch bounds where
            the motion stops being valid.
        */
        bool march(const ob::State *s1, const ob::State *s2, std::pair<ob::State *, double> &lastValid) const
        {
            LevelValues low, high;
            readBounds(low, high);

            const std::optional<ob::FlatMotion> motion = stateSpace_->steer(s1, s2);
            std::uint64_t passed = 0u;
            std::uint64_t samples = 0u;
            bool result = true;
            if (motion.has_value())
            {
                const Edge edge = prepare(s1, *motion);
                samples = rake * edge.batches;

                alignas(Vector::S::Alignment) std::array<float, rake> firsts{};
                for (std::size_t lane = 0; lane < rake; ++lane)
                    firsts[lane] = static_cast<float>(lane + 1u);
                const Vector first(firsts.data());

                Block block, derivative;
                for (; passed < samples; passed += rake)
                    if (!isValidBatch(edge, first + static_cast<float>(passed), low, high, block, derivative))
                    {
                        result = false;
                        break;
                    }
            }

            if (result && isValidState(s2, low, high))
                return true;

            lastValid.second = static_cast<double>(passed) / static_cast<double>(samples + 1u);
            if (lastValid.first != nullptr)
            {
                if (passed == 0u)
                    si_->copyState(lastValid.first, s1);
                else
                    stateSpace_->interpolate(s1, *motion, lastValid.second, lastValid.first);
            }
            return false;
        }
    };

}  // namespace ompl::vamp
