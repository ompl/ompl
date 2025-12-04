#pragma once

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <utility>

#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/vamp/vector.hh>

namespace ompl::vamp
{
    namespace ob = ompl::base;

    //==========================================================================
    // Constants
    //==========================================================================

    namespace constants
    {
        // Backported from C++20
        inline constexpr double e = 2.718281828459045235360287471352662498L;
        inline constexpr double pi = 3.141592653589793238462643383279502884L;
    }  // namespace constants

    //==========================================================================
    // General Utilities
    //==========================================================================

    inline constexpr auto round_size(std::size_t size, std::size_t block) noexcept -> std::size_t
    {
        return ((size + block - 1) / block) * block;
    }

    inline bool is_aligned(const void *ptr, uintptr_t alignment) noexcept
    {
        auto iptr = reinterpret_cast<uintptr_t>(ptr);
        return not(iptr % alignment);
    }

    template <typename T, std::size_t alignment, std::size_t vector_size>
    inline auto vector_alloc(std::size_t n) noexcept -> T *
    {
        return static_cast<T *>(aligned_alloc(alignment, sizeof(T) * round_size(n, vector_size)));
    }

    // Because ceil isn't constexpr until C++23
    inline constexpr auto c_ceil(double d) noexcept -> std::size_t
    {
        const auto s = static_cast<std::size_t>(d);
        return d > s ? s + 1 : s;
    }

    // Same deal with div()
    inline constexpr auto c_div(std::size_t idx, std::size_t dim) noexcept
        -> std::pair<std::size_t, std::size_t>
    {
        return {idx / dim, idx % dim};
    }

    inline auto get_elapsed_nanoseconds(const std::chrono::time_point<std::chrono::steady_clock> &start)
        -> std::size_t
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start)
            .count();
    }

    //==========================================================================
    // OMPL Conversion Utilities
    //==========================================================================

    /// Convert an OMPL state to a VAMP Configuration
    template <typename Robot>
    inline auto ompl_to_vamp(const ob::State *state) -> typename Robot::Configuration
    {
        using Configuration = typename Robot::Configuration;

        alignas(Configuration::S::Alignment)
            std::array<typename Configuration::S::ScalarT, Configuration::num_scalars>
                aligned_buffer{};

        const auto *as = state->as<ob::RealVectorStateSpace::StateType>();
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            aligned_buffer[i] = static_cast<float>(as->values[i]);
        }

        return Configuration(aligned_buffer.data());
    }

    /// Convert a VAMP Configuration to an OMPL state
    template <typename Robot>
    inline void vamp_to_ompl(const typename Robot::Configuration &config, ob::State *state)
    {
        auto *as = state->as<ob::RealVectorStateSpace::StateType>();
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            as->values[i] = static_cast<double>(config[{i, 0}]);
        }
    }

    /// Get OMPL RealVectorBounds from Robot scaling parameters
    template <typename Robot>
    inline auto get_robot_bounds() -> ob::RealVectorBounds
    {
        using Configuration = typename Robot::Configuration;

        std::array<float, Robot::dimension> zeros{};
        std::array<float, Robot::dimension> ones{};
        std::fill(ones.begin(), ones.end(), 1.0f);

        auto zero_v = Configuration(zeros);
        auto one_v = Configuration(ones);

        Robot::scale_configuration(zero_v);
        Robot::scale_configuration(one_v);

        ob::RealVectorBounds bounds(Robot::dimension);
        for (std::size_t i = 0; i < Robot::dimension; ++i)
        {
            bounds.setLow(i, zero_v[{i, 0}]);
            bounds.setHigh(i, one_v[{i, 0}]);
        }

        return bounds;
    }

}  // namespace ompl::vamp

