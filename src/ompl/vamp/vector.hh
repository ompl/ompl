#pragma once

#include <cstdint>
#include <ompl/vamp/vector/interface.hh>

#if defined(__x86_64__)
#include <ompl/vamp/vector/avx.hh>
#elif defined(__ARM_NEON) || defined(__ARM_NEON__)
#include <ompl/vamp/vector/neon.hh>
#endif

namespace ompl::vamp
{
#if defined(__x86_64__)
    using FloatT = float;
    using SimdFloatT = __m256;
    using SimdIntT = __m256i;
#elif defined(__ARM_NEON) || defined(__ARM_NEON__)
    using FloatT = float;
    using SimdFloatT = float32x4_t;
    using SimdIntT = int32x4_t;
#endif

    static constexpr std::size_t FloatVectorWidth = SIMDVector<SimdFloatT>::VectorWidth;
    static constexpr std::size_t FloatVectorAlignment = FloatVectorWidth * sizeof(FloatT);

    template <std::size_t scalars_per_row = FloatVectorWidth, std::size_t num_rows = 1>
    using FloatVector = Vector<SIMDVector<SimdFloatT>, num_rows, scalars_per_row>;

    template <std::size_t scalars_per_row = FloatVectorWidth, std::size_t num_rows = 1>
    using IntVector = Vector<SIMDVector<SimdIntT>, num_rows, scalars_per_row>;
}  // namespace ompl::vamp
