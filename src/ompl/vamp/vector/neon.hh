#pragma once

#include <initializer_list>
#if not defined(__ARM_NEON)
#error "Tried to compile NEON intrinsics on non-ARM platform!"
#endif

#include <cstdint>

#include <ompl/vamp/vector/interface.hh>

#include <arm_neon.h>
#include <limits>

namespace ompl::vamp
{
    template <>
    struct SIMDVector<int32x4_t>
    {
        using VectorT = int32x4_t;
        using ScalarT = int32_t;
        static constexpr std::size_t VectorWidth = 4;
        static constexpr std::size_t Alignment = 16;

        template <unsigned int = 0>
        inline static auto extract(VectorT v, int idx) noexcept -> ScalarT
        {
            return ((int *)(&v))[idx];
        }

        template <unsigned int = 0>
        inline static constexpr auto constant(ScalarT v) noexcept -> VectorT
        {
            return vdupq_n_s32(v);
        }

        template <unsigned int = 0>
        inline static constexpr auto sub(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vsubq_s32(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto add(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vaddq_s32(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto mul(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vmulq_s32(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto bitneg(VectorT l) noexcept -> VectorT
        {
            return vreinterpretq_s32_u32(vmvnq_u32(vreinterpretq_u32_s32(l)));  // maybe a reverse is needed
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_equal(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vreinterpretq_s32_u32(vceqq_s32(l, r));
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_not_equal(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vreinterpretq_s32_u32(vmvnq_u32(vceqq_s32(l, r)));
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_greater_than(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vreinterpretq_s32_u32(vcgeq_s32(l, r));
        }

        template <unsigned int = 0>
        inline static constexpr auto and_(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vreinterpretq_s32_u32(vandq_u32(vreinterpretq_u32_s32(l), vreinterpretq_u32_s32(r)));
        }

        template <unsigned int = 0>
        inline static constexpr auto or_(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vreinterpretq_s32_u32(vorrq_u32(vreinterpretq_u32_s32(l), vreinterpretq_u32_s32(r)));
        }

        template <std::size_t... I>
        inline static constexpr auto
        lshift_lookup(VectorT v, ScalarT shift, std::index_sequence<I...>) noexcept -> VectorT
        {
            VectorT ret = zero_vector();
            std::initializer_list<int>(
                {(shift == I ? (ret = lshift_dispatch<std::integral_constant<int, I>{}>(v)), 0 : 0)...});
            return ret;
        }

        template <ScalarT i>
        inline static constexpr auto lshift_dispatch(VectorT v) noexcept -> VectorT
        {
            return vshlq_n_s32(v, i);
        }

        template <unsigned int = 0>
        inline static constexpr auto shift_left(VectorT v, ScalarT i) noexcept -> VectorT
        {
            return lshift_lookup(v, i, std::make_index_sequence<32>());
        }

        template <std::size_t... I>
        inline static constexpr auto
        rshift_lookup(VectorT v, ScalarT shift, std::index_sequence<I...>) noexcept -> VectorT
        {
            VectorT ret = zero_vector();
            std::initializer_list<int>(
                {(shift == I + 1 ? (ret = rshift_dispatch<std::integral_constant<int, I + 1>{}>(v)),
                  0 :
                                   0)...});
            return ret;
        }

        template <ScalarT i>
        inline static constexpr auto rshift_dispatch(VectorT v) noexcept -> VectorT
        {
            return vshrq_n_s32(v, i);
        }

        template <unsigned int = 0>
        inline static constexpr auto shift_right(VectorT v, ScalarT i) noexcept -> VectorT
        {
            return rshift_lookup(v, i, std::make_index_sequence<32>());
        }

        template <unsigned int = 0>
        inline static auto zero_vector() noexcept -> VectorT
        {
            return vmovq_n_s32(0);
        }

        template <unsigned int = 0>
        inline static auto test_zero(VectorT l, VectorT r) noexcept -> unsigned int
        {
            auto andlr = vandq_u32(vreinterpretq_u32_s32(l), vreinterpretq_u32_s32(r));
            auto horizor = vorr_u32(vget_low_u32(andlr), vget_high_u32(andlr));
            uint32x2_t mask = {0x80000000, 0x80000000};
            auto test = vand_u32(horizor, mask);
            return (vget_lane_u32(test, 0) || vget_lane_u32(test, 1)) == 0;
        }

        template <unsigned int = 0>
        inline static auto load(const ScalarT *const i) noexcept -> VectorT
        {
            return vld1q_s32((const int32_t *const)i);
        }

        template <unsigned int = 0>
        inline static auto load_unaligned(const ScalarT *const i) noexcept -> VectorT
        {
            // NOTE: The same instruction seems to do double-duty for ARM?
            return vld1q_s32((const int32_t *const)i);
        }

        template <unsigned int = 0>
        inline static auto store(ScalarT *i, VectorT v) noexcept -> void
        {
            return vst1q_s32(i, v);
        }

        template <unsigned int = 0>
        inline static auto store_unaligned(ScalarT *i, VectorT v) noexcept -> void
        {
            return vst1q_s32(i, v);
        }

        template <unsigned int = 0>
        inline static constexpr auto blend(VectorT a, VectorT b, VectorT blend_mask) noexcept -> VectorT
        {
            return vbslq_s32(vreinterpretq_u32_s32(blend_mask), b, a);
        }

        template <unsigned int = 0>
        inline static auto mask(VectorT v) noexcept -> unsigned int
        {
            auto MSB = vsliq_n_u32(vdupq_n_u32(0), vreinterpretq_u32_s32(v), 16);
            auto sumtwo = vreinterpret_u32_u16(
                vpadd_u16(vreinterpret_u16_u32(vget_low_u32(MSB)), vreinterpret_u16_u32(vget_high_u32(MSB))));
            auto attempt = vreinterpret_u16_u32(sumtwo);
            auto attempt2 = vreinterpret_u8_u16(attempt);
            auto reorg = vshrn_n_u16(vreinterpretq_u16_u8(vcombine_u8(attempt2, attempt2)), 8);
            return vget_lane_u32(vreinterpret_u32_u8(reorg), 0);
            // IT MAY NEED A REVERSE vrev32_u8
            // vget_lane_u32(vreinterpret_u32_u8(vrev32_u8(reorg)), 0);
        }

        template <typename = void>
        inline static constexpr auto gather(int32x4_t idxs, const ScalarT *base) noexcept -> VectorT
        {
            // Pretty sure there isn't a better way to do a 32-bit lookup table...
            int32x4_t result = vdupq_n_s32(0);
            result = vsetq_lane_s32(base[vgetq_lane_s32(idxs, 0)], result, 0);
            result = vsetq_lane_s32(base[vgetq_lane_s32(idxs, 1)], result, 1);
            result = vsetq_lane_s32(base[vgetq_lane_s32(idxs, 2)], result, 2);
            result = vsetq_lane_s32(base[vgetq_lane_s32(idxs, 3)], result, 3);
            return result;
        }

        template <typename = void>
        inline static constexpr auto
        gather_select(int32x4_t idxs, VectorT mask, VectorT alternative, const ScalarT *base) noexcept
            -> VectorT
        {
            auto overlay = gather(idxs, base);
            return blend(overlay, alternative, mask);
        }

        template <typename OtherVectorT>
        inline static constexpr auto to(VectorT v) noexcept -> OtherVectorT
        {
            if constexpr (std::is_same_v<OtherVectorT, float32x4_t>)
            {
                return vcvtq_f32_s32(v);
            }
            else
            {
                static_assert("Invalid cast-as type!");
            }
        }

        template <typename OtherVectorT>
        inline static constexpr auto from(OtherVectorT v) noexcept -> VectorT
        {
            if constexpr (std::is_same_v<OtherVectorT, float32x4_t>)
            {
                return vcvtq_s32_f32(v);
            }
            else
            {
                static_assert("Invalid cast-as type!");
            }
        }

        template <typename OtherVectorT>
        inline static constexpr auto as(VectorT v) noexcept -> OtherVectorT
        {
            if constexpr (std::is_same_v<OtherVectorT, float32x4_t>)
            {
                return vreinterpretq_f32_s32(v);
            }
            else
            {
                static_assert("Invalid cast-as type!");
            }
        }
    };

    template <>
    struct SIMDVector<float32x4_t>
    {
        using VectorT = float32x4_t;
        using ScalarT = float32_t;
        static constexpr std::size_t VectorWidth = 4;
        static constexpr std::size_t Alignment = 16;

        template <unsigned int = 0>
        inline static auto constant(ScalarT v) noexcept -> VectorT
        {
            return vdupq_n_f32(v);
        }

        template <unsigned int = 0>
        inline static constexpr auto constant_int(unsigned int v) noexcept -> VectorT
        {
            return vcvtq_f32_s32(vdupq_n_s32(v));
        }

        template <unsigned int = 0>
        inline static auto load(const ScalarT *const f) noexcept -> VectorT
        {
            return vld1q_f32(f);
        }

        template <unsigned int = 0>
        inline static auto load_unaligned(const ScalarT *const f) noexcept -> VectorT
        {
            // NOTE: The same instruction seems to do double-duty for ARM?
            return vld1q_f32(f);
        }

        template <unsigned int = 0>
        inline static auto store(ScalarT *f, VectorT v) noexcept -> void
        {
            return vst1q_f32(f, v);
        }

        template <unsigned int = 0>
        inline static auto store_unaligned(ScalarT *f, VectorT v) noexcept -> void
        {
            return vst1q_f32(f, v);
        }

        template <unsigned int = 0>
        inline static auto extract(VectorT v, int idx) noexcept -> ScalarT
        {
            return v[idx];
        }

        // C++ is so dumb. We have to do this (unless someone has a cleverer idea) because (1) vdupq_laneq_f32
        // is a macro and the preprocessor hates commas and (2) you can't use the usual parenthesis trick f or
        // commas with parameter packs, apparently
        template <std::size_t idx>
        inline static constexpr auto broadcast_dispatch(VectorT v) noexcept -> VectorT
        {
            return vdupq_laneq_f32(v, idx);
        }

        template <std::size_t... I>
        inline static constexpr auto
        broadcast_lookup(VectorT v, std::size_t lane, std::index_sequence<I...>) noexcept -> VectorT
        {
            VectorT ret = zero_vector();
            std::initializer_list<int>(
                {(lane == I ? (ret = broadcast_dispatch<std::integral_constant<int, I>{}>(v)), 0 : 0)...});
            return ret;
        }

        template <unsigned int = 0>
        inline static constexpr auto broadcast(VectorT v, std::size_t lane) noexcept -> VectorT
        {
            return broadcast_lookup(v, lane, std::make_index_sequence<VectorWidth>());
        }

        // NOTE: Dummy parameter because otherwise we get constexpr errors with set1_ps...
        template <unsigned int = 0>
        inline static constexpr auto bitneg(VectorT l) noexcept -> VectorT
        {
            return vreinterpretq_f32_u32(vmvnq_u32(vreinterpretq_u32_f32(l)));  // maybe a reverse is needed
        }

        template <unsigned int = 0>
        inline static constexpr auto neg(VectorT l) noexcept -> VectorT
        {
            return vreinterpretq_f32_s32(veorq_s32(
                vreinterpretq_s32_f32(l),
                vreinterpretq_s32_f32(vdupq_n_f32(-0.0))));  // maybe a reverse is needed
        }

        template <unsigned int = 0>
        inline static constexpr auto add(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vaddq_f32(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto sub(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vsubq_f32(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto mul(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vmulq_f32(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_less_equal(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vreinterpretq_f32_u32(vcleq_f32(l, r));
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_less_than(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vreinterpretq_f32_u32(vcltq_f32(l, r));
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_greater_equal(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vreinterpretq_f32_u32(vcgeq_f32(l, r));
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_greater_than(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vreinterpretq_f32_u32(vcgtq_f32(l, r));
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_equal(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vreinterpretq_f32_u32(vceqq_f32(l, r));
        }

        template <unsigned int = 0>
        inline static constexpr auto cmp_not_equal(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vreinterpretq_f32_u32(vmvnq_u32(vceqq_f32(l, r)));
        }

        // NOTE: Dummy parameter because otherwise we get constexpr errors with set1_ps...
        template <unsigned int = 0>
        inline static auto floor(VectorT v) noexcept -> VectorT
        {
            return vrndmq_f32(v);
        }

        template <unsigned int = 0>
        inline static constexpr auto div(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vdivq_f32(l, r);
        }

        template <unsigned int = 0>
        inline static constexpr auto rcp(VectorT l) noexcept -> VectorT
        {
            auto s = vrecpeq_f32(l);
            auto p = vrecpsq_f32(l, s);
            return vmulq_f32(s, p);
        }

        template <unsigned int = 0>
        inline static auto mask(VectorT v) noexcept -> unsigned int
        {
            auto MSB = vsliq_n_u32(vdupq_n_u32(0), vreinterpretq_u32_f32(v), 16);
            auto sumtwo = vreinterpret_u32_u16(
                vpadd_u16(vreinterpret_u16_u32(vget_low_u32(MSB)), vreinterpret_u16_u32(vget_high_u32(MSB))));
            auto attempt = vreinterpret_u16_u32(sumtwo);
            auto attempt2 = vreinterpret_u8_u16(attempt);
            auto reorg = vshrn_n_u16(vreinterpretq_u16_u8(vcombine_u8(attempt2, attempt2)), 8);
            return vget_lane_u32(vreinterpret_u32_u8(reorg), 0);
            // IT MAY NEED A REVERSE vrev32_u8
            // vget_lane_u32(vreinterpret_u32_u8(vrev32_u8(reorg)), 0);
        }

        template <unsigned int = 0>
        inline static auto zero_vector() noexcept -> VectorT
        {
            return vmovq_n_f32(0.0f);
        }

        template <unsigned int = 0>
        inline static auto test_zero(VectorT l, VectorT r) noexcept -> unsigned int
        {
            auto andlr = vandq_u32(vreinterpretq_u32_f32(l), vreinterpretq_u32_f32(r));
            auto horizor = vorr_u32(vget_low_u32(andlr), vget_high_u32(andlr));
            uint32x2_t mask = {0x80000000, 0x80000000};
            auto test = vand_u32(horizor, mask);
            return (vget_lane_u32(test, 0) || vget_lane_u32(test, 1)) == 0;
        }

        template <unsigned int = 0>
        inline static constexpr auto abs(VectorT v) noexcept -> VectorT
        {
            return vabsq_f32(v);
        }

        template <unsigned int = 0>
        inline static constexpr auto and_(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vreinterpretq_f32_u32(vandq_u32(vreinterpretq_u32_f32(l), vreinterpretq_u32_f32(r)));
        }

        template <unsigned int = 0>
        inline static constexpr auto or_(VectorT l, VectorT r) noexcept -> VectorT
        {
            return vreinterpretq_f32_u32(vorrq_u32(vreinterpretq_u32_f32(l), vreinterpretq_u32_f32(r)));
        }

        template <std::size_t... I>
        inline static constexpr auto
        lshift_lookup(VectorT v, ScalarT shift, std::index_sequence<I...>) noexcept -> VectorT
        {
            VectorT ret = zero_vector();
            std::initializer_list<int>(
                {(shift == I ? (ret = lshift_dispatch<std::integral_constant<int, I>{}>(v)), 0 : 0)...});
            return ret;
        }

        template <unsigned int i>
        inline static constexpr auto lshift_dispatch(VectorT v) noexcept -> VectorT
        {
            return vreinterpretq_f32_u32(vshlq_n_u32(vreinterpretq_u32_f32(v), i));
        }

        template <unsigned int = 0>
        inline static constexpr auto shift_left(VectorT v, ScalarT i) noexcept -> VectorT
        {
            return lshift_lookup(v, i, std::make_index_sequence<32>());
        }

        template <std::size_t... I>
        inline static constexpr auto
        rshift_lookup(VectorT v, ScalarT shift, std::index_sequence<I...>) noexcept -> VectorT
        {
            VectorT ret = zero_vector();
            std::initializer_list<int>(
                {(shift == I + 1 ? (ret = rshift_dispatch<std::integral_constant<int, I + 1>{}>(v)),
                  0 :
                                   0)...});
            return ret;
        }

        template <unsigned int i>
        inline static constexpr auto rshift_dispatch(VectorT v) noexcept -> VectorT
        {
            return vreinterpretq_f32_u32(vshrq_n_u32(vreinterpretq_u32_f32(v), i));
        }

        template <unsigned int = 0>
        inline static constexpr auto shift_right(VectorT v, ScalarT i) noexcept -> VectorT
        {
            return rshift_lookup(v, i, std::make_index_sequence<32>());
        }

        template <unsigned int = 0>
        inline static constexpr auto sqrt(VectorT v) noexcept -> VectorT
        {
            return vsqrtq_f32(v);
        }

        template <unsigned int = 0>
        inline static constexpr auto clamp(VectorT v, VectorT lower, VectorT upper) noexcept -> VectorT
        {
            return vminq_f32(vmaxq_f32(v, lower), upper);
        }

        template <unsigned int = 0>
        inline static constexpr auto max(VectorT v, VectorT other) noexcept -> VectorT
        {
            return vmaxq_f32(v, other);
        }

        template <unsigned int = 0>
        inline static constexpr auto hsum(VectorT v) noexcept -> float
        {
            return vaddvq_f32(v);
        }

        // converted from http://gruntthepeon.free.fr/ssemath/neon_mathfun.html
        template <unsigned int = 0>
        inline static constexpr auto sin(VectorT x) noexcept -> VectorT
        {
            using IntVector = SIMDVector<int32x4_t>;

            // Constants
            const auto c_cephes_FOPI = constant(1.27323954473516f);  // 4 / M_PI
            const auto c_minus_cephes_DP1 = constant(-0.78515625f);
            const auto c_minus_cephes_DP2 = constant(-2.4187564849853515625e-4f);
            const auto c_minus_cephes_DP3 = constant(-3.77489497744594108e-8f);
            const auto c_sincof_p0 = constant(-1.9515295891E-4f);
            const auto c_sincof_p1 = constant(8.3321608736E-3f);
            const auto c_sincof_p2 = constant(-1.6666654611E-1f);
            const auto one = constant(1.0f);
            const auto half = constant(0.5f);

            auto sign_mask_sin = cmp_less_than(x, zero_vector());
            x = abs(x);

            auto y = mul(x, c_cephes_FOPI);

            auto emm2 = to<int32x4_t>(y);
            emm2 = IntVector::add(emm2, IntVector::constant(1));
            emm2 = IntVector::and_(emm2, IntVector::constant(~1));
            y = from<int32x4_t>(emm2);

            auto poly_mask = IntVector::and_(emm2, IntVector::constant(2));
            poly_mask = IntVector::cmp_not_equal(poly_mask, IntVector::zero_vector());

            auto xmm1 = mul(y, c_minus_cephes_DP1);
            auto xmm2 = mul(y, c_minus_cephes_DP2);
            auto xmm3 = mul(y, c_minus_cephes_DP3);
            x = add(x, xmm1);
            x = add(x, xmm2);
            x = add(x, xmm3);

            // Update sign mask
            auto temp_mask = IntVector::and_(emm2, IntVector::constant(4));
            temp_mask = IntVector::cmp_not_equal(temp_mask, IntVector::zero_vector());
            sign_mask_sin = vreinterpretq_f32_u32(veorq_u32(
                vreinterpretq_u32_f32(sign_mask_sin),
                vreinterpretq_u32_f32(IntVector::template as<VectorT>(temp_mask))));

            // Evaluate polynomials
            auto z = mul(x, x);

            // First polynomial (cosine)
            auto y1 = mul(z, constant(2.443315711809948E-005f));
            y1 = add(y1, constant(-1.388731625493765E-003f));
            y1 = mul(y1, z);
            y1 = add(y1, constant(4.166664568298827E-002f));
            y1 = mul(y1, z);
            y1 = mul(y1, z);
            y1 = sub(y1, mul(z, half));
            y1 = add(y1, one);

            // Second polynomial (sine)
            auto y2 = mul(z, c_sincof_p0);
            y2 = add(y2, c_sincof_p1);
            y2 = mul(y2, z);
            y2 = add(y2, c_sincof_p2);
            y2 = mul(y2, z);
            y2 = mul(y2, x);
            y2 = add(y2, x);

            auto poly_mask_f = IntVector::template as<VectorT>(poly_mask);
            auto ys = blend(y2, y1, poly_mask_f);
            return blend(ys, neg(ys), sign_mask_sin);
        }

        // NOTE: Dummy parameter because otherwise we get constexpr errors with set1_ps...
        template <unsigned int = 0>
        inline static constexpr auto log(VectorT x) noexcept -> VectorT
        {
            using IntVector = SIMDVector<int32x4_t>;

            const auto half = constant(0.5F);
            const auto one = constant(1.0F);
            auto invalid_mask = cmp_less_equal(x, zero_vector());

            // cut off denormalized values
            x = max(x, vreinterpretq_f32_u32(vdupq_n_u32(0x00800000u)));

            auto emm0 = IntVector::shift_right(as<IntVector::VectorT>(x), 23);

            x = and_(x, vreinterpretq_f32_u32(vdupq_n_u32(~0x7f800000u)));
            x = or_(x, half);

            // keep only the fractional part
            emm0 = IntVector::sub(emm0, IntVector::constant(0x7f));
            auto e = from<IntVector::VectorT>(emm0);

            e = add(e, one);

            // compute approx
            auto mask = cmp_less_than(x, constant(0.707106781186547524f));
            auto tmp = and_(x, mask);
            x = sub(x, one);
            e = sub(e, and_(one, mask));
            x = add(x, tmp);

            auto z = mul(x, x);

            auto y = constant(7.0376836292E-2f);
            y = mul(y, x);
            y = add(y, constant(-1.1514610310E-1f));
            y = mul(y, x);
            y = add(y, constant(1.1676998740E-1f));
            y = mul(y, x);
            y = add(y, constant(-1.2420140846E-1f));
            y = mul(y, x);
            y = add(y, constant(+1.4249322787E-1f));
            y = mul(y, x);
            y = add(y, constant(-1.6668057665E-1f));
            y = mul(y, x);
            y = add(y, constant(+2.0000714765E-1f));
            y = mul(y, x);
            y = add(y, constant(-2.4999993993E-1f));
            y = mul(y, x);
            y = add(y, constant(+3.3333331174E-1f));
            y = mul(y, mul(x, z));
            tmp = mul(e, constant(-2.12194440e-4f));
            y = add(y, tmp);
            tmp = mul(z, half);
            y = sub(y, tmp);
            tmp = mul(e, constant(0.693359375f));
            x = add(x, add(y, tmp));

            x = or_(x, invalid_mask);  // negative arg will be NAN
            return x;
        }

        template <unsigned int = 0>
        inline static constexpr auto blend(VectorT a, VectorT b, VectorT blend_mask) noexcept -> VectorT
        {
            return vbslq_f32(vreinterpretq_u32_f32(blend_mask), b, a);
        }

        // HACK: We only ever use this for trim() and pack_and_pad, and ARM makes it hard to go from
        // a scalar mask to an appropriate vector mask for vbslq. So, we special-case for the values
        // we could possibly get
        template <unsigned int blend_mask>
        inline static constexpr auto blend_constant(VectorT a, VectorT b) noexcept -> VectorT
        {
            if constexpr (blend_mask == 8)
            {
                return vbslq_f32(vcombine_u32(vcreate_u32(0l), vcreate_u32(0xffffffff00000000)), b, a);
            }
            else if constexpr (blend_mask == 12)
            {
                return vbslq_f32(vcombine_u32(vcreate_u32(0l), vcreate_u32(0xffffffffffffffff)), b, a);
            }
            else if constexpr (blend_mask == 14)
            {
                return vbslq_f32(
                    vcombine_u32(vcreate_u32(0xffffffff00000000), vcreate_u32(0xffffffffffffffff)), b, a);
            }
            else
            {
                static_assert(always_false<blend_mask>, "blend_mask not in allowed value set!");
            }
        }

        template <typename OtherVectorT>
        inline static constexpr auto to(VectorT v) noexcept -> OtherVectorT
        {
            return vcvtq_s32_f32(v);
        }

        template <typename OtherVectorT>
        inline static constexpr auto from(OtherVectorT v) noexcept -> VectorT
        {
            return vcvtq_f32_s32(v);
        }

        template <typename OtherVectorT>
        inline static constexpr auto as(VectorT v) noexcept -> OtherVectorT
        {
            return vreinterpretq_s32_f32(v);
        }

        template <typename OtherVectorT>
        inline auto map_to_range(OtherVectorT v) -> VectorT
        {
            const auto v_1 = vandq_s32(v, vdupq_n_s32(1));
            const auto v1_f = vcvtq_f32_s32(v_1);
            const auto v_scaled = vaddq_f32(vcvtq_f32_s32(v), v1_f);
            return vmulq_f32(
                v_scaled, vdupq_n_f32(1.f / static_cast<float>(std::numeric_limits<unsigned int>::max())));
        }

        template <typename = void>
        inline static auto gather(int32x4_t idxs, const ScalarT *base) noexcept -> VectorT
        {
            // Pretty sure there isn't a better way to do a 32-bit lookup table...
            float32x4_t result = vdupq_n_f32(0);
            result = vsetq_lane_f32(base[vgetq_lane_s32(idxs, 0)], result, 0);
            result = vsetq_lane_f32(base[vgetq_lane_s32(idxs, 1)], result, 1);
            result = vsetq_lane_f32(base[vgetq_lane_s32(idxs, 2)], result, 2);
            result = vsetq_lane_f32(base[vgetq_lane_s32(idxs, 3)], result, 3);
            return result;
        }

        template <typename = void>
        inline static constexpr auto
        gather_select(int32x4_t idxs, VectorT mask, VectorT alternative, const ScalarT *base) noexcept
            -> VectorT
        {
            auto overlay = gather(idxs, base);
            return blend(overlay, alternative, mask);
        }
    };
}  // namespace ompl::vamp
