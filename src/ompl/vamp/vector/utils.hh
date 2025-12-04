#pragma once
#include <cstdint>
#include <type_traits>
#include <functional>
#include <array>

#include <ompl/vamp/Utils.h>

namespace ompl::vamp
{
    namespace
    {
        template <auto fn>
        struct return_type
        {
            using type = typename decltype(std::function{fn})::result_type;
        };

        template <typename DataT>
        inline constexpr auto data_size() noexcept -> std::size_t
        {
            return std::tuple_size<DataT>::value;
        }

        // TODO: There's probably a neater way to implement this...
        struct unpack
        {
            template <typename ArrT>
            inline static constexpr auto or_(const ArrT &arr) noexcept -> bool
            {
                return run_or(arr, std::make_index_sequence<data_size<ArrT>()>());
            }

            template <typename ArrT, std::size_t... I>
            inline static constexpr auto run_or(const ArrT &arr, std::index_sequence<I...>) noexcept -> bool
            {
                return (... or arr[I]);
            }

            template <typename ArrT>
            inline static constexpr auto and_(const ArrT &arr) noexcept -> bool
            {
                return run_and(arr, std::make_index_sequence<data_size<ArrT>()>());
            }

            template <typename ArrT, std::size_t... I>
            inline static constexpr auto run_and(const ArrT &arr, std::index_sequence<I...>) noexcept -> bool
            {
                return (... and arr[I]);
            }

            template <typename ArrT>
            inline static constexpr auto sum_(const ArrT &arr) noexcept
            {
                return run_sum(arr, std::make_index_sequence<data_size<ArrT>()>());
            }

            template <typename ArrT, std::size_t... I>
            inline static constexpr auto run_sum(const ArrT &arr, std::index_sequence<I...>) noexcept
            {
                return (... + arr[I]);
            }
        };

        template <auto fn, typename DataT>
        struct applicator
        {
            using RetT = typename std::array<typename return_type<fn>::type, data_size<DataT>()>;

            template <std::size_t... I>
            inline static auto run(std::index_sequence<I...>) noexcept -> RetT
            {
                RetT result;
                (..., void(result[I] = fn()));
                return result;
            }

            template <std::size_t... I>
            inline static auto run(std::index_sequence<I...>, const DataT data) noexcept -> RetT
            {
                RetT result;
                (..., void(result[I] = fn(std::get<I>(data))));
                return result;
            }

            template <std::size_t... I>
            inline static auto run(std::index_sequence<I...>, const DataT l_data, const DataT r_data) noexcept
                -> RetT
            {
                RetT result;
                (..., void(result[I] = fn(std::get<I>(l_data), std::get<I>(r_data))));
                return result;
            }

            template <std::size_t... I>
            inline static auto
            run(std::index_sequence<I...>,
                const DataT i_data,
                const DataT m_data,
                const DataT a_data) noexcept -> RetT
            {
                RetT result;
                (..., void(result[I] = fn(std::get<I>(i_data), std::get<I>(m_data), std::get<I>(a_data))));
                return result;
            }

            template <typename IndexT, typename... FixedArgT, std::size_t... I>
            inline static auto
            run(std::index_sequence<I...>,
                const IndexT i_data,
                const DataT m_data,
                const DataT a_data,
                const FixedArgT... fixed_args) noexcept -> RetT
            {
                RetT result;
                (...,
                 void(
                     result[I] =
                         fn(std::get<I>(i_data), std::get<I>(m_data), std::get<I>(a_data), fixed_args...)));
                return result;
            }

            template <typename IndexT, typename... FixedArgT, std::size_t... I>
            inline static auto
            run(std::index_sequence<I...>, const IndexT i_data, const FixedArgT... fixed_args) noexcept
                -> RetT
            {
                RetT result;
                (..., void(result[I] = fn(std::get<I>(i_data), fixed_args...)));
                return result;
            }

            template <typename... FixedArgT, std::size_t... I>
            inline static auto
            run(std::index_sequence<I...>, const DataT data, const FixedArgT... fixed_args) noexcept -> RetT
            {
                RetT result;
                (..., void(result[I] = fn(std::get<I>(data), fixed_args...)));
                return result;
            }
        };

        template <auto fn, typename DataT>
        inline constexpr auto apply() noexcept
        {
            return applicator<fn, DataT>::run(std::make_index_sequence<data_size<DataT>()>());
        }

        template <auto fn, typename DataT>
        inline constexpr auto apply(DataT data) noexcept
        {
            return applicator<fn, DataT>::run(std::make_index_sequence<data_size<DataT>()>(), data);
        }

        template <auto fn, typename DataT>
        inline constexpr auto apply(DataT l_data, DataT r_data) noexcept
        {
            return applicator<fn, DataT>::run(std::make_index_sequence<data_size<DataT>()>(), l_data, r_data);
        }

        template <auto fn, typename DataT>
        inline constexpr auto apply(DataT l_data, DataT r_data, DataT a_data) noexcept
        {
            return applicator<fn, DataT>::run(
                std::make_index_sequence<data_size<DataT>()>(), l_data, r_data, a_data);
        }

        template <auto fn, typename DataT, typename... FixedArgT>
        inline constexpr auto apply(DataT data, FixedArgT... fixed_args) noexcept
        {
            return applicator<fn, DataT>::run(
                std::make_index_sequence<data_size<DataT>()>(), data, fixed_args...);
        }

        // HACK: This is a gross crufty hack to work around a problem with
        // intersection(AVX intrinsics, template/overload resolution)
        template <auto fn, typename IndexT, typename DataT, typename... FixedArgT>
        inline constexpr auto
        apply_indexed(IndexT i_data, DataT m_data, DataT a_data, FixedArgT... fixed_args) noexcept
        {
            return applicator<fn, DataT>::run(
                std::make_index_sequence<data_size<DataT>()>(), i_data, m_data, a_data, fixed_args...);
        }

        template <auto fn, typename IndexT, typename DataT, typename... FixedArgT>
        inline constexpr auto apply_indexed(IndexT i_data, FixedArgT... fixed_args) noexcept
        {
            return applicator<fn, DataT>::run(
                std::make_index_sequence<data_size<DataT>()>(), i_data, fixed_args...);
        }

        template <typename... Ts>
        struct allow_types
        {
            template <typename QueryT>
            using check = std::
                conditional_t<(... || std::is_same_v<std::decay_t<QueryT>, std::decay_t<Ts>>), bool, void>;
        };

        inline constexpr auto
        fit_scalars_to_vectors(std::size_t num_scalars, std::size_t vector_width) noexcept -> std::size_t
        {
            return c_ceil(static_cast<double>(num_scalars) / static_cast<double>(vector_width));
        }

        // Patching yet another half-baked part of C++...
        template <unsigned int...>
        constexpr std::false_type always_false{};

    }  // namespace

    template <typename S1, typename S2>
    struct same_num_scalars
    {
        static constexpr bool value = (S1::num_scalars == S2::num_scalars);
    };
}  // namespace ompl::vamp
