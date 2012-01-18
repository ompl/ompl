/*
 [auto_generated]
 boost/numeric/odeint/algebra/default_operations.hpp

 [begin_description]
 Default operations. They work with the default numerical types, like float, double, complex< double> ...
 [end_description]

 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#ifndef BOOST_NUMERIC_ODEINT_ALGEBRA_DEFAULT_OPERATIONS_HPP_INCLUDED
#define BOOST_NUMERIC_ODEINT_ALGEBRA_DEFAULT_OPERATIONS_HPP_INCLUDED

#include <algorithm>
#include <cmath>      // for std::max
#include <boost/array.hpp>

#ifndef __CUDACC__
#include <boost/utility/result_of.hpp>
#include <boost/units/quantity.hpp>
#endif

namespace boost {
namespace numeric {
namespace odeint {

/*
 * Conversion of boost::units for use in standard_operations::rel_error and standard_operations::maximum
 */
namespace detail {

template<class T>
struct get_value_impl
{
    static T value(const T &t)
    {
        return t;
    }
    typedef T result_type;
};

#ifndef __CUDACC__
template<class Unit , class T>
struct get_value_impl<boost::units::quantity<Unit , T> >
{
    static T value(const boost::units::quantity<Unit , T> &t)
    {
        return t.value();
    }
    typedef T result_type;
};
#endif

template<class T>
typename get_value_impl<T>::result_type get_value(const T &t)
{
    return get_value_impl<T>::value(t);
}

template<class T , class V>
struct set_value_impl
{
    static void set_value(T &t , const V &v)
    {
        t = v;
    }
};

#ifndef __CUDACC__
template<class Unit , class T , class V>
struct set_value_impl<boost::units::quantity<Unit , T> , V>
{
    static void set_value(boost::units::quantity<Unit , T> &t , const V &v)
    {
        t = boost::units::quantity<Unit , T>::from_value(v);
    }
};
#endif

template<class T , class V>

void set_value(T &t , const V &v)
{
    return set_value_impl<T , V>::set_value(t , v);
}

}



/*
 * Notes:
 *
 * * the results structs are needed in order to work with fusion_algebra
 */
struct default_operations
{

    template< class Fac1 = double >
    struct scale
    {
        const Fac1 m_alpha1;

        scale( const Fac1 &alpha1 ) : m_alpha1( alpha1 ) { }

        template< class T1 >
        void operator()( T1 &t1 ) const
        {
            t1 *= m_alpha1;
        }

        typedef void result_type;
    };

    template< class Fac1 = double >
    struct scale_sum1
    {
        const Fac1 m_alpha1;

        scale_sum1( const Fac1 &alpha1 ) : m_alpha1( alpha1 ) { }

        template< class T1 , class T2 >
        void operator()( T1 &t1 , const T2 &t2 ) const
        {
            t1 = m_alpha1 * t2;
        }

        typedef void result_type;
    };


    template< class Fac1 = double , class Fac2 = Fac1 >
    struct scale_sum2
    {
        const Fac1 m_alpha1;
        const Fac2 m_alpha2;

        scale_sum2( const Fac1 &alpha1 , const Fac2 &alpha2 ) : m_alpha1( alpha1 ) , m_alpha2( alpha2 ) { }

        template< class T1 , class T2 , class T3 >
        void operator()( T1 &t1 , const T2 &t2 , const T3 &t3) const
        {
            t1 = m_alpha1 * t2 + m_alpha2 * t3;
        }

        typedef void result_type;
    };


    template< class Fac1 = double , class Fac2 = Fac1 , class Fac3 = Fac2 >
    struct scale_sum3
    {
        const Fac1 m_alpha1;
        const Fac2 m_alpha2;
        const Fac3 m_alpha3;

        scale_sum3( const Fac1 &alpha1 , const Fac2 &alpha2 , const Fac3 &alpha3 )
        : m_alpha1( alpha1 ) , m_alpha2( alpha2 ) , m_alpha3( alpha3 ) { }

        template< class T1 , class T2 , class T3 , class T4 >
        void operator()( T1 &t1 , const T2 &t2 , const T3 &t3 , const T4 &t4 ) const
        {
            t1 = m_alpha1 * t2 + m_alpha2 * t3 + m_alpha3 * t4;
        }

        typedef void result_type;
    };


    template< class Fac1 = double , class Fac2 = Fac1 , class Fac3 = Fac2 , class Fac4 = Fac3 >
    struct scale_sum4
    {
        const Fac1 m_alpha1;
        const Fac2 m_alpha2;
        const Fac3 m_alpha3;
        const Fac4 m_alpha4;

        scale_sum4( const Fac1 &alpha1 , const Fac2 &alpha2 , const Fac3 &alpha3 , const Fac4 &alpha4 )
        : m_alpha1( alpha1 ) , m_alpha2( alpha2 ) , m_alpha3( alpha3 ) , m_alpha4( alpha4 ) { }

        template< class T1 , class T2 , class T3 , class T4 , class T5 >
        void operator()( T1 &t1 , const T2 &t2 , const T3 &t3 , const T4 &t4 , const T5 &t5) const
        {
            t1 = m_alpha1 * t2 + m_alpha2 * t3 + m_alpha3 * t4 + m_alpha4 * t5;
        }

        typedef void result_type;
    };


    template< class Fac1 = double , class Fac2 = Fac1 , class Fac3 = Fac2 , class Fac4 = Fac3 , class Fac5 = Fac4 >
    struct scale_sum5
    {
        const Fac1 m_alpha1;
        const Fac2 m_alpha2;
        const Fac3 m_alpha3;
        const Fac4 m_alpha4;
        const Fac5 m_alpha5;

        scale_sum5( const Fac1 &alpha1 , const Fac2 &alpha2 , const Fac3 &alpha3 , const Fac4 &alpha4 , const Fac5 &alpha5 )
        : m_alpha1( alpha1 ) , m_alpha2( alpha2 ) , m_alpha3( alpha3 ) , m_alpha4( alpha4 ) , m_alpha5( alpha5 ) { }

        template< class T1 , class T2 , class T3 , class T4 , class T5 , class T6 >
        void operator()( T1 &t1 , const T2 &t2 , const T3 &t3 , const T4 &t4 , const T5 &t5 , const T6 &t6) const
        {
            t1 = m_alpha1 * t2 + m_alpha2 * t3 + m_alpha3 * t4 + m_alpha4 * t5 + m_alpha5 * t6;
        }

        typedef void result_type;
    };


    template< class Fac1 = double , class Fac2 = Fac1 , class Fac3 = Fac2 , class Fac4 = Fac3 , class Fac5 = Fac4 , class Fac6 = Fac5 >
    struct scale_sum6
    {
        const Fac1 m_alpha1;
        const Fac2 m_alpha2;
        const Fac3 m_alpha3;
        const Fac4 m_alpha4;
        const Fac5 m_alpha5;
        const Fac6 m_alpha6;

        scale_sum6( const Fac1 &alpha1 , const Fac2 &alpha2 , const Fac3 &alpha3 , const Fac4 &alpha4 , const Fac5 &alpha5 , const Fac6 &alpha6 )
        : m_alpha1( alpha1 ) , m_alpha2( alpha2 ) , m_alpha3( alpha3 ) , m_alpha4( alpha4 ) , m_alpha5( alpha5 ) , m_alpha6( alpha6 ){ }

        template< class T1 , class T2 , class T3 , class T4 , class T5 , class T6 , class T7 >
        void operator()( T1 &t1 , const T2 &t2 , const T3 &t3 , const T4 &t4 , const T5 &t5 , const T6 &t6 ,const T7 &t7) const
        {
            t1 = m_alpha1 * t2 + m_alpha2 * t3 + m_alpha3 * t4 + m_alpha4 * t5 + m_alpha5 * t6 + m_alpha6 * t7;
        }

        typedef void result_type;
    };


    template< class Fac1 = double , class Fac2 = Fac1 , class Fac3 = Fac2 , class Fac4 = Fac3 , class Fac5 = Fac4 , class Fac6 = Fac5 , class Fac7 = Fac6 >
    struct scale_sum7
    {
        const Fac1 m_alpha1;
        const Fac2 m_alpha2;
        const Fac3 m_alpha3;
        const Fac4 m_alpha4;
        const Fac5 m_alpha5;
        const Fac6 m_alpha6;
        const Fac7 m_alpha7;

        scale_sum7( const Fac1 &alpha1 , const Fac2 &alpha2 , const Fac3 &alpha3 , const Fac4 &alpha4 ,
                const Fac5 &alpha5 , const Fac6 &alpha6 , const Fac7 &alpha7 )
        : m_alpha1( alpha1 ) , m_alpha2( alpha2 ) , m_alpha3( alpha3 ) , m_alpha4( alpha4 ) , m_alpha5( alpha5 ) , m_alpha6( alpha6 ) , m_alpha7( alpha7 ) { }

        template< class T1 , class T2 , class T3 , class T4 , class T5 , class T6 , class T7 , class T8 >
        void operator()( T1 &t1 , const T2 &t2 , const T3 &t3 , const T4 &t4 , const T5 &t5 , const T6 &t6 , const T7 &t7 , const T8 &t8 ) const
        {
            t1 = m_alpha1 * t2 + m_alpha2 * t3 + m_alpha3 * t4 + m_alpha4 * t5 + m_alpha5 * t6 + m_alpha6 * t7 + m_alpha7 * t8;
        }

        typedef void result_type;
    };


    template< class Fac1 = double , class Fac2 = Fac1 , class Fac3 = Fac2 , class Fac4 = Fac3 , class Fac5 = Fac4 , class Fac6 = Fac5 , class Fac7 = Fac6 , class Fac8 = Fac7 >
    struct scale_sum8
    {
        const Fac1 m_alpha1;
        const Fac2 m_alpha2;
        const Fac3 m_alpha3;
        const Fac4 m_alpha4;
        const Fac5 m_alpha5;
        const Fac6 m_alpha6;
        const Fac7 m_alpha7;
        const Fac8 m_alpha8;

        scale_sum8( const Fac1 &alpha1 , const Fac2 &alpha2 , const Fac3 &alpha3 , const Fac4 &alpha4 ,
                const Fac5 &alpha5 , const Fac6 &alpha6 , const Fac7 &alpha7 , const Fac8 &alpha8 )
        : m_alpha1( alpha1 ) , m_alpha2( alpha2 ) , m_alpha3( alpha3 ) , m_alpha4( alpha4 ) , m_alpha5( alpha5 ) , m_alpha6( alpha6 ) , m_alpha7( alpha7 ) , m_alpha8( alpha8 ) { }

        template< class T1 , class T2 , class T3 , class T4 , class T5 , class T6 , class T7 , class T8 , class T9 >
        void operator()( T1 &t1 , const T2 &t2 , const T3 &t3 , const T4 &t4 , const T5 &t5 , const T6 &t6 , const T7 &t7 , const T8 &t8 , const T9 &t9 ) const
        {
            t1 = m_alpha1 * t2 + m_alpha2 * t3 + m_alpha3 * t4 + m_alpha4 * t5 + m_alpha5 * t6 + m_alpha6 * t7 + m_alpha7 * t8 + m_alpha8 * t9;
        }

        typedef void result_type;
    };

    template< class Fac1 = double , class Fac2 = Fac1 , class Fac3 = Fac2 , class Fac4 = Fac3 , class Fac5 = Fac4 , class Fac6 = Fac5 , class Fac7 = Fac6 , class Fac8 = Fac7 , class Fac9 = Fac8 >
    struct scale_sum9
    {
        const Fac1 m_alpha1;
        const Fac2 m_alpha2;
        const Fac3 m_alpha3;
        const Fac4 m_alpha4;
        const Fac5 m_alpha5;
        const Fac6 m_alpha6;
        const Fac7 m_alpha7;
        const Fac8 m_alpha8;
        const Fac9 m_alpha9;

        scale_sum9( const Fac1 &alpha1 , const Fac2 &alpha2 , const Fac3 &alpha3 , const Fac4 &alpha4 ,
                const Fac5 &alpha5 , const Fac6 &alpha6 , const Fac7 &alpha7 , const Fac8 &alpha8 , const Fac9 &alpha9 )
        : m_alpha1( alpha1 ) , m_alpha2( alpha2 ) , m_alpha3( alpha3 ) , m_alpha4( alpha4 ) , m_alpha5( alpha5 ) , m_alpha6( alpha6 ) , m_alpha7( alpha7 ) , m_alpha8( alpha8 ) , m_alpha9( alpha9 ) { }

        template< class T1 , class T2 , class T3 , class T4 , class T5 , class T6 , class T7 , class T8 , class T9 , class T10 >
        void operator()( T1 &t1 , const T2 &t2 , const T3 &t3 , const T4 &t4 , const T5 &t5 , const T6 &t6 , const T7 &t7 , const T8 &t8 , const T9 &t9 , const T10 &t10 ) const
        {
            t1 = m_alpha1 * t2 + m_alpha2 * t3 + m_alpha3 * t4 + m_alpha4 * t5 + m_alpha5 * t6 + m_alpha6 * t7 + m_alpha7 * t8 + m_alpha8 * t9 + m_alpha9 * t10;
        }

        typedef void result_type;
    };

    template< class Fac1 = double , class Fac2 = Fac1 , class Fac3 = Fac2 , class Fac4 = Fac3 , class Fac5 = Fac4 , class Fac6 = Fac5 , class Fac7 = Fac6 , class Fac8 = Fac7 , class Fac9 = Fac8 , class Fac10 = Fac9 >
    struct scale_sum10
    {
        const Fac1 m_alpha1;
        const Fac2 m_alpha2;
        const Fac3 m_alpha3;
        const Fac4 m_alpha4;
        const Fac5 m_alpha5;
        const Fac6 m_alpha6;
        const Fac7 m_alpha7;
        const Fac8 m_alpha8;
        const Fac9 m_alpha9;
        const Fac10 m_alpha10;

        scale_sum10( const Fac1 &alpha1 , const Fac2 &alpha2 , const Fac3 &alpha3 , const Fac4 &alpha4 ,
                const Fac5 &alpha5 , const Fac6 &alpha6 , const Fac7 &alpha7 , const Fac8 &alpha8 , const Fac9 &alpha9 , const Fac10 &alpha10 )
        : m_alpha1( alpha1 ) , m_alpha2( alpha2 ) , m_alpha3( alpha3 ) , m_alpha4( alpha4 ) , m_alpha5( alpha5 ) , m_alpha6( alpha6 ) , m_alpha7( alpha7 ) , m_alpha8( alpha8 ) , m_alpha9( alpha9 ) , m_alpha10( alpha10 ) { }

        template< class T1 , class T2 , class T3 , class T4 , class T5 , class T6 , class T7 , class T8 , class T9 , class T10 , class T11 >
        void operator()( T1 &t1 , const T2 &t2 , const T3 &t3 , const T4 &t4 , const T5 &t5 , const T6 &t6 , const T7 &t7 , const T8 &t8 , const T9 &t9 , const T10 &t10 , const T11 &t11 ) const
        {
            t1 = m_alpha1 * t2 + m_alpha2 * t3 + m_alpha3 * t4 + m_alpha4 * t5 + m_alpha5 * t6 + m_alpha6 * t7 + m_alpha7 * t8 + m_alpha8 * t9 + m_alpha9 * t10 + m_alpha10 * t11;
        }

        typedef void result_type;
    };


    template< class Fac1 = double , class Fac2 = Fac1 , class Fac3 = Fac2 , class Fac4 = Fac3 , class Fac5 = Fac4 , class Fac6 = Fac5 , class Fac7 = Fac6 , class Fac8 = Fac7 , class Fac9 = Fac8 , class Fac10 = Fac9 , class Fac11 = Fac10 >
    struct scale_sum11
    {
        const Fac1 m_alpha1;
        const Fac2 m_alpha2;
        const Fac3 m_alpha3;
        const Fac4 m_alpha4;
        const Fac5 m_alpha5;
        const Fac6 m_alpha6;
        const Fac7 m_alpha7;
        const Fac8 m_alpha8;
        const Fac9 m_alpha9;
        const Fac10 m_alpha10;
        const Fac11 m_alpha11;

        scale_sum11( const Fac1 &alpha1 , const Fac2 &alpha2 , const Fac3 &alpha3 , const Fac4 &alpha4 ,
                const Fac5 &alpha5 , const Fac6 &alpha6 , const Fac7 &alpha7 , const Fac8 &alpha8 , const Fac9 &alpha9 ,
                const Fac10 &alpha10 , const Fac11 &alpha11 )
        : m_alpha1( alpha1 ) , m_alpha2( alpha2 ) , m_alpha3( alpha3 ) , m_alpha4( alpha4 ) , m_alpha5( alpha5 ) , m_alpha6( alpha6 ) , m_alpha7( alpha7 ) , m_alpha8( alpha8 ) , m_alpha9( alpha9 ) , m_alpha10( alpha10 ) , m_alpha11( alpha11 ) { }

        template< class T1 , class T2 , class T3 , class T4 , class T5 , class T6 , class T7 , class T8 , class T9 , class T10 , class T11 , class T12 >
        void operator()( T1 &t1 , const T2 &t2 , const T3 &t3 , const T4 &t4 , const T5 &t5 , const T6 &t6 , const T7 &t7 , const T8 &t8 , const T9 &t9 , const T10 &t10 , const T11 &t11 , const T12 &t12 ) const
        {
            t1 = m_alpha1 * t2 + m_alpha2 * t3 + m_alpha3 * t4 + m_alpha4 * t5 + m_alpha5 * t6 + m_alpha6 * t7 + m_alpha7 * t8 + m_alpha8 * t9 + m_alpha9 * t10 + m_alpha10 * t11 + m_alpha11 * t12;
        }

        typedef void result_type;
    };

    template< class Fac1 = double , class Fac2 = Fac1 , class Fac3 = Fac2 , class Fac4 = Fac3 , class Fac5 = Fac4 , class Fac6 = Fac5 , class Fac7 = Fac6 , class Fac8 = Fac7 , class Fac9 = Fac8 , class Fac10 = Fac9 , class Fac11 = Fac10 , class Fac12 = Fac11 >
    struct scale_sum12
    {
        const Fac1 m_alpha1;
        const Fac2 m_alpha2;
        const Fac3 m_alpha3;
        const Fac4 m_alpha4;
        const Fac5 m_alpha5;
        const Fac6 m_alpha6;
        const Fac7 m_alpha7;
        const Fac8 m_alpha8;
        const Fac9 m_alpha9;
        const Fac10 m_alpha10;
        const Fac11 m_alpha11;
        const Fac12 m_alpha12;

        scale_sum12( const Fac1 &alpha1 , const Fac2 &alpha2 , const Fac3 &alpha3 , const Fac4 &alpha4 ,
                const Fac5 &alpha5 , const Fac6 &alpha6 , const Fac7 &alpha7 , const Fac8 &alpha8 , const Fac9 &alpha9 ,
                const Fac10 &alpha10 , const Fac11 &alpha11 , const Fac12 &alpha12 )
        : m_alpha1( alpha1 ) , m_alpha2( alpha2 ) , m_alpha3( alpha3 ) , m_alpha4( alpha4 ) , m_alpha5( alpha5 ) , m_alpha6( alpha6 ) , m_alpha7( alpha7 ) , m_alpha8( alpha8 ) , m_alpha9( alpha9 ) , m_alpha10( alpha10 ) , m_alpha11( alpha11 ) , m_alpha12( alpha12 ) { }

        template< class T1 , class T2 , class T3 , class T4 , class T5 , class T6 , class T7 , class T8 , class T9 , class T10 , class T11 , class T12 , class T13 >
        void operator()( T1 &t1 , const T2 &t2 , const T3 &t3 , const T4 &t4 , const T5 &t5 , const T6 &t6 , const T7 &t7 , const T8 &t8 , const T9 &t9 , const T10 &t10 , const T11 &t11 , const T12 &t12 , const T13 &t13 ) const
        {
            t1 = m_alpha1 * t2 + m_alpha2 * t3 + m_alpha3 * t4 + m_alpha4 * t5 + m_alpha5 * t6 + m_alpha6 * t7 + m_alpha7 * t8 + m_alpha8 * t9 + m_alpha9 * t10 + m_alpha10 * t11 + m_alpha11 * t12 + m_alpha12 * t13;
        }

        typedef void result_type;
    };

    template< class Fac1 = double , class Fac2 = Fac1 , class Fac3 = Fac2 , class Fac4 = Fac3 , class Fac5 = Fac4 , class Fac6 = Fac5 , class Fac7 = Fac6 , class Fac8 = Fac7 , class Fac9 = Fac8 , class Fac10 = Fac9 , class Fac11 = Fac10 , class Fac12 = Fac11 , class Fac13 = Fac12 >
    struct scale_sum13
    {
        const Fac1 m_alpha1;
        const Fac2 m_alpha2;
        const Fac3 m_alpha3;
        const Fac4 m_alpha4;
        const Fac5 m_alpha5;
        const Fac6 m_alpha6;
        const Fac7 m_alpha7;
        const Fac8 m_alpha8;
        const Fac9 m_alpha9;
        const Fac10 m_alpha10;
        const Fac11 m_alpha11;
        const Fac12 m_alpha12;
        const Fac13 m_alpha13;

        scale_sum13( const Fac1 &alpha1 , const Fac2 &alpha2 , const Fac3 &alpha3 , const Fac4 &alpha4 ,
                const Fac5 &alpha5 , const Fac6 &alpha6 , const Fac7 &alpha7 , const Fac8 &alpha8 , const Fac9 &alpha9 ,
                const Fac10 &alpha10 , const Fac11 &alpha11 , const Fac12 &alpha12 , const Fac13 &alpha13 )
        : m_alpha1( alpha1 ) , m_alpha2( alpha2 ) , m_alpha3( alpha3 ) , m_alpha4( alpha4 ) , m_alpha5( alpha5 ) , m_alpha6( alpha6 ) , m_alpha7( alpha7 ) , m_alpha8( alpha8 ) , m_alpha9( alpha9 ) , m_alpha10( alpha10 ) , m_alpha11( alpha11 ) , m_alpha12( alpha12 ) , m_alpha13( alpha13 ) { }

        template< class T1 , class T2 , class T3 , class T4 , class T5 , class T6 , class T7 , class T8 , class T9 , class T10 , class T11 , class T12 , class T13 , class T14 >
        void operator()( T1 &t1 , const T2 &t2 , const T3 &t3 , const T4 &t4 , const T5 &t5 , const T6 &t6 , const T7 &t7 , const T8 &t8 , const T9 &t9 , const T10 &t10 , const T11 &t11 , const T12 &t12 , const T13 &t13 , const T14 &t14 ) const
        {
            t1 = m_alpha1 * t2 + m_alpha2 * t3 + m_alpha3 * t4 + m_alpha4 * t5 + m_alpha5 * t6 + m_alpha6 * t7 + m_alpha7 * t8 + m_alpha8 * t9 + m_alpha9 * t10 + m_alpha10 * t11 + m_alpha11 * t12 + m_alpha12 * t13 + m_alpha13 * t14;
        }

        typedef void result_type;
    };

    template< class Fac1 = double , class Fac2 = Fac1 , class Fac3 = Fac2 , class Fac4 = Fac3 , class Fac5 = Fac4 , class Fac6 = Fac5 , class Fac7 = Fac6 , class Fac8 = Fac7 , class Fac9 = Fac8 , class Fac10 = Fac9 , class Fac11 = Fac10 , class Fac12 = Fac11 , class Fac13 = Fac12 , class Fac14 = Fac13 >
    struct scale_sum14
    {
        const Fac1 m_alpha1;
        const Fac2 m_alpha2;
        const Fac3 m_alpha3;
        const Fac4 m_alpha4;
        const Fac5 m_alpha5;
        const Fac6 m_alpha6;
        const Fac7 m_alpha7;
        const Fac8 m_alpha8;
        const Fac9 m_alpha9;
        const Fac10 m_alpha10;
        const Fac11 m_alpha11;
        const Fac12 m_alpha12;
        const Fac13 m_alpha13;
        const Fac14 m_alpha14;

        scale_sum14( const Fac1 &alpha1 , const Fac2 &alpha2 , const Fac3 &alpha3 , const Fac4 &alpha4 ,
                const Fac5 &alpha5 , const Fac6 &alpha6 , const Fac7 &alpha7 , const Fac8 &alpha8 , const Fac9 &alpha9 ,
                const Fac10 &alpha10 , const Fac11 &alpha11 , const Fac12 &alpha12 , const Fac13 &alpha13 , const Fac14 &alpha14 )
        : m_alpha1( alpha1 ) , m_alpha2( alpha2 ) , m_alpha3( alpha3 ) , m_alpha4( alpha4 ) , m_alpha5( alpha5 ) , m_alpha6( alpha6 ) , m_alpha7( alpha7 ) , m_alpha8( alpha8 ) , m_alpha9( alpha9 ) , m_alpha10( alpha10 ) , m_alpha11( alpha11 ) , m_alpha12( alpha12 ) , m_alpha13( alpha13 ) , m_alpha14( alpha14 ) { }

        template< class T1 , class T2 , class T3 , class T4 , class T5 , class T6 , class T7 , class T8 , class T9 , class T10 , class T11 , class T12 , class T13 , class T14 , class T15 >
        void operator()( T1 &t1 , const T2 &t2 , const T3 &t3 , const T4 &t4 , const T5 &t5 , const T6 &t6 , const T7 &t7 , const T8 &t8 , const T9 &t9 , const T10 &t10 , const T11 &t11 , const T12 &t12 , const T13 &t13 , const T14 &t14 , const T15 &t15 ) const
        {
            t1 = m_alpha1 * t2 + m_alpha2 * t3 + m_alpha3 * t4 + m_alpha4 * t5 + m_alpha5 * t6 + m_alpha6 * t7 + m_alpha7 * t8 + m_alpha8 * t9 + m_alpha9 * t10 + m_alpha10 * t11 + m_alpha11 * t12 + m_alpha12 * t13 + m_alpha13 * t14 + m_alpha14 * t15;
        }

        typedef void result_type;
    };

    template< class Fac1 = double , class Fac2 = Fac1 >
    struct scale_sum_swap2
    {
        const Fac1 m_alpha1;
        const Fac2 m_alpha2;

        scale_sum_swap2( const Fac1 &alpha1 , const Fac2 &alpha2 ) : m_alpha1( alpha1 ) , m_alpha2( alpha2 ) { }

        template< class T1 , class T2 , class T3 >
        void operator()( T1 &t1 , T2 &t2 , const T3 &t3) const
        {
            const T1 tmp( t1 );
            t1 = m_alpha1 * t2 + m_alpha2 * t3;
            t2 = tmp;
        }

        typedef void result_type;
    };

    /*
     * for usage in for_each2
     *
     * Works with boost::units by eliminating the unit
     */
    template< class Fac1 = double >
    struct rel_error
    {
        const Fac1 m_eps_abs , m_eps_rel , m_a_x , m_a_dxdt;

        rel_error( const Fac1 &eps_abs , const Fac1 &eps_rel , const Fac1 &a_x , const Fac1 &a_dxdt )
        : m_eps_abs( eps_abs ) , m_eps_rel( eps_rel ) , m_a_x( a_x ) , m_a_dxdt( a_dxdt ) { }


        template< class T1 , class T2 , class T3 >
        void operator()( T3 &t3 , const T1 &t1 , const T2 &t2 ) const
        {
            using std::abs;
            using detail::get_value;
            using detail::set_value;
            set_value( t3 , abs( get_value( t3 ) ) / ( m_eps_abs + m_eps_rel * ( m_a_x * abs( get_value( t1 ) ) + m_a_dxdt * abs( get_value( t2 ) ) ) ) );
        }

        typedef void result_type;
    };


    /*
     * for usage in reduce
     */

    template< class Value >
    struct maximum
    {
        template< class Fac1 , class Fac2 >
        Value operator()( const Fac1 &t1 , const Fac2 &t2 ) const
        {
            using std::max;
            using std::abs;
            using detail::get_value;
            Value a1 = abs( get_value( t1 ) ) , a2 = abs( get_value( t2 ) );
            return ( a1 < a2 ) ? a2 : a1 ;
        }

        typedef Value result_type;
    };


};


} // odeint
} // numeric
} // boost


#endif // BOOST_NUMERIC_ODEINT_ALGEBRA_DEFAULT_OPERATIONS_HPP_INCLUDED
