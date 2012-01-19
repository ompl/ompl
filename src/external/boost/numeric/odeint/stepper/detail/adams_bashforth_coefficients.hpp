/*
 [auto_generated]
 boost/numeric/odeint/stepper/detail/adams_bashforth_coefficients.hpp

 [begin_description]
 Definition of the coefficients for the Adams-Bashforth method.
 [end_description]

 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#ifndef BOOST_NUMERIC_ODEINT_STEPPER_DETAIL_ADAMS_BASHFORTH_COEFFICIENTS_HPP_INCLUDED
#define BOOST_NUMERIC_ODEINT_STEPPER_DETAIL_ADAMS_BASHFORTH_COEFFICIENTS_HPP_INCLUDED

#include <boost/array.hpp>


namespace boost {
namespace numeric {
namespace odeint {
namespace detail {

template< class Value , size_t Steps >
class adams_bashforth_coefficients ;

template< class Value >
class adams_bashforth_coefficients< Value , 1 > : public boost::array< Value , 1 >
{
public:
    adams_bashforth_coefficients( void )
    : boost::array< Value , 1 >()
      {
        (*this)[0] = static_cast< Value >( 1.0 );
      }
};


template< class Value >
class adams_bashforth_coefficients< Value , 2 > : public boost::array< Value , 2 >
{
public:
    adams_bashforth_coefficients( void )
    : boost::array< Value , 2 >()
      {
        (*this)[0] = static_cast< Value >( 3.0 ) / static_cast< Value >( 2.0 );
        (*this)[1] = -static_cast< Value >( 1.0 ) / static_cast< Value >( 2.0 );
      }
};


template< class Value >
class adams_bashforth_coefficients< Value , 3 > : public boost::array< Value , 3 >
{
public:
    adams_bashforth_coefficients( void )
    : boost::array< Value , 3 >()
      {
        (*this)[0] = static_cast< Value >( 23.0 ) / static_cast< Value >( 12.0 );
        (*this)[1] = -static_cast< Value >( 4.0 ) / static_cast< Value >( 3.0 );
        (*this)[2] = static_cast< Value >( 5.0 ) / static_cast< Value >( 12.0 );
      }
};


template< class Value >
class adams_bashforth_coefficients< Value , 4 > : public boost::array< Value , 4 >
{
public:
    adams_bashforth_coefficients( void )
    : boost::array< Value , 4 >()
      {
        (*this)[0] = static_cast< Value >( 55.0 ) / static_cast< Value >( 24.0 );
        (*this)[1] = -static_cast< Value >( 59.0 ) / static_cast< Value >( 24.0 );
        (*this)[2] = static_cast< Value >( 37.0 ) / static_cast< Value >( 24.0 );
        (*this)[3] = -static_cast< Value >( 3.0 ) / static_cast< Value >( 8.0 );
      }
};


template< class Value >
class adams_bashforth_coefficients< Value , 5 > : public boost::array< Value , 5 >
{
public:
    adams_bashforth_coefficients( void )
    : boost::array< Value , 5 >()
      {
        (*this)[0] = static_cast< Value >( 1901.0 ) / static_cast< Value >( 720.0 );
        (*this)[1] = -static_cast< Value >( 1387.0 ) / static_cast< Value >( 360.0 );
        (*this)[2] = static_cast< Value >( 109.0 ) / static_cast< Value >( 30.0 );
        (*this)[3] = -static_cast< Value >( 637.0 ) / static_cast< Value >( 360.0 );
        (*this)[4] = static_cast< Value >( 251.0 ) / static_cast< Value >( 720.0 );
      }
};


template< class Value >
class adams_bashforth_coefficients< Value , 6 > : public boost::array< Value , 6 >
{
public:
    adams_bashforth_coefficients( void )
    : boost::array< Value , 6 >()
      {
        (*this)[0] = static_cast< Value >( 4277.0 ) / static_cast< Value >( 1440.0 );
        (*this)[1] = -static_cast< Value >( 2641.0 ) / static_cast< Value >( 480.0 );
        (*this)[2] = static_cast< Value >( 4991.0 ) / static_cast< Value >( 720.0 );
        (*this)[3] = -static_cast< Value >( 3649.0 ) / static_cast< Value >( 720.0 );
        (*this)[4] = static_cast< Value >( 959.0 ) / static_cast< Value >( 480.0 );
        (*this)[5] = -static_cast< Value >( 95.0 ) / static_cast< Value >( 288.0 );
      }
};


template< class Value >
class adams_bashforth_coefficients< Value , 7 > : public boost::array< Value , 7 >
{
public:
    adams_bashforth_coefficients( void )
    : boost::array< Value , 7 >()
      {
        (*this)[0] = static_cast< Value >( 198721.0 ) / static_cast< Value >( 60480.0 );
        (*this)[1] = -static_cast< Value >( 18637.0 ) / static_cast< Value >( 2520.0 );
        (*this)[2] = static_cast< Value >( 235183.0 ) / static_cast< Value >( 20160.0 );
        (*this)[3] = -static_cast< Value >( 10754.0 ) / static_cast< Value >( 945.0 );
        (*this)[4] = static_cast< Value >( 135713.0 ) / static_cast< Value >( 20160.0 );
        (*this)[5] = -static_cast< Value >( 5603.0 ) / static_cast< Value >( 2520.0 );
        (*this)[6] = static_cast< Value >( 19087.0 ) / static_cast< Value >( 60480.0 );
      }
};


template< class Value >
class adams_bashforth_coefficients< Value , 8 > : public boost::array< Value , 8 >
{
public:
    adams_bashforth_coefficients( void )
    : boost::array< Value , 8 >()
      {
        (*this)[0] = static_cast< Value >( 16083.0 ) / static_cast< Value >( 4480.0 );
        (*this)[1] = -static_cast< Value >( 1152169.0 ) / static_cast< Value >( 120960.0 );
        (*this)[2] = static_cast< Value >( 242653.0 ) / static_cast< Value >( 13440.0 );
        (*this)[3] = -static_cast< Value >( 296053.0 ) / static_cast< Value >( 13440.0 );
        (*this)[4] = static_cast< Value >( 2102243.0 ) / static_cast< Value >( 120960.0 );
        (*this)[5] = -static_cast< Value >( 115747.0 ) / static_cast< Value >( 13440.0 );
        (*this)[6] = static_cast< Value >( 32863.0 ) / static_cast< Value >( 13440.0 );
        (*this)[7] = -static_cast< Value >( 5257.0 ) / static_cast< Value >( 17280.0 );
      }
};







} // detail
} // odeint
} // numeric
} // boost



#endif // BOOST_NUMERIC_ODEINT_STEPPER_DETAIL_ADAMS_BASHFORTH_COEFFICIENTS_HPP_INCLUDED
