/*
 [auto_generated]
 boost/numeric/odeint/stepper/detail/adams_moulton_coefficients.hpp

 [begin_description]
 Coefficients for the Adams Moulton method.
 [end_description]

 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#ifndef BOOST_NUMERIC_ODEINT_STEPPER_DETAIL_ADAMS_MOULTON_COEFFICIENTS_HPP_INCLUDED
#define BOOST_NUMERIC_ODEINT_STEPPER_DETAIL_ADAMS_MOULTON_COEFFICIENTS_HPP_INCLUDED


#include <boost/array.hpp>


namespace boost {
namespace numeric {
namespace odeint {
namespace detail {

template< class Value , size_t Steps >
class adams_moulton_coefficients ;

template< class Value >
class adams_moulton_coefficients< Value , 1 > : public boost::array< Value , 1 >
{
public:
    adams_moulton_coefficients( void )
    : boost::array< Value , 1 >()
      {
        (*this)[0] = static_cast< Value >( 1.0 );
      }
};


template< class Value >
class adams_moulton_coefficients< Value , 2 > : public boost::array< Value , 2 >
{
public:
    adams_moulton_coefficients( void )
    : boost::array< Value , 2 >()
      {
        (*this)[0] = static_cast< Value >( 1.0 ) / static_cast< Value >( 2.0 );
        (*this)[1] = static_cast< Value >( 1.0 ) / static_cast< Value >( 2.0 );
      }
};


template< class Value >
class adams_moulton_coefficients< Value , 3 > : public boost::array< Value , 3 >
{
public:
    adams_moulton_coefficients( void )
    : boost::array< Value , 3 >()
      {
        (*this)[0] = static_cast< Value >( 5.0 ) / static_cast< Value >( 12.0 );
        (*this)[1] = static_cast< Value >( 2.0 ) / static_cast< Value >( 3.0 );
        (*this)[2] = -static_cast< Value >( 1.0 ) / static_cast< Value >( 12.0 );
      }
};


template< class Value >
class adams_moulton_coefficients< Value , 4 > : public boost::array< Value , 4 >
{
public:
    adams_moulton_coefficients( void )
    : boost::array< Value , 4 >()
      {
        (*this)[0] = static_cast< Value >( 3.0 ) / static_cast< Value >( 8.0 );
        (*this)[1] = static_cast< Value >( 19.0 ) / static_cast< Value >( 24.0 );
        (*this)[2] = -static_cast< Value >( 5.0 ) / static_cast< Value >( 24.0 );
        (*this)[3] = static_cast< Value >( 1.0 ) / static_cast< Value >( 24.0 );
      }
};


template< class Value >
class adams_moulton_coefficients< Value , 5 > : public boost::array< Value , 5 >
{
public:
    adams_moulton_coefficients( void )
    : boost::array< Value , 5 >()
      {
        (*this)[0] = static_cast< Value >( 251.0 ) / static_cast< Value >( 720.0 );
        (*this)[1] = static_cast< Value >( 323.0 ) / static_cast< Value >( 360.0 );
        (*this)[2] = -static_cast< Value >( 11.0 ) / static_cast< Value >( 30.0 );
        (*this)[3] = static_cast< Value >( 53.0 ) / static_cast< Value >( 360.0 );
        (*this)[4] = -static_cast< Value >( 19.0 ) / static_cast< Value >( 720.0 );
      }
};


template< class Value >
class adams_moulton_coefficients< Value , 6 > : public boost::array< Value , 6 >
{
public:
    adams_moulton_coefficients( void )
    : boost::array< Value , 6 >()
      {
        (*this)[0] = static_cast< Value >( 95.0 ) / static_cast< Value >( 288.0 );
        (*this)[1] = static_cast< Value >( 1427.0 ) / static_cast< Value >( 1440.0 );
        (*this)[2] = -static_cast< Value >( 133.0 ) / static_cast< Value >( 240.0 );
        (*this)[3] = static_cast< Value >( 241.0 ) / static_cast< Value >( 720.0 );
        (*this)[4] = -static_cast< Value >( 173.0 ) / static_cast< Value >( 1440.0 );
        (*this)[5] = static_cast< Value >( 3.0 ) / static_cast< Value >( 160.0 );
      }
};

template< class Value >
class adams_moulton_coefficients< Value , 7 > : public boost::array< Value , 7 >
{
public:
    adams_moulton_coefficients( void )
    : boost::array< Value , 7 >()
      {
        (*this)[0] = static_cast< Value >( 19087.0 ) / static_cast< Value >( 60480.0 );
        (*this)[1] = static_cast< Value >( 2713.0 ) / static_cast< Value >( 2520.0 );
        (*this)[2] = -static_cast< Value >( 15487.0 ) / static_cast< Value >( 20160.0 );
        (*this)[3] = static_cast< Value >( 586.0 ) / static_cast< Value >( 945.0 );
        (*this)[4] = -static_cast< Value >( 6737.0 ) / static_cast< Value >( 20160.0 );
        (*this)[5] = static_cast< Value >( 263.0 ) / static_cast< Value >( 2520.0 );
        (*this)[6] = -static_cast< Value >( 863.0 ) / static_cast< Value >( 60480.0 );
      }
};


template< class Value >
class adams_moulton_coefficients< Value , 8 > : public boost::array< Value , 8 >
{
public:
    adams_moulton_coefficients( void )
    : boost::array< Value , 8 >()
      {
        (*this)[0] = static_cast< Value >( 5257.0 ) / static_cast< Value >( 17280.0 );
        (*this)[1] = static_cast< Value >( 139849.0 ) / static_cast< Value >( 120960.0 );
        (*this)[2] = -static_cast< Value >( 4511.0 ) / static_cast< Value >( 4480.0 );
        (*this)[3] = static_cast< Value >( 123133.0 ) / static_cast< Value >( 120960.0 );
        (*this)[4] = -static_cast< Value >( 88547.0 ) / static_cast< Value >( 120960.0 );
        (*this)[5] = static_cast< Value >( 1537.0 ) / static_cast< Value >( 4480.0 );
        (*this)[6] = -static_cast< Value >( 11351.0 ) / static_cast< Value >( 120960.0 );
        (*this)[7] = static_cast< Value >( 275.0 ) / static_cast< Value >( 24192.0 );
      }
};







} // detail
} // odeint
} // numeric
} // boost



#endif // BOOST_NUMERIC_ODEINT_STEPPER_DETAIL_ADAMS_MOULTON_COEFFICIENTS_HPP_INCLUDED
