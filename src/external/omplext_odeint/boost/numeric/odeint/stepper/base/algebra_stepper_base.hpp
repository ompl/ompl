/*
 [auto_generated]
 boost/numeric/odeint/stepper/base/algebra_stepper_base.hpp

 [begin_description]
 Base class for all steppers with an algebra and operations.
 [end_description]

 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#ifndef OMPLEXT_BOOST_NUMERIC_ODEINT_STEPPER_BASE_ALGEBRA_STEPPER_BASE_HPP_INCLUDED
#define OMPLEXT_BOOST_NUMERIC_ODEINT_STEPPER_BASE_ALGEBRA_STEPPER_BASE_HPP_INCLUDED


namespace boost {
namespace numeric {
namespace omplext_odeint {


template< class Algebra , class Operations >
class algebra_stepper_base
{
public:

    typedef Algebra algebra_type;
    typedef Operations operations_type;

    algebra_stepper_base( const algebra_type &algebra = algebra_type() )
    : m_algebra( algebra ) { }

    algebra_type& algebra()
    {
        return m_algebra;
    }

    const algebra_type& algebra() const
    {
        return m_algebra;
    }


protected:

    algebra_type m_algebra;
};

} // odeint
} // numeric
} // boost


#endif // BOOST_NUMERIC_ODEINT_STEPPER_BASE_ALGEBRA_STEPPER_BASE_HPP_INCLUDED
