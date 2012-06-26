/*
 [auto_generated]
 boost/numeric/odeint/integrate/null_observer.hpp

 [begin_description]
 null_observer
 [end_description]

 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#ifndef OMPLEXT_BOOST_NUMERIC_ODEINT_INTEGRATE_NULL_OBSERVER_HPP_INCLUDED
#define OMPLEXT_BOOST_NUMERIC_ODEINT_INTEGRATE_NULL_OBSERVER_HPP_INCLUDED

namespace boost {
namespace numeric {
namespace omplext_odeint {

struct null_observer
{
    template< class State , class Time >
    void operator()( const State& /* x */ , Time /* t */ ) const
    {

    }
};

} // namespace omplext_odeint
} // namespace numeric
} // namespace boost

#endif // BOOST_NUMERIC_ODEINT_INTEGRATE_NULL_OBSERVER_HPP_INCLUDED
