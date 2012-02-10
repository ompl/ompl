/*
 [auto_generated]
 boost/numeric/odeint/algebra/detail/reduce.hpp

 [begin_description]
 Default reduce implementation.
 [end_description]

 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */


#ifndef OMPLEXT_BOOST_NUMERIC_ODEINT_ALGEBRA_DETAIL_REDUCE_HPP_INCLUDED
#define OMPLEXT_BOOST_NUMERIC_ODEINT_ALGEBRA_DETAIL_REDUCE_HPP_INCLUDED


namespace boost {
namespace numeric {
namespace omplext_odeint {
namespace detail {

template< class ValueType , class Iterator1 , class Reduction >
inline ValueType reduce( Iterator1 first1 , Iterator1 last1 , Reduction red, ValueType init)
{
    for( ; first1 != last1 ; )
        init = red( init , *first1++ );
    return init;
}

} // detail
} // odeint
} // numeric
} // boost


#endif // BOOST_NUMERIC_ODEINT_ALGEBRA_DETAIL_REDUCE_HPP_INCLUDED
