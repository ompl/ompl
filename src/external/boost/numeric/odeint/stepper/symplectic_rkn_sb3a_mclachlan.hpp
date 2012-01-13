/*
 [auto_generated]
 boost/numeric/odeint/stepper/symplectic_rkn_sb3a_mclachlan.hpp
 
 [begin_description]
 Implementation of the symplectic MacLachlan stepper for separable Hamiltonian system.
 [end_description]
 
 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky
 
 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
*/


#ifndef BOOST_NUMERIC_ODEINT_STEPPER_SYMPLECTIC_RKN_SB3A_MCLACHLAN_HPP_INCLUDED
#define BOOST_NUMERIC_ODEINT_STEPPER_SYMPLECTIC_RKN_SB3A_MCLACHLAN_HPP_INCLUDED


#include <boost/numeric/odeint/stepper/base/symplectic_rkn_stepper_base.hpp>

#include <boost/numeric/odeint/algebra/range_algebra.hpp>
#include <boost/numeric/odeint/algebra/default_operations.hpp>

#include <boost/numeric/odeint/stepper/detail/macros.hpp>

#include <boost/array.hpp>

namespace boost {
namespace numeric {
namespace odeint {

namespace detail {
namespace symplectic_rkn_sb3a_mclachlan {

/*
	rk_a[0]=0.40518861839525227722;
	rk_a[1]=-0.28714404081652408900;
	rk_a[2]=0.5-(rk_a[0]+rk_a[1]);
	rk_a[3]=rk_a[2];
	rk_a[4]=rk_a[1];
	rk_a[5]=rk_a[0];

	rk_b[0]=-3.0/73.0;
	rk_b[1]=17.0/59.0;
	rk_b[2]=1.0-2.0*(rk_b[0]+rk_b[1]);
	rk_b[3]=rk_b[1];
	rk_b[4]=rk_b[0];
	rk_b[5]=0.0;
*/


	template< class Value >
	struct coef_a_type : public boost::array< Value , 6 >
	{
		coef_a_type( void )
		{
			(*this)[0] = 0.40518861839525227722;
			(*this)[1] = -0.28714404081652408900;
			(*this)[2] = 0.5 - ( (*this)[0] + (*this)[1] );
			(*this)[3] = (*this)[2];
			(*this)[4] = (*this)[1];
			(*this)[5] = (*this)[0];

		}
	};

	template< class Value >
	struct coef_b_type : public boost::array< Value , 6 >
	{
		coef_b_type( void )
		{
			(*this)[0] = -3.0 / 73.0;
			(*this)[1] = 17.0 / 59.0;
			(*this)[2] = 1.0 - 2.0 * ( (*this)[0] + (*this)[1] );
			(*this)[3] = (*this)[1];
			(*this)[4] = (*this)[0];
			(*this)[5] = 0.0;
		}
	};

} // namespace symplectic_rkn_sb3a_mclachlan
} // namespace detail



template<
	class Coor ,
	class Momentum = Coor ,
	class Value = double ,
	class CoorDeriv = Coor ,
	class MomentumDeriv = Coor ,
	class Time = Value ,
	class Algebra = range_algebra ,
	class Operations = default_operations ,
	class Resizer = initially_resizer
	>
class symplectic_rkn_sb3a_mclachlan :
	public symplectic_nystroem_stepper_base
	<
		6 ,
		symplectic_rkn_sb3a_mclachlan< Coor , Momentum , Value , CoorDeriv , MomentumDeriv , Time , Algebra , Operations , Resizer > ,
		Coor , Momentum , Value , CoorDeriv , MomentumDeriv , Time , Algebra , Operations , Resizer
	>
{
public:

    BOOST_ODEINT_SYMPLECTIC_NYSTROEM_STEPPER_TYPEDEFS( symplectic_rkn_sb3a_mclachlan , 6 );

	symplectic_rkn_sb3a_mclachlan( const algebra_type &algebra = algebra_type() )
        : stepper_base_type(
            detail::symplectic_rkn_sb3a_mclachlan::coef_a_type< value_type >() ,
            detail::symplectic_rkn_sb3a_mclachlan::coef_b_type< value_type >() ,
            algebra )
    { }


    symplectic_rkn_sb3a_mclachlan( const symplectic_rkn_sb3a_mclachlan &stepper )
        : stepper_base_type( stepper )
    { }

    symplectic_rkn_sb3a_mclachlan& operator = ( const symplectic_rkn_sb3a_mclachlan &stepper )
    {
        stepper_base_type::operator=( stepper );
        return *this;
    }

};



} // namespace odeint
} // namespace numeric
} // namespace boost

#endif // BOOST_NUMERIC_ODEINT_STEPPER_SYMPLECTIC_RKN_SB3A_MCLACHLAN_HPP_INCLUDED
