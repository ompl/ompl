/*
 [auto_generated]
 boost/numeric/odeint/version.hpp

 [begin_description]
 Defines the current version of odeint.
 [end_description]

 Copyright 2009-2011 Karsten Ahnert
 Copyright 2009-2011 Mario Mulansky

 Distributed under the Boost Software License, Version 1.0.
 (See accompanying file LICENSE_1_0.txt or
 copy at http://www.boost.org/LICENSE_1_0.txt)
 */

#ifndef BOOST_NUMERIC_ODEINT_VERSION_HPP_INCLUDED
#define BOOST_NUMERIC_ODEINT_VERSION_HPP_INCLUDED

#include <string>
#include <sstream>

namespace boost {
namespace numeric {
namespace odeint {

namespace version {

const int major = 2;
const int minor = 1;

}

inline std::string get_version_string( void )
{
    std::ostringstream str;
    str << "v" << version::major << "." << version::minor;
    return str.str();
}


}
}
}

#endif // BOOST_NUMERIC_ODEINT_VERSION_HPP_INCLUDED
