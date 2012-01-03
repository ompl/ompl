include(FindPackageHandleStandardArgs)

include(ExternalProject)
# download Boost::Numeric::ODEInt
ExternalProject_Add(Boost.ODEInt
                    DOWNLOAD_DIR "${CMAKE_SOURCE_DIR}/src/external"
                    SVN_REPOSITORY "http://svn.boost.org/svn/boost/sandbox/odeint/"
                    SVN_REVISION "-r75863"

                    # Version 2.  Not compatible with v1.  Requires Boost 1.43
                    # GIT_REPOSITORY "https://github.com/headmyshoulder/odeint-v2.git"
                    CONFIGURE_COMMAND ""
                    UPDATE_COMMAND ""
                    BUILD_COMMAND ""
                    INSTALL_COMMAND "")

set(ODEINT_INCLUDE_DIR "${CMAKE_BINARY_DIR}/Boost.ODEInt-prefix/src/Boost.ODEInt/")
include_directories(BEFORE "${ODEINT_INCLUDE_DIR}")
