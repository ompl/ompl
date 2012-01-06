include(FindPackageHandleStandardArgs)

include(ExternalProject)
# download Boost::Numeric::ODEInt
ExternalProject_Add(Boost.ODEInt
                    DOWNLOAD_DIR "${CMAKE_SOURCE_DIR}/src/external"
                    GIT_REPOSITORY "https://github.com/headmyshoulder/odeint-v2.git"
                    GIT_TAG "v2.1-54-g45a9b69" # Head revision as of 6 Jan 2012

                    CONFIGURE_COMMAND ""
                    UPDATE_COMMAND ""
                    BUILD_COMMAND ""
                    INSTALL_COMMAND "")

set(ODEINT_INCLUDE_DIR "${CMAKE_BINARY_DIR}/Boost.ODEInt-prefix/src/Boost.ODEInt/")
include_directories(BEFORE "${ODEINT_INCLUDE_DIR}")
