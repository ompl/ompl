add_subdirectory(ompl)
install(DIRECTORY ompl/ DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/ompl${OMPL_INSTALL_SUFFIX}"
    COMPONENT ompl
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".DS_Store" EXCLUDE
    PATTERN "sample_contrib" EXCLUDE
    REGEX "/src$" EXCLUDE
    REGEX "/doc$" EXCLUDE
    REGEX "/tests$" EXCLUDE)

if(${Boost_VERSION} LESS 105300)
    install(DIRECTORY external/omplext_odeint/
        DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/omplext_odeint${OMPL_INSTALL_SUFFIX}"
        COMPONENT ompl)
endif()

find_program(CURL curl)
if(CURL)
    set(DOWNLOAD_CMD "${CURL} --location-trusted")
else()
    find_program(WGET wget)
    if (WGET)
        set(DOWNLOAD_CMD "${WGET} --no-check-certificate -O -")
    else()
        message(WARNING "Neither curl nor wget is installed. Install either one of those programs for \"make installpyplusplus\" to succeed.")
    endif()
endif()

# Add target to install Py++ and its dependencies pygccxml and gccxml
if (APPLE)
    # need to compile gccxml with llvm-gcc-4.2 instead of clang
    set(CMAKE_GCCXML_ARGS "${CMAKE_GCCXML_ARGS} -DCMAKE_C_COMPILER=llvm-gcc-4.2 -DCMAKE_CXX_COMPILER=llvm-g++-4.2")
endif()
if(WIN32)
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/external/installPyPlusPlus.bat.in"
        "${CMAKE_CURRENT_SOURCE_DIR}/external/installPyPlusPlus.bat" @ONLY)
    add_custom_target(installpyplusplus
        COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/external/installPyPlusPlus.bat" VERBATIM)
else(WIN32)
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/external/installPyPlusPlus.sh.in"
        "${CMAKE_CURRENT_SOURCE_DIR}/external/installPyPlusPlus.sh" @ONLY)
    add_custom_target(installpyplusplus
        COMMAND "sh" "${CMAKE_CURRENT_SOURCE_DIR}/external/installPyPlusPlus.sh")
endif(WIN32)
