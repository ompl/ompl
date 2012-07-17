set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "The Open Motion Planning Library (OMPL)")
set(CPACK_PACKAGE_VENDOR "Rice University")
set(CPACK_PACKAGE_CONTACT "Mark Moll <mmoll@rice.edu>")
set(CPACK_RSRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

set(CPACK_PACKAGE_VERSION_MAJOR "${OMPL_MAJOR_VERSION}")
set(CPACK_PACKAGE_VERSION_MINOR "${OMPL_MINOR_VERSION}")
set(CPACK_PACKAGE_VERSION_PATCH "${OMPL_PATCH_VERSION}")

set(CPACK_SOURCE_IGNORE_FILES
    "/.hg"
    "/build/"
    ".pyc$"
    ".pyo$"
    "__pycache__"
    ".so$"
    ".dylib$"
    ".orig$"
    ".DS_Store"
    ".tm_properties"
    "mkwebdocs.sh"
    "/html/"
    "/bindings/"
    "TODO"
    "exposed_decl.pypp.txt"
    "ompl.pc$"
    "installPyPlusPlus.bat$"
    "installPyPlusPlus.sh$"
    "config.h$")
set(CPACK_SOURCE_GENERATOR "TGZ;ZIP")
set(CPACK_GENERATOR "TGZ")

if(${CMAKE_SYSTEM_NAME} STREQUAL "Linux")
    set(CPACK_GENERATOR "DEB;${CPACK_GENERATOR}")
    if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "i686")
        set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "i386")
    endif()
    if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
        set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
    endif()
    execute_process(COMMAND "/usr/bin/lsb_release" "-rs"
        OUTPUT_VARIABLE UBUNTU_RELEASE
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    set(CPACK_PACKAGE_FILE_NAME "omplapp_${OMPL_VERSION}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}-Ubuntu${UBUNTU_RELEASE}")
    set(CPACK_DEBIAN_PACKAGE_DEPENDS "python${PYTHON_VERSION}, libboost-all-dev, libode-dev")
    if(OMPL_VERSIONED_INSTALL)
      set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA "${CMAKE_SOURCE_DIR}/CMakeModules/postinst;${CMAKE_SOURCE_DIR}/CMakeModules/prerm;")
    endif()
endif()

if(WIN32)
  set(CPACK_GENERATOR "ZIP;${CPACK_GENERATOR}")
endif()

include(CPack)

