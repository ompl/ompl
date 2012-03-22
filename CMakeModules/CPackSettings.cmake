set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "The Open Motion Planning Library (OMPL)")
set(CPACK_PACKAGE_VENDOR "Rice University")
set(CPACK_PACKAGE_CONTACT "Mark Moll <mmoll@rice.edu>")
set(CPACK_RSRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

set(CPACK_PACKAGE_VERSION_MAJOR "${OMPL_MAJOR_VERSION}")
set(CPACK_PACKAGE_VERSION_MINOR "${OMPL_MINOR_VERSION}")
set(CPACK_PACKAGE_VERSION_PATCH "${OMPL_PATCH_VERSION}")

set(CPACK_SOURCE_IGNORE_FILES
    "/.hg"
    "/.svn/"
    "/build/"
    ".pyc$"
    ".pyo$"
    "__pycache__"
    ".so$"
    ".dylib$"
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
  set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")
  set(CPACK_PACKAGE_FILE_NAME "ompl_${OMPL_VERSION}_${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "python${PYTHON_VERSION}, libboost-all-dev, libode-dev")
endif()

if(WIN32)
  set(CPACK_GENERATOR "ZIP;${CPACK_GENERATOR}")
endif()

include(CPack)

