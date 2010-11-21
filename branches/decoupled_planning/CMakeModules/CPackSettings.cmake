set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "The Open Motion Planning Library (OMPL)")
set(CPACK_PACKAGE_VENDOR "Rice University")
set(CPACK_PACKAGE_CONTACT "Mark Moll <mmoll@rice.edu>")
set(CPACK_RSRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules")

set(CPACK_PACKAGE_VERSION_MAJOR "${OMPL_MAJOR_VERSION}")
set(CPACK_PACKAGE_VERSION_MINOR "${OMPL_MINOR_VERSION}")
set(CPACK_PACKAGE_VERSION_PATCH "${OMPL_PATCH_VERSION}")

set(CPACK_SOURCE_IGNORE_FILES 
	"$/CVS/;/.svn/;.swp$;.#;/#;/build/;.pyc$;.pyo$;.so$;.md5$;/blueprint/;.DS_Store;_cache$;mkwebdocs.sh;TODO;bindings")
set(CPACK_SOURCE_GENERATOR "TGZ;ZIP")
set(CPACK_GENERATOR "TGZ;ZIP")

include(CPack)

