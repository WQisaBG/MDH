#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rmf_utils::rmf_utils" for configuration ""
set_property(TARGET rmf_utils::rmf_utils APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rmf_utils::rmf_utils PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librmf_utils.so"
  IMPORTED_SONAME_NOCONFIG "librmf_utils.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS rmf_utils::rmf_utils )
list(APPEND _IMPORT_CHECK_FILES_FOR_rmf_utils::rmf_utils "${_IMPORT_PREFIX}/lib/librmf_utils.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
