#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "esp32_comm::esp32_comm" for configuration ""
set_property(TARGET esp32_comm::esp32_comm APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(esp32_comm::esp32_comm PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libesp32_comm.so"
  IMPORTED_SONAME_NOCONFIG "libesp32_comm.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS esp32_comm::esp32_comm )
list(APPEND _IMPORT_CHECK_FILES_FOR_esp32_comm::esp32_comm "${_IMPORT_PREFIX}/lib/libesp32_comm.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
