# - Try to find srvlib
# Once done this will define
#  SRVLib_FOUND - System has SRVLib
#  SRVLib_INCLUDE_DIRS - The SRVLib include directories
#  SRVLib_LIBRARIES - The libraries needed to use SRVLib
#  SRVLib_DEFINITIONS - Compiler switches required for using SRVLib

find_path(SRVLib_INCLUDE_DIR srvlib/srvlib.hpp
          
          )

find_library(SRVLib_LIBRARY_RELEASE 
              NAMES srvlib
             )

find_library(SRVLib_LIBRARY_DEBUG 
              NAMES srvlib_debug
             )

SET(SRVLib_LIBRARY
  debug ${SRVLib_LIBRARY_DEBUG}
  optimized ${SRVLib_LIBRARY_RELEASE}
 )

             
include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(SRVLib  DEFAULT_MSG
                                  SRVLib_LIBRARY SRVLib_INCLUDE_DIR)

mark_as_advanced(SRVLib_INCLUDE_DIR SRVLib_LIBRARY )

set(SRVLib_LIBRARIES ${SRVLib_LIBRARY} )
set(SRVLib_INCLUDE_DIRS ${SRVLib_INCLUDE_DIR} )