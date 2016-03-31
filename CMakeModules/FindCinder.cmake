#.rst
# FindCinder
# ----------
#
# Created by Walter Gray
# Locate and configure cinder 0.8.5 for vc2010. Untested with other versions.
#
# Interface Targets
# ^^^^^^^^^^^^^^^^^
#   Cinder::Cinder
#     Interface target. references Cinder::Core and all
#	    required boost libraries with the required ordering.
#
#   Cinder::Core
#     Static import target. Does not include cinder's dependencies.
#     Typically not referenced outside of Cinder::Cinder
#
# Variables
# ^^^^^^^^^
#   Cinder_ROOT_DIR
#     Root of the cinder package
#   Cinder_FOUND
#     If false, do not link to Cinder
#   Cinder_LIBRARIES
#     The names of the libraries to feed into target_link_libraries
#   Cinder_INCLUDE_DIR
#     Where to find the root of the cinder includes
#   Cinder_LIBRARY_<CONFIG>
#     The location of the main cinder library
#
# This module will also find and add the Boost package which is included with Cinder,
# create a master Boost::Boost interface library which links to a series of 
# Boost::<component> static import libraries.

if(MSVC10)
  set(_compiler_SUFFIX "vc2010")
endif()

find_path(Cinder_ROOT_DIR
          NAMES include/cinder/Cinder.h
          PATH_SUFFIXES cinder_${Cinder_FIND_VERSION}_${_compiler_SUFFIX}
                        cinder_${Cinder_FIND_VERSION}
                        cinder)


find_path(Cinder_INCLUDE_DIR 
          NAMES "cinder/Cinder.h"
          HINTS "${Cinder_ROOT_DIR}"
          PATH_SUFFIXES "include")

find_library(Cinder_LIBRARY_RELEASE "cinder.lib" HINTS "${Cinder_ROOT_DIR}/lib")
find_library(Cinder_LIBRARY_DEBUG "cinder_d.lib" HINTS "${Cinder_ROOT_DIR}/lib")

set(Cinder_LIBRARIES
  debug ${Cinder_LIBRARY_DEBUG}
  optimized ${Cinder_LIBRARY_RELEASE})

find_package_handle_standard_args(Cinder DEFAULT_MSG Cinder_ROOT_DIR Cinder_LIBRARY_RELEASE Cinder_LIBRARY_DEBUG Cinder_INCLUDE_DIR)