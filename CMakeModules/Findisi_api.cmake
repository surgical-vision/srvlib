#
# $Id: $
#
# Author(s):  Anton Deguet
# Created on: 2010-07-08
#
# (C) Copyright 2010 Johns Hopkins University (JHU), All Rights
# Reserved.
#
# Find the isi_api includes and library
#
# This module defines
# isi_api_INCLUDE_DIR, where to find headers
# isi_api_LIBRARIES, the libraries to link against to use isi_api.
# isi_api_FOUND, If false, do not try to use asi_api.

# This file is assumed to be used for building the installed examples
# and will only work when maintained relative to the rest of the tree

find_path (isi_api_INCLUDE_DIR
           NAMES isi_api.h
           PATHS "${CMAKE_SOURCE_DIR}/../include"
)

find_library (isi_api_LIBRARY 
              NAMES isi_api
              PATHS "${CMAKE_SOURCE_DIR}/../lib"
)

if (isi_api_INCLUDE_DIR)
    if (isi_api_LIBRARY)
        set (isi_api_FOUND "YES")
            if (WIN32)
                set (isi_api_LIBRARIES ${isi_api_LIBRARY} ws2_32 iphlpapi)
	    else (WIN32)
                find_package(Threads REQUIRED)
                set (isi_api_LIBRARIES ${isi_api_LIBRARY} ${CMAKE_THREAD_LIBS_INIT})
	endif (WIN32)
    endif (isi_api_LIBRARY)
endif (isi_api_INCLUDE_DIR)

