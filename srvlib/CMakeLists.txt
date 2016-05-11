macro( append_glob list glob )
    file(GLOB files ${glob})
    set(${list} "${${list}};${files}")
endmacro()

set( INCDIR "../include/srvlib")

list(APPEND LIB_INC_DIR  "${PROJECT_SOURCE_DIR}/include;${CMAKE_CURRENT_BINARY_DIR}/include" )


set( HEADERS ${INCDIR}/srvlib.hpp )
append_glob(HEADERS ${INCDIR}/model/*.h*)
append_glob(HEADERS ${INCDIR}/view/*.h*)
append_glob(HEADERS ${INCDIR}/io/*.h*)
append_glob(HEADERS ${INCDIR}/io/pose/*.h*)
append_glob(HEADERS ${INCDIR}/io/video/*.h*)
append_glob(HEADERS ${INCDIR}/utils/*.h*)

append_glob(SOURCES model/*.cpp)
append_glob(SOURCES view/*.cpp)
append_glob(SOURCES io/*.cpp)
append_glob(SOURCES io/pose/*.cpp)
append_glob(SOURCES io/video/*.cpp)
append_glob(SOURCES utils/*.cpp)

#####################################################
# user options

option(BUILD_DA_VINCI "Build support for Da Vinci Instruments" ON)
if(BUILD_DA_VINCI)
  
  add_definitions(-DUSE_DA_VINCI)
  append_glob( HEADERS ${INCDIR}/model/davinci/*.h*)
  append_glob( SOURCES model/davinci/*.cpp)

endif()

if(_LINUX_)
  set (CMAKE_CXX_FLAGS "-std=c++11")
endif()

if(_LINUX_)

  option(BUILD_ROS "Build ROS support for Da Vinci" ON)
  if(BUILD_ROS)
    
    add_definitions(-DUSE_ROS)
    find_package(roscpp REQUIRED) 
    include_directories(${roscpp_INCLUDE_DIRS})
    list(APPEND LINK_LIBS "${roscpp_LIBRARIES}" )

  endif()

else(_WIN_)
  
  if( CMAKE_SIZEOF_VOID_P EQUAL 8 ) #ISI api only supported 64 bit
   
    option(BUILD_ISI "Build ISI API support for Da Vinci" ON)
    if(BUILD_ISI)
   
      add_definitions(-DUSE_ISI_API)
      #include_directories("C:/Program Files (x86)/ISI API 1.0/include/")
      #list(APPEND LINK_LIBS "C:/Program Files (x86)/ISI API 1.0/lib/isi_api.lib" )
      find_package(isi_api REQUIRED)
      if(isi_api_FOUND)
        include_directories("${isi_api_INCLUDE_DIR}")
        list(APPEND LINK_LIBS "${isi_api_LIBRARIES}")
      endif()
    
    endif() 
  
  endif( CMAKE_SIZEOF_VOID_P EQUAL 8 )

# include intutiive api stuff

endif()

option(BUILD_SDI_IO "Build support for SDI IO" OFF)
if(BUILD_SDI_IO)
  
  add_definitions(-DUSE_SDI_IO)
  append_glob( HEADERS ${INCDIR}/io/video/sdi/*.h*)
  append_glob( SOURCES io/video/sdi/*.cpp)
  
  #hack for now
  if (_LINUX_)
    list(APPEND LIB_INC_DIR "/home/davinci/sdks/BlackMagic/Blackmagic_Decklink_SDK_10.3.1/Linux/include")
    list(APPEND LIB_INC_DIR "/home/davinci/sdks/LeapSDK_2.2.2/LeapSDK/include/")
    list(APPEND LINK_LIBS "/home/davinci/sdks/LeapSDK_2.2.2/LeapSDK/lib/x64/libLeap.so")
    find_package(roscpp REQUIRED)
    include_directories(${roscpp_INCLUDE_DIRS})

  else(_WIN_)

    list(APPEND HEADERS ${INCDIR}/io/video/sdi/DeckLinkAPI.h)
    list(APPEND SOURCES io/video/sdi/DeckLinkAPI_i.c)
    
  endif()
  
endif()


#######################################################

#Boost
set(Boost_USE_STATIC_LIBS ON) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF)
#set(Boost_BOOST_ALL_DYN_LINK)
find_package(Boost REQUIRED COMPONENTS regex chrono date_time system filesystem thread python)
if(Boost_FOUND)
    ADD_DEFINITIONS("-DBOOST_ALL_NO_LIB")
    list(APPEND USER_INC "${Boost_INCLUDE_DIRS}")
    list(APPEND LINK_LIBS "${Boost_LIBRARIES}")
endif()

#Opencv
find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
  message("Found OpenCV")
  list(APPEND USER_INC "${OpenCV_INCLUDE_DIRS}" )
  list(APPEND LINK_LIBS "${OpenCV_LIBRARIES}" )
endif()

#Cinder

find_package(Cinder REQUIRED)
if(Cinder_FOUND)
  message("Found Cinder")
  list(APPEND USER_INC "${Cinder_INCLUDE_DIR}" )
  list(APPEND LINK_LIBS "${Cinder_LIBRARIES}" )
endif()

#######################################################
## Add Libraries / Include Directories / Link directories

include_directories( ${LIB_INC_DIR} )
include_directories( ${USER_INC} )
link_directories( ${USER_LIB} )

add_library(${LIBRARY_NAME} ${SOURCES} ${HEADERS} )

target_link_libraries(${LIBRARY_NAME} ${LINK_LIBS})



