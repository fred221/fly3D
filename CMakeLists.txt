CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(OurRecast)

 
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
   message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

ADD_DEFINITIONS("-fPIC -g")
AUX_SOURCE_DIRECTORY("./OurRecast/Include" SRC)
AUX_SOURCE_DIRECTORY("./OurRecast/Source" SRC_SOURCE)
INCLUDE_DIRECTORIES("/usr/java/jdk1.8.0_192-amd64/include")
INCLUDE_DIRECTORIES("/usr/java/jdk1.8.0_192-amd64/include/linux")

INCLUDE_DIRECTORIES("./Recast/Include")
INCLUDE_DIRECTORIES("./DebugUtils/Include")
INCLUDE_DIRECTORIES("./Detour/Include")
INCLUDE_DIRECTORIES("./DetourCrowd/Include")
INCLUDE_DIRECTORIES("./DetourTileCache/Include")
INCLUDE_DIRECTORIES("./OurRecast/Include")

#include_directories("Include")

ADD_SUBDIRECTORY(Recast)
ADD_SUBDIRECTORY(DebugUtils)
ADD_SUBDIRECTORY(Detour)
ADD_SUBDIRECTORY(DetourCrowd)
ADD_SUBDIRECTORY(DetourTileCache)


ADD_LIBRARY(OurRecast SHARED ${SRC} ${SRC_SOURCE} )
#add_dependencies(Recast)
#add_dependencies(DebugUtils Detour DetourCrowd DetourTileCache Recast)
target_link_libraries(OurRecast DebugUtils Detour DetourCrowd DetourTileCache Recast)
#target_link_libraries(Recast)

#TARGET_LINK_LIBRARIES(recast )
