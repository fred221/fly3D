CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(OurRecast)

ADD_DEFINITIONS("-fPIC -g")
AUX_SOURCE_DIRECTORY("./" SRC)
AUX_SOURCE_DIRECTORY("./Source" SRC_SOURCE)

INCLUDE_DIRECTORIES("/usr/java/jdk1.8.0_192-amd64/include")
INCLUDE_DIRECTORIES("/usr/java/jdk1.8.0_192-amd64/include/linux")

#INCLUDE_DIRECTORIES("./DebugUtils")
#INCLUDE_DIRECTORIES("./Detour")
#INCLUDE_DIRECTORIES("./DetourCrowd")
#INCLUDE_DIRECTORIES("./DetourTileCache")
INCLUDE_DIRECTORIES("../Recast")
#INCLUDE_DIRECTORIES("./OurRecast")

#include_directories("Include")

#ADD_SUBDIRECTORY(DebugUtils)
#ADD_SUBDIRECTORY(Detour)
#ADD_SUBDIRECTORY(DetourCrowd)
#ADD_SUBDIRECTORY(DetourTileCache)
ADD_SUBDIRECTORY(Recast)

add_dependencies(Recast)
#add_dependencies(DebugUtils Detour DetourCrowd DetourTileCache Recast)
#target_link_libraries(OurRecast DebugUtils Detour DetourCrowd DetourTileCache Recast)
target_link_libraries(OurRecast  Recast)

#ADD_LIBRARY(OurRecast SHARED ${SRC} ${SRC_SOURCE})
#TARGET_LINK_LIBRARIES(recast )