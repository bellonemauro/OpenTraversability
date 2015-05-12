#  +---------------------------------------------------------------------------+
#  |                                         			                       |
#  |               https://sites.google.com/site/bellonemauro/                 |
#  |                                                                           |
#  | Copyright (c) 2015, - All rights reserved.                                |
#  | Authors: Mauro Bellone                                                    |
#  | Released under BDS License.                                               |
#  +---------------------------------------------------------------------------+ */

# Note: This CMake file is not standalone

project(VrepRemoteApi)   #define the project

# necessary to allow the *.cpp to find headers 
include_directories(${VrepRemoteApi_SOURCE_DIR}/include)

#select all headers in the include dir
FILE(GLOB LIB_HEADERS "${VrepRemoteApi_SOURCE_DIR}/include/*.h")   
#select all sources in the include dir
FILE(GLOB LIB_SOURCES "${VrepRemoteApi_SOURCE_DIR}/src/*.c")  

# add library 	
add_library(${PROJECT_NAME}  STATIC ${LIB_SOURCES} ${LIB_HEADERS} ) 
#TODO --- strange that must be static - does not work with shared
#TODO --- this have to be shared !!!

# set the postfix _d for debug libraries 
set_target_properties(${PROJECT_NAME}  PROPERTIES DEBUG_POSTFIX "_d" LINKER_LANGUAGE C)
# set the include directory to allow external apps to find the library
set (VrepRemoteApi_INCLUDE_DIR ${VrepRemoteApi_SOURCE_DIR}/include CACHE PATH "Path to library VrepRemoteApi")

add_definitions(-DVREPREMOTEAPI) 
add_definitions(-DMAX_EXT_API_CONNECTIONS=255)	
add_definitions(-DNON_MATLAB_PARSING)	
#target_link_libraries(${PROJECT_NAME}  ${PCL_LIBRARIES} ${Boost_LIBRARIES})
#link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARIES})

# allows folders for MSVC
if (MSVC AND ENABLE_SOLUTION_FOLDERS) 
	set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "libraries")
endif (MSVC AND ENABLE_SOLUTION_FOLDERS)