#  +---------------------------------------------------------------------------+
#  |                                         			                       |
#  |               https://sites.google.com/site/bellonemauro/                 |
#  |                                                                           |
#  | Copyright (c) 2015, - All rights reserved.                                |
#  | Authors: Mauro Bellone                                                    |
#  | Released under BDS License.                                               |
#  +---------------------------------------------------------------------------+ */

# Note: This CMake file is not standalone, it defines the library udp


project(upd_lib)   #define the project

# pcl is mandatory but it should be already found from the main cmake and boost as well 
include_directories(${PCL_INCLUDE_DIRS})  
include_directories(${Boost_LIBRARY_DIR})
# necessary to allow the *.cpp to find headers 
include_directories(${upd_lib_SOURCE_DIR}/include)

#select all headers in the include dir
FILE(GLOB UPD_HEADERS "${upd_lib_SOURCE_DIR}/include/*.h")   
#select all sources in the include dir
FILE(GLOB UPD_SOURCES "${upd_lib_SOURCE_DIR}/src/*.cpp")  

# add library 
add_library(upd_lib SHARED ${UPD_SOURCES} ${UPD_HEADERS} )

# set the postfix _d for debug libraries 
set_target_properties(upd_lib PROPERTIES DEBUG_POSTFIX "_d")

add_definitions(-DUPD_LIBRARY) 
	
target_link_libraries(upd_lib ${PCL_LIBRARIES} ${Boost_LIBRARIES})
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARIES})