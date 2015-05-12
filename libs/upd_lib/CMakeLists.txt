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
FILE(GLOB LIB_HEADERS "${upd_lib_SOURCE_DIR}/include/*.h")   
#select all sources in the include dir
FILE(GLOB LIB_SOURCES "${upd_lib_SOURCE_DIR}/src/*.cpp")  

# add library 
add_library(${PROJECT_NAME}  SHARED ${LIB_SOURCES} ${LIB_HEADERS} )

# set the postfix _d for debug libraries 
set_target_properties(${PROJECT_NAME}  PROPERTIES DEBUG_POSTFIX "_d")
set (upd_lib_INCLUDE_DIR ${upd_lib_SOURCE_DIR}/include CACHE PATH "Path to the headers of upd_lib")

add_definitions(-DUPD_LIBRARY) 
	
target_link_libraries(${PROJECT_NAME}  ${PCL_LIBRARIES} ${Boost_LIBRARIES})
link_directories(${PCL_LIBRARY_DIRS} ${Boost_LIBRARIES})

# allows folders for MSVC
if (MSVC AND ENABLE_SOLUTION_FOLDERS) 
	set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "libraries")
endif (MSVC AND ENABLE_SOLUTION_FOLDERS)