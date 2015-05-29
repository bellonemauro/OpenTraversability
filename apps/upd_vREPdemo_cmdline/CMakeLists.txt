#  +---------------------------------------------------------------------------+
#  |                                                                           |
#  |               https://sites.google.com/site/bellonemauro/                 |
#  |                                                                           |
#  | Copyright (c) 2015, - All rights reserved.                                |
#  | Authors: Mauro Bellone                                                    |
#  | Released under BDS License.                                               |
#  +---------------------------------------------------------------------------+ */

project(upd_vREPdemo_cmdline)


# --------------------------------------
#  including external libraries
# --------------------------------------

# pcl is mandatory but it should be already found from the main cmake and boost as well 
include_directories(${PCL_INCLUDE_DIRS})  
include_directories(${Boost_LIBRARY_DIR})
include_directories(${Boost_INCLUDE_DIRS})
# necessary to allow the *.cpp to find headers 
include_directories(${upd_lib_INCLUDE_DIR})
include_directories(${upd_vREPdemo_cmdline_SOURCE_DIR}/include) 
include_directories(${VrepRemoteApi_INCLUDE_DIR})
message(STATUS " including VrepRemoteApi_BINARY_DIR  :: ${VrepRemoteApi_BINARY_DIR}" )

# --------------------------------------
#  set variables for compilation 
# --------------------------------------

FILE(GLOB upd_vREPdemo_SOURCES "${upd_vREPdemo_cmdline_SOURCE_DIR}/*.cpp") 
#select all headers in the include dir
FILE(GLOB upd_vREPdemo_HEADERS "${upd_vREPdemo_cmdline_SOURCE_DIR}/include/*.h")   
#select all sources in the include dir




link_directories(${PCL_LIBRARY_DIRS})  #link external libraries

add_definitions (${PCL_DEFINITIONS})  #add definitions:: required to use PCL

add_executable (upd_vREPdemo_cmdline  	${upd_vREPdemo_SOURCES} ${upd_vREPdemo_HEADERS})


target_link_libraries ( upd_vREPdemo_cmdline  upd_lib  VrepRemoteApi
						${Boost_LIBRARIES}
						${PCL_LIBRARIES} )
message (STATUS "VrepRemoteApi_BINARY_DIR :: ${VrepRemoteApi_LIBRARY}" )
message (STATUS "upd_lib :: ${upd_lib}" )
message (STATUS "PCL_LIBRARIES :: ${PCL_LIBRARIES}" )
message (STATUS "Boost_LIBRARIES :: ${Boost_LIBRARIES}" )
						
# allows folders for MSVC
if (MSVC AND ENABLE_SOLUTION_FOLDERS) 
	set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Apps")
endif (MSVC AND ENABLE_SOLUTION_FOLDERS)	
					
