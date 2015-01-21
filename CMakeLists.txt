#  +---------------------------------------------------------------------------+
#  |                              			                                   |
#  |               https://sites.google.com/site/bellonemauro/                 |
#  |                                                                           |
#  | Copyright (c) 2015, - All rights reserved.                                |
#  | Authors: Mauro Bellone                                                    |
#  | Released under BDS License.                                               |
#  +---------------------------------------------------------------------------+ 

project(OTA)   # OpenTraversabilityApplications - OTA

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)


#cmake tweaks
if(COMMAND cmake_policy) #return true if the COMMAND cmake_policy exist - this allows compatibility with older versions
	# We don't want to mix relative and absolute paths in linker lib lists.
	cmake_policy(SET CMP0003 NEW) # Required by CMake 2.7+
	
	# avoid link warnings - see http://www.cmake.org/cmake/help/v3.0/policy/CMP0015.html
	cmake_policy(SET CMP0015 NEW) 

	# Allow using the LOCATION target property.
	if(NOT "${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 3.0)
		cmake_policy(SET CMP0026 OLD)
	endif(NOT "${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}" LESS 3.0)
endif(COMMAND cmake_policy)

include(${PROJECT_SOURCE_DIR}/CMakeModules/script_init_cmake.cmake REQUIRED)		# CMake configurations
include(${PROJECT_SOURCE_DIR}/CMakeModules/UtilsMacros.cmake REQUIRED)		        # CMake macros 

set (OTA_SOURCE_DIR ${PROJECT_SOURCE_DIR})

# --------------------------------------
#  FIND all required libraries 
# --------------------------------------
find_package ( PCL 1.7 REQUIRED )
#find_package ( Boost COMPONENTS thread chrono date_time program_options filesystem system REQUIRED) #already done by PCL
find_package ( Qt REQUIRED)
find_package ( VTK REQUIRED)

# --------------------------------------
#  including external libraries
# --------------------------------------
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

# --------------------------------------
#  link external libraries
# --------------------------------------
link_directories(${PCL_LIBRARY_DIRS})
add_definitions (${PCL_DEFINITIONS})  


# --------------------------------------
#  Add options to build specific targets
# --------------------------------------
option(setLocalDebugEnvironment "Set the local environment variables for debugging --- USE THIS ONLY FOR DEBUG" "OFF")
if(MSVC AND setLocalDebugEnvironment)
#this is not for all users !!!!
include(${CMAKE_SOURCE_DIR}/CMakeModules/script_setLocalDebugEnvironment.cmake REQUIRED)		# debug local environment configurations
endif(MSVC AND setLocalDebugEnvironment)


# --------------------------------------
#  upd_lib 
# --------------------------------------
option(BUILD_UPD "Option for building UPD_lib - ON by default" ON)
if (BUILD_UPD)
	add_subdirectory(libs/upd_lib)
	set (upd_lib_INCLUDE_DIR ${upd_lib_SOURCE_DIR}/include )
	include_directories("${upd_lib_INCLUDE_DIR}")   
endif(BUILD_UPD)


# --------------------------------------
#  apps
# --------------------------------------
OPTION(BUILD_apps "Option for building apps - ON by default" ON)
if (BUILD_apps)
	add_subdirectory(apps)
endif(BUILD_apps)
# --------------------------------------
#  apps 
# --------------------------------------

# --------------------------------------
#  settings for install and package --- not supported yet
# --------------------------------------					
#include(${PROJECT_SOURCE_DIR}/CMakeModules/script_BuildInstall.cmake REQUIRED)		# CMake configurations
#include(${PROJECT_SOURCE_DIR}/CMakeModules/script_BuildPackage.cmake REQUIRED)		# CMake configurations						
						
						
