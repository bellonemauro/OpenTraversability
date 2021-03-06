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
	
	# allows automatic linking of QT - see http://www.cmake.org/cmake/help/v3.0/policy/CMP0020.html
	cmake_policy(SET CMP0020 NEW) 
	
	#Ignore COMPILE_DEFINITIONS_<Config> properties - see http://www.cmake.org/cmake/help/v3.0/policy/CMP0043.html
	if (${CMAKE_MAJOR_VERSION}>2)	
	cmake_policy(SET CMP0043 NEW) 
	endif (${CMAKE_MAJOR_VERSION}>2)

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
option(ENABLE_SOLUTION_FOLDERS "Enable folders for MSVC - ON by default if MSVC - OFF otherwise" ON)
if (MSVC)
  set (ENABLE_SOLUTION_FOLDERS 1)
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)
else (MSVC)
  set (ENABLE_SOLUTION_FOLDERS 0)
endif (MSVC) 

option(ENABLE_setLocalDebugEnvironment "Set the local environment variables for debugging --- USE THIS ONLY FOR DEBUG" "OFF")
if(MSVC AND ENABLE_setLocalDebugEnvironment)
#this is not for all users !!!!
include(${CMAKE_SOURCE_DIR}/CMakeModules/script_setLocalDebugEnvironment.cmake REQUIRED)		# debug local environment configurations
endif(MSVC AND ENABLE_setLocalDebugEnvironment)


# add all libraries in the /libs folder
add_subdirectory(libs) 

# -----------  upd_lib  --------------------
include_directories("${upd_lib_INCLUDE_DIR}")   

# -------------------- apps --------------------------------
OPTION(ENABLE_BUILD_apps "Option for building apps - ON by default" ON)
if (ENABLE_BUILD_apps)
	add_subdirectory(apps)
endif(ENABLE_BUILD_apps)


# --------------------------------------
#  settings for install and package --- not supported yet
# --------------------------------------					
if (MSVC)   #install does not work on unix for now
include(${PROJECT_SOURCE_DIR}/CMakeModules/script_BuildInstall.cmake REQUIRED)		# CMake configurations
include(${PROJECT_SOURCE_DIR}/CMakeModules/script_BuildPackage.cmake REQUIRED)		# CMake configurations						
endif(MSVC)						
						
