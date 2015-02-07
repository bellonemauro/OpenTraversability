#  +---------------------------------------------------------------------------+
#  |                                         			                       |
#  |               https://sites.google.com/site/bellonemauro/                 |
#  |                                                                           |
#  | Copyright (c) 2015, - All rights reserved.                                |
#  | Authors: Mauro Bellone                                                    |
#  | Released under BDS License.                                               |
#  +---------------------------------------------------------------------------+ */

project(ALL_OT_LIBS)

# Enumerate all subdirs and process them:
list_subdirectories(LIST_LIBS_DIRS "${CMAKE_SOURCE_DIR}/libs")

foreach(dir ${LIST_LIBS_DIRS})
	SET(VAR_NAME "LIBS_BUILD_${dir}")
	SET(${VAR_NAME} ON CACHE BOOL "Build the lib ${dir}")
	MARK_AS_ADVANCED(${VAR_NAME})

	if (${VAR_NAME})
	add_subdirectory(${dir})
	#TODO MAURO  check for remove subdirectory in case of subdirectory not defined 
	endif (${VAR_NAME})
endforeach(dir)
