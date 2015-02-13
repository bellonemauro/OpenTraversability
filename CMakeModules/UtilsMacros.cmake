# ----------------------------------------------------------------------------
# Useful macros to define strings and directories 
# inspired by https://github.com/jlblancoc/mrpt/blob/master/cmakemodules/UtilsMacros.cmake
# ----------------------------------------------------------------------------

# simple macro to append a _VALUE as suffix to _VAR
macro(STRING_APPEND  _VAR _VALUE )
  IF (${_VAR})
    # not empty, add space and value
    SET(${_VAR} "${${_VAR}} ${_VALUE}")
  ELSE(${_VAR})
    # empty, no space required.
    SET(${_VAR} "${_VALUE}")
  ENDIF (${_VAR})
endmacro(STRING_APPEND)

#-----------------------------------------------------------------------------
# Only if GNU GCC is used, add one "-isystem" flag for each include directory.
# Useful to discard -pedantic errors in system libraries not prepared to be so... well, pedantic.
macro(ADD_DIRECTORIES_AS_ISYSTEM INCLUDE_DIRS)
	IF($ENV{VERBOSE})
		MESSAGE(STATUS "isystem: INCLUDE_DIRS= ${${INCLUDE_DIRS}}")
	ENDIF($ENV{VERBOSE})
	IF(CMAKE_COMPILER_IS_GNUCXX OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
		FOREACH(DIR ${${INCLUDE_DIRS}})
			SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -isystem ${DIR}")
		ENDFOREACH(DIR)
	ENDIF(CMAKE_COMPILER_IS_GNUCXX OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
endmacro(ADD_DIRECTORIES_AS_ISYSTEM)
#-----------------------------------------------------------------------------


#-----------------------------------------------------------------------------
# Only if GNU GCC is used, add one "-isystem" flag for each include directory.
macro(ADD_DIRECTORIES_AS_INCLUDE_AND_ISYSTEM INCLUDE_DIRS)
	IF($ENV{VERBOSE})
		MESSAGE(STATUS "isystem: INCLUDE_DIRS= ${${INCLUDE_DIRS}}")
	ENDIF($ENV{VERBOSE})
	IF(CMAKE_COMPILER_IS_GNUCXX OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
		FOREACH(DIR ${${INCLUDE_DIRS}})
			SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I ${DIR} -isystem ${DIR}")
		ENDFOREACH(DIR)
	ENDIF(CMAKE_COMPILER_IS_GNUCXX OR "${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
endmacro(ADD_DIRECTORIES_AS_INCLUDE_AND_ISYSTEM)
#-----------------------------------------------------------------------------


#-----------------------------------------------------------------------------
# Based on: http://www.cmake.org/pipermail/cmake/2008-February/020114.html
# Usage: list_subdirectories(the_list_is_returned_here C:/cwd)
macro(list_subdirectories retval curdir)
  file(GLOB sub_dir RELATIVE ${curdir} *)
  set(list_of_dirs "")
  foreach(dir ${sub_dir})
    string(SUBSTRING ${dir} 0 1 dir1st)
    if(IS_DIRECTORY ${curdir}/${dir} AND NOT "${dir1st}" STREQUAL "." AND NOT ${dir} STREQUAL "CMakeFiles")
        set(list_of_dirs ${list_of_dirs} ${dir})
    endif(IS_DIRECTORY ${curdir}/${dir} AND NOT "${dir1st}" STREQUAL "." AND NOT ${dir} STREQUAL "CMakeFiles")
  endforeach(dir)
  set(${retval} ${list_of_dirs})
endmacro(list_subdirectories)
#-----------------------------------------------------------------------------

# Parse a list of "pkg-config"-like flags and split them into INCLUDE_DIRS,etc.
# Example: pkgconfig_parse("-Ldir1 -llib1 -Idir2" "FOO")
#  --> FOO_INCLUDE_DIRS = "dir2"
#  --> FOO_LINK_DIRS = "dir1"
#  --> FOO_LIBS = "lib1"
macro(pkgconfig_parse _FLAGS _OUT_PREFIX)
	string(REPLACE " " ";" _FLAGS_LST ${_FLAGS})
	SET(${_OUT_PREFIX}_INCLUDE_DIRS "")
	SET(${_OUT_PREFIX}_LINK_DIRS "")
	SET(${_OUT_PREFIX}_LIBS "")
	foreach(str ${_FLAGS_LST})
		string(LENGTH ${str} _LEN)
		if (_LEN GREATER 2)
			string(SUBSTRING ${str} 0 2 _START)
			MATH( EXPR _LEN2 "${_LEN}-2" )
			string(SUBSTRING ${str} 2 ${_LEN2} _REST)
			IF (${_START} STREQUAL "-L")
				LIST(APPEND ${_OUT_PREFIX}_LINK_DIRS ${_REST})
			ELSEIF (${_START} STREQUAL "-l")
				LIST(APPEND ${_OUT_PREFIX}_LIBS ${_REST})
			ELSEIF (${_START} STREQUAL "-I")
				LIST(APPEND ${_OUT_PREFIX}_INCLUDE_DIRS ${_REST})
			ENDIF (${_START} STREQUAL "-L")
		endif (_LEN GREATER 2)
	endforeach(str)
endmacro(pkgconfig_parse )

# Convert a decimal value [0,15] to hexadecimal
# From: http://stackoverflow.com/questions/26182289/convert-from-decimal-to-hexadecimal-in-cmake
macro(DECCHAR2HEX VAR VAL)
        if (${VAL} LESS 10)
            SET(${VAR} ${VAL})
        elseif(${VAL} EQUAL 10)
            SET(${VAR} "A")
        elseif(${VAL} EQUAL 11)
            SET(${VAR} "B")
        elseif(${VAL} EQUAL 12)
            SET(${VAR} "C")
        elseif(${VAL} EQUAL 13)
            SET(${VAR} "D")
        elseif(${VAL} EQUAL 14)
            SET(${VAR} "E")
        elseif(${VAL} EQUAL 15)
            SET(${VAR} "F")
        else(${VAL} LESS 10)
            MESSAGE(FATAL_ERROR "Invalid format for hexidecimal character")
        endif(${VAL} LESS 10)
endmacro(DECCHAR2HEX)

# Converts a version like "1.2.3" into a string "0x123"
# Usage: VERSION_TO_HEXADECIMAL(TARGET_VAR "1.2.3")
macro(VERSION_TO_HEXADECIMAL  OUT_VAR IN_VERSION)
	STRING(REGEX MATCHALL "[0-9]+" VERSION_PARTS "${IN_VERSION}")
	LIST(GET VERSION_PARTS 0 VERSION_NUMBER_MAJOR)
	LIST(GET VERSION_PARTS 1 VERSION_NUMBER_MINOR)
	LIST(GET VERSION_PARTS 2 VERSION_NUMBER_PATCH)
	# Convert each part to hex:
	DECCHAR2HEX(VERSION_NUMBER_MAJOR_HEX ${VERSION_NUMBER_MAJOR})
	DECCHAR2HEX(VERSION_NUMBER_MINOR_HEX ${VERSION_NUMBER_MINOR})
	DECCHAR2HEX(VERSION_NUMBER_PATCH_HEX ${VERSION_NUMBER_PATCH})
	# Concat version string:
	SET(${OUT_VAR} "0x${VERSION_NUMBER_MAJOR_HEX}${VERSION_NUMBER_MINOR_HEX}${VERSION_NUMBER_PATCH_HEX}")
endmacro(VERSION_TO_HEXADECIMAL)


# this macros are define to match libraries file name in a specific format
# Example of usage: 
#  REMOVE_MATCHING_FILES_FROM_LIST(".*_LIN.cpp" my_srcs)

macro(REMOVE_MATCHING_FILES_FROM_LIST match_expr lst_files)
	SET(lst_files_aux "")
	FOREACH(FIL ${${lst_files}})
		IF(NOT ${FIL} MATCHES "${match_expr}")
			SET(lst_files_aux "${lst_files_aux}" "${FIL}")
		ENDIF(NOT ${FIL} MATCHES "${match_expr}")
	ENDFOREACH(FIL)
	SET(${lst_files} ${lst_files_aux})
endmacro(REMOVE_MATCHING_FILES_FROM_LIST)



macro(KEEP_MATCHING_FILES_FROM_LIST match_expr lst_files)
	SET(lst_files_aux "")
	FOREACH(FIL ${${lst_files}})
		IF(${FIL} MATCHES "${match_expr}")
			SET(lst_files_aux "${lst_files_aux}" "${FIL}")
		ENDIF(${FIL} MATCHES "${match_expr}")
	ENDFOREACH(FIL)
	SET(${lst_files} ${lst_files_aux})
endmacro(KEEP_MATCHING_FILES_FROM_LIST)

# this macro extract release and debug dlls from a directory to chose which one have to be installed
# EXTRACT_DEB_REL_DLLS  -- macro name
# DLL_DIR        --- in - the folder where you want to search dlls
# DEBUG_POSTFIX  --- in - debug postfix to be analysed
# 
# /note this macro create and replace 2 vars  REL_DLLS and DEB_DLLS 
#       such variables are intentionally NOT cached by cmake
# /note This macro is only for windows !! 
macro (EXTRACT_DEB_REL_DLLS DLL_DIR DEBUG_POSTFIX)
      
	  if (EXISTS ${DLL_DIR})

		FILE(GLOB REL_DLLS "${DLL_DIR}/*.dll")
		FILE(GLOB DEB_DLLS "${DLL_DIR}/*${DEBUG_POSTFIX}.dll")	

		FOREACH(F ${DEB_DLLS} )
		LIST (REMOVE_ITEM REL_DLLS "${F}")
		ENDFOREACH(F)
	  else ()
		message (WARNING "EXTRACT_DEB_REL_DLLS error - dir empty")
	  endif ()
	  
endmacro (EXTRACT_DEB_REL_DLLS)