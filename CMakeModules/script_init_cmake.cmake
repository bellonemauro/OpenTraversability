#cmake init information

#set CMAKEMODULES paths
SET( CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules;${CMAKE_MODULE_PATH}" )   

#support for shared libraries
SET_PROPERTY(GLOBAL PROPERTY TARGET_SUPPORTS_SHARED_LIBS TRUE)  

# find different libraries for different OSs
IF(WIN32 OR win64)
   SET(CMAKE_FIND_LIBRARY_SUFFIXES .lib .dll)
ELSE()
   SET(CMAKE_FIND_LIBRARY_SUFFIXES .a .so) #A simple unix configuration for libraries 
ENDIF()

# Set optimized building (must go BEFORE declaring the targets)
IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -mtune=native")
ENDIF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")

if(MSVC)
	ADD_DEFINITIONS(-D_CRT_SECURE_NO_WARNINGS)  # Disable annoying deprecated warnings in MSVC 
endif(MSVC)

## check for windows 
if(CMAKE_SYSTEM_NAME MATCHES "Windows")
 ## Check for Version ##
    if( ${CMAKE_SYSTEM_VERSION} EQUAL 6.1 ) # Windows 7
		message ("Windows 7 detected")
	else( ${CMAKE_SYSTEM_VERSION} EQUAL 6.2 ) # Windows 8
		message ("Windows 8 detected")
	endif()
else() # Some other Windows
	message ("Windows older than 7 detected")
endif()

#check for 64 or 32 bit architecture
if("${CMAKE_SIZEOF_VOID_P}" EQUAL "8")
	set(BITNESS 64)
else()
	set(BITNESS 32)
endif()

# A function to get all user defined variables with a specified prefix to allow groups in cmake
function (getListOfVarsStartingWith _prefix _varResult)
    get_cmake_property(_vars CACHE_VARIABLES)
    string (REGEX MATCHALL "(^|;)${_prefix}[A-Za-z0-9_]*" _matchedVars "${_vars}")
    set (${_varResult} ${_matchedVars} PARENT_SCOPE)
endfunction()


# ----------------------------------------------------------------------------
# Detect GNU version:
# ----------------------------------------------------------------------------
IF(CMAKE_COMPILER_IS_GNUCXX)
	execute_process(COMMAND ${CMAKE_CXX_COMPILER} -dumpversion
		          OUTPUT_VARIABLE CMAKE_OTA_GCC_VERSION_FULL
		          OUTPUT_STRIP_TRAILING_WHITESPACE)

	  # set the compiler version to g++11
	  if (CMAKE_VERSION VERSION_LESS "3.1")
	    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
	      set (CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")
	    endif ()
	  else ()
	    set (CMAKE_CXX_STANDARD 11)
	  endif ()

	# we enable standard C++11 multithread support.
	if( NOT WIN32 )
	    #list(APPEND EXTRA_C_FLAGS -pthread)
	    set(USE_PTHREAD ON)
	    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
	    message(STATUS "TBB and OMP disabled: Using Pthread instead.")
	else( NOT WIN32 )
	    set(USE_PTHREAD OFF)
	endif( NOT WIN32 )

	# Output in CMAKE_OTA_GCC_VERSION_FULL: "X.Y"
	#  Look for the version number
	STRING(REGEX MATCH "[0-9]+.[0-9]+" CMAKE_GCC_REGEX_VERSION "${CMAKE_OTA_GCC_VERSION_FULL}")

	# Split the three parts:
	STRING(REGEX MATCHALL "[0-9]+" CMAKE_OTA_GCC_VERSIONS "${CMAKE_GCC_REGEX_VERSION}")

	LIST(GET CMAKE_OTA_GCC_VERSIONS 0 CMAKE_OTA_GCC_VERSION_MAJOR)
	LIST(GET CMAKE_OTA_GCC_VERSIONS 1 CMAKE_OTA_GCC_VERSION_MINOR)

	SET(CMAKE_CADSAMPLES_GCC_VERSION ${CMAKE_OTA_GCC_VERSION_MAJOR}${CMAKE_OTA_GCC_VERSION_MINOR})

	IF($ENV{VERBOSE})
		MESSAGE(STATUS "gcc -dumpversion: ${CMAKE_OTA_GCC_VERSION_FULL}  -> Major=${CMAKE_OTA_GCC_VERSION_MAJOR} Minor=${CMAKE_OTA_GCC_VERSION_MINOR}")
	ENDIF($ENV{VERBOSE})


ENDIF(CMAKE_COMPILER_IS_GNUCXX)



# ----------------------------------------------------------------------------
#  Loads the current version number:
# ----------------------------------------------------------------------------
FILE(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/version_prefix.txt" CMAKE_OTA_VERSION_NUMBER LIMIT_COUNT 1) # Read only the first line

# For example: "0.5.1"
STRING(REGEX MATCHALL "[0-9]+" CMAKE_OTA_VERSION_PARTS "${CMAKE_OTA_VERSION_NUMBER}")

LIST(GET CMAKE_OTA_VERSION_PARTS 0 CMAKE_OTA_VERSION_NUMBER_MAJOR)
LIST(GET CMAKE_OTA_VERSION_PARTS 1 CMAKE_OTA_VERSION_NUMBER_MINOR)
LIST(GET CMAKE_OTA_VERSION_PARTS 2 CMAKE_OTA_VERSION_NUMBER_PATCH)
SET(CMAKE_OTA_FULL_VERSION "${CMAKE_OTA_VERSION_NUMBER_MAJOR}.${CMAKE_OTA_VERSION_NUMBER_MINOR}.${CMAKE_OTA_VERSION_NUMBER_PATCH}")

IF(WIN32)
	SET(OTA_DLL_VERSION_POSTFIX "${CMAKE_OTA_VERSION_NUMBER_MAJOR}${CMAKE_OTA_VERSION_NUMBER_MINOR}${CMAKE_OTA_VERSION_NUMBER_PATCH}")
ELSE(WIN32)
	SET(OTA_DLL_VERSION_POSTFIX "")
ENDIF(WIN32)



# Setup output Directories 
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin  CACHE PATH "Single Directory for all Libraries" ) 
# libraries and executables are going into the same folder to avoid annoying environmental variables in MSVC 

# Setup the Executables output Directory 
SET (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin  CACHE PATH "Single Directory for all Executables." )

# Setup the archive output Directory 
SET (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin  CACHE PATH "Single Directory for all static libraries.")

