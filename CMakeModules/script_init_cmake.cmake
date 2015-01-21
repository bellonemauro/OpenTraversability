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

# Setup output Directories 
set (CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin  CACHE PATH "Single Directory for all Libraries" ) 
# libraries and executables are going into the same folder to avoid annoying environmental variables in MSVC 

# Setup the Executables output Directory 
SET (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin  CACHE PATH "Single Directory for all Executables." )

# Setup the archive output Directory 
SET (CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin  CACHE PATH "Single Directory for all static libraries.")

