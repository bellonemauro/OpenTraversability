# install internal executables file (only from the release folder
# remember that this file will only work on windows !!!

FILE(GLOB FILE_EXE "${PROJECT_BINARY_DIR}/bin/Release/*.exe")
		FOREACH(F ${FILE_EXE})
			INSTALL(FILES "${F}" DESTINATION bin)
		ENDFOREACH(F)

FILE(GLOB INTERNAL_DLL "${PROJECT_BINARY_DIR}/bin/Release/*.dll")
		FOREACH(F ${INTERNAL_DLL})
			INSTALL(FILES "${F}" DESTINATION bin)
		ENDFOREACH(F)

INSTALL(FILES "${PROJECT_SOURCE_DIR}/apps/upd_vREPdemo_cmdline/wheelchairLaserNav.1.2.ttt" DESTINATION ./)		

# This looks for all .dll in QT library, 
# it's of for installers but maybe allow the choice of necessary dlls only is mandatory for the sake of space saving

# TODO: remember to check if dir exist !!
EXTRACT_DEB_REL_DLLS (${QT_BINARY_DIR} d4 )# --> OUR custom macro for searching dlls 
if (ENABLE_VERBOSE)
message (STATUS "\n\n REL DLLS are : ${REL_DLLS}")
message (STATUS "\n\n DEB DLLS are : ${DEB_DLLS}")
endif (ENABLE_VERBOSE) 

FOREACH(F ${REL_DLLS})
	 INSTALL(FILES "${F}" DESTINATION bin)   # install all release dll
ENDFOREACH(F)

FOREACH(F ${DEB_DLLS})
	 #INSTALL(FILES "${F}" DESTINATION bin)   # no install debug for now
ENDFOREACH(F)

# This looks for all .dll in VTK library - VTK has separated folders for debug and release so it is not necessary to use the macro
FILE(GLOB REL_DLLS "${VTK_DIR}/bin/Release/*.dll")
		FOREACH(F ${REL_DLLS})
			INSTALL(FILES "${F}" DESTINATION bin)
		ENDFOREACH(F)

		
		
# TODO: remember to check if dir exist !!
if (Boost_LIBRARY_DIR_RELEASE )
EXTRACT_DEB_REL_DLLS (${Boost_LIBRARY_DIR_RELEASE}   d )# --> OUR custom macro for searching dlls -- d is not the postfix for boost so it will take everything
elseif (Boost_LIBRARY_DIR)
EXTRACT_DEB_REL_DLLS (${Boost_LIBRARY_DIR}   d )# --> OUR custom macro for searching dlls -- d is not the postfix for boost so it will take everything
endif ()
if (ENABLE_VERBOSE)
message (STATUS "\n\n REL DLLS are : ${REL_DLLS}")
message (STATUS "\n\n DEB DLLS are : ${DEB_DLLS}")
endif (ENABLE_VERBOSE) 
FOREACH(F ${REL_DLLS})
	 INSTALL(FILES "${F}" DESTINATION bin)   # install all release dll
ENDFOREACH(F)
#EXTRACT_DEB_REL_DLLS (${Boost_LIBRARY_DIR_DEBUG}   d )# --> OUR custom macro for searching dlls -- d is not the postfix for boost so it will take everything
#FOREACH(F ${DEB_DLLS})
	 #INSTALL(FILES "${F}" DESTINATION bin)   # no install debug for now
#ENDFOREACH(F)

# TODO: remember to check if dir exist !!
EXTRACT_DEB_REL_DLLS (${PCL_DIR}/bin/ _debug )# --> OUR custom macro for searching dlls -- 
if (ENABLE_VERBOSE)
message (STATUS "\n\n REL DLLS are : ${REL_DLLS}")
message (STATUS "\n\n DEB DLLS are : ${DEB_DLLS}")
endif (ENABLE_VERBOSE) 
FOREACH(F ${REL_DLLS})
	 INSTALL(FILES "${F}" DESTINATION bin)   # install all release dll
ENDFOREACH(F)

FOREACH(F ${DEB_DLLS})
	 #INSTALL(FILES "${F}" DESTINATION bin)   # no install debug for now
ENDFOREACH(F)

FILE(GLOB EXTRA_DLLS "${PCL_DIR}/bin/*.dll")
		FOREACH(F ${EXTRA_DLLS})
			INSTALL(FILES "${F}" DESTINATION bin)
		ENDFOREACH(F)

#install (FILES  "${PROJECT_SOURCE_DIR}/Resources/logo_cetma_ant.jpg"  
#                DESTINATION bin/Resources)				
#NOTE: install target doesn't work yet

#NOTE: this have to go into a separated file script____install.cmake
#install (TARGETS ${PROJECT_NAME} DESTINATION bin
#				RUNTIME DESTINATION bin
#				LIBRARY DESTINATION lib
#				ARCHIVE DESTINATION lib)
#install (FILES  "${PROJECT_SOURCE_DIR}/_.h"  DESTINATION include)
#install (FILES  "${upd_lib_BINARY_DIR}/Release/_.dll"   DESTINATION bin)
				
