#  +---------------------------------------------------------------------------+
#  |                                                                           |
#  |               https://sites.google.com/site/bellonemauro/                 |
#  |                                                                           |
#  | Copyright (c) 2015, - All rights reserved.                                |
#  | Authors: Mauro Bellone                                                    |
#  | Released under BDS License.                                               |
#  +---------------------------------------------------------------------------+ */

project(PCL_upd_DEMO)


# --------------------------------------
#  including external libraries
# --------------------------------------
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Boost_LIBRARY_DIR})
include_directories(${Boost_INCLUDE_DIRS})


find_package ( VTK REQUIRED)

# --------------------------------------
#  set variables for compilation 
# --------------------------------------
# select all headers in the include dir
FILE(GLOB PCL_upd_DEMO_HEADERS "${PCL_upd_DEMO_SOURCE_DIR}/*.h")   
# select all sources in the include dir
FILE(GLOB PCL_upd_DEMO_SOURCES "${PCL_upd_DEMO_SOURCE_DIR}/*.cpp")  
# select all files for the graphical interface
FILE(GLOB PCL_upd_DEMO_FORMS "${PCL_upd_DEMO_SOURCE_DIR}/*.ui")  

set  (VTK_LIBRARIES )# vtkRendering vtkGraphics vtkHybrid QVTK)  #--> the specific libraries are not necessary in VTK6.1

# this is necessary to find in the code the ui____.h file built in the build folder
include_directories(${PCL_upd_DEMO_BINARY_DIR}) 
include_directories(${upd_lib_INCLUDE_DIR}) 

# --------------------------------------
#  link external libraries
# --------------------------------------
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${QT_LIBRARY_DIR})
link_directories(${QT_BINARY_DIR})



# --------------------------------------
#  add definitions
# --------------------------------------
add_definitions (${PCL_DEFINITIONS})  #required to use PCL
add_definitions (${QT_DEFINITIONS})   #required to use QT and gui

# Add options to build specific targets
option(build_QT5 "Build support with QT 5" "OFF")
if(NOT build_QT5)
	find_package (Qt4 REQUIRED QtCore QtNetwork QtGui QtOpenGL QtWebkit)

	set (QT_QMAKE_EXECUTABLE )
	message (STATUS "QT_BINARY_DIR ---- ${QT_BINARY_DIR}")


	QT4_WRAP_CPP (PCL_upd_DEMO_HEADERS_MOC   ${PCL_upd_DEMO_HEADERS})

	QT4_WRAP_UI  (PCL_upd_DEMO_FORMS_HEADERS ${PCL_upd_DEMO_FORMS})
	include(${VTK_USE_FILE})
	include         (${QT_USE_FILE})
	add_definitions (${QT_DEFINITIONS})   #required to use QT and gui

	add_executable (PCL_upd_DEMO  	${PCL_upd_DEMO_SOURCES} ${PCL_upd_DEMO_HEADERS}
		                        ${PCL_upd_DEMO_FORMS_HEADERS}
		                        ${PCL_upd_DEMO_HEADERS_MOC})

if (NOT UNIX)
	message (STATUS "PCL_UPD_DEMO ---- WIN found")
	target_link_libraries ( PCL_upd_DEMO  upd_lib  #Qt4::QtGui QVTK   # windows does not need to link this 
							${Boost_LIBRARIES}
							${PCL_LIBRARIES} )

elseif ()
	message (STATUS "PCL_UPD_DEMO ---- UNIX found")
	target_link_libraries ( PCL_upd_DEMO  upd_lib  Qt4::QtGui QVTK
							${Boost_LIBRARIES}
							${PCL_LIBRARIES} )
endif ()

elseif(  build_QT5)

	#set(CMAKE_PREFIX_PATH "C:/Qt/5.4/msvc2013_64_opengl/")

	# Find the QtWidgets library
	find_package(Qt5Widgets REQUIRED)

	# Instruct CMake to run moc automatically when needed.
	set(CMAKE_AUTOMOC ON)
	# Find includes in corresponding build directories
	set(CMAKE_INCLUDE_CURRENT_DIR ON)

	 
	QT5_WRAP_UI(PCL_upd_DEMO_FORMS_HEADERS ${PCL_upd_DEMO_FORMS})

	message (STATUS "PCL_upd_DEMO_FORMS_HEADERS  :: ${PCL_upd_DEMO_FORMS_HEADERS} ")

	add_executable (PCL_upd_DEMO  	${PCL_upd_DEMO_SOURCES} ${PCL_upd_DEMO_HEADERS}
		                        ${PCL_upd_DEMO_FORMS_HEADERS})
		                        #${PCL_upd_DEMO_HEADERS_MOC})#${PCL_upd_DEMO_FORMS} )
	qt5_use_modules(PCL_upd_DEMO Widgets)

	target_link_libraries ( PCL_upd_DEMO  upd_lib  
							${Boost_LIBRARIES}
							${PCL_LIBRARIES} )
			

endif()

# allows folders for MSVC
if (MSVC AND ENABLE_SOLUTION_FOLDERS) 
	set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Apps")
endif (MSVC AND ENABLE_SOLUTION_FOLDERS)


