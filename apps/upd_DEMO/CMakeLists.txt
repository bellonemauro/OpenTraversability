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
include  (${QT_USE_FILE})


# --------------------------------------
#  set variables for compilation 
# --------------------------------------
set  (PCL_upd_DEMO_SOURCES ${PCL_upd_DEMO_SOURCE_DIR}/main.cpp ${PCL_upd_DEMO_SOURCE_DIR}/PCL_upd_DEMO.cpp) 	# sources
set  (PCL_upd_DEMO_HEADERS ${PCL_upd_DEMO_SOURCE_DIR}/PCL_upd_DEMO.h)	   			# headers
set  (PCL_upd_DEMO_FORMS   ${PCL_upd_DEMO_SOURCE_DIR}/PCL_upd_DEMO.ui)				# graphical interface
set  (VTK_LIBRARIES  vtkRendering vtkGraphics vtkHybrid QVTK)  #--> the specific libraries are not necessary in VTK6.1

QT4_WRAP_CPP (PCL_upd_DEMO_HEADERS_MOC   ${PCL_upd_DEMO_HEADERS})
QT4_WRAP_UI  (PCL_upd_DEMO_FORMS_HEADERS ${PCL_upd_DEMO_FORMS})


# this is necessary to find in the code the ui____.h file built in the build folder
include_directories(${PCL_upd_DEMO_BINARY_DIR}) 

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

add_executable (PCL_upd_DEMO  	${PCL_upd_DEMO_SOURCES} ${PCL_upd_DEMO_HEADERS}
                                ${PCL_upd_DEMO_FORMS_HEADERS}
                                ${PCL_upd_DEMO_HEADERS_MOC})

target_link_libraries ( PCL_upd_DEMO  upd_lib  
						${Boost_LIBRARIES}
						${PCL_LIBRARIES} 
						${QT_LIBRARIES}
						${VTK_LIBRARIES}) 

					
