
project(upd_svm_cmdline_demo)

# --------------------------------------
#  including external libraries
# --------------------------------------
include_directories(${PCL_INCLUDE_DIRS})

# --------------------------------------
#  link external libraries
# --------------------------------------
link_directories(${PCL_LIBRARY_DIRS})

add_executable( upd_svm_cmdline_demo upd_svm_cmdline_demo.cpp )

target_link_libraries (upd_svm_cmdline_demo ${PCL_LIBRARIES} ) 

# allows folders for MSVC
if (MSVC AND ENABLE_SOLUTION_FOLDERS) 
	set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "Apps")
endif (MSVC AND ENABLE_SOLUTION_FOLDERS)