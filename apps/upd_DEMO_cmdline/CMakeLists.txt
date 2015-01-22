
project(upd_cmdline_demo)

# --------------------------------------
#  including external libraries
# --------------------------------------
include_directories(${PCL_INCLUDE_DIRS})

# --------------------------------------
#  link external libraries
# --------------------------------------
link_directories(${PCL_LIBRARY_DIRS})

add_executable( upd_cmdline_demo upd_cmdline_demo.cpp )

target_link_libraries (upd_cmdline_demo ${PCL_LIBRARIES} ) 

