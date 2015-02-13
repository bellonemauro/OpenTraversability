#NOTE: this have to go into a separated file script_DUNE_package.cmake
#NOTE: package target doesn't work yet
# build a CPack driven installer package
include (InstallRequiredSystemLibraries)
set (CPACK_RESOURCE_FILE_LICENSE  
     "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set (CPACK_PACKAGE_VERSION_MAJOR "${PCL_upd_demo_VERSION_MAJOR}")
set (CPACK_PACKAGE_VERSION_MINOR "${PCL_upd_demo_VERSION_MINOR}")
set (CPACK_NSIS_MUI_ICON "${CMAKE_CURRENT_SOURCE_DIR}/Resources/icon.ico")
#set (CPACK_PACKAGE_ICON "${CMAKE_CURRENT_SOURCE_DIR}/Resources/icon.bmp") -- FILE NOT FOUND -- ??
if( WIN32 AND NOT UNIX )
    #SET(CPACK_PACKAGE_EXECUTABLES "Target_Name" "Target Name")
    SET(CPACK_PACKAGE_EXECUTABLES OpenTraversability "OpenTraversability")
endif()
include (CPack)
