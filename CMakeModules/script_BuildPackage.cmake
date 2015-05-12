#NOTE: this have to go into a separated file script_DUNE_package.cmake
#NOTE: package target doesn't work yet
# build a CPack driven installer package
include (InstallRequiredSystemLibraries)
set (CPACK_NSIS_MODIFY_PATH "ON")
set (_WEBSITE "https://www.maurobellone.com")
set (CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set (CPACK_PACKAGE_VERSION "${CMAKE_OTA_FULL_VERSION}")

set (CPACK_NSIS_MUI_ICON "${CMAKE_CURRENT_SOURCE_DIR}/Resources/icon.ico")
#set (CPACK_PACKAGE_ICON "${CMAKE_CURRENT_SOURCE_DIR}/Resources/icon.bmp") -- FILE NOT FOUND -- ??
if( WIN32 AND NOT UNIX )
    #SET(CPACK_PACKAGE_EXECUTABLES "Target_Name" "Target Name")
    SET(CPACK_PACKAGE_EXECUTABLES PCL_upd_DEMO "PCL_upd_DEMO")
endif()

message (STATUS "BUILD PACKAGE STATUS MESSAGE : building version ${CMAKE_OTA_FULL_VERSION} " )


set(CPACK_PACKAGE_INSTALL_REGISTRY_KEY "${CMAKE_PROJECT_NAME}-${CMAKE_OTA_FULL_VERSION}")


set(CPACK_NSIS_MENU_LINKS "${_WEBSITE}" "Homepage for PCL_upd_DEMO")

set(CPACK_CREATE_DESKTOP_LINKS PCL_upd_DEMO)

# Icon in the add/remove control panel. Must be an .exe file 
set(CPACK_NSIS_INSTALLED_ICON_NAME bin\\\\PCL_upd_DEMO.exe)

set(CPACK_NSIS_URL_INFO_ABOUT "${_WEBSITE}")
set(CPACK_NSIS_HELP_LINK "${_WEBSITE}")


include (CPack)
