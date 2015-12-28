CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)

PROJECT(ros_glc)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
catkin_package()

SET(GLC_SOVER 0)
SET(GLC_VER 0.5.8)

IF (NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE "Release")
ENDIF (NOT CMAKE_BUILD_TYPE)

OPTION(QUICKLZ
       "QuickLZ support"
       ON)
OPTION(LZO
       "LZO support"
       ON)
OPTION(LZJB
       "LZJB support"
       ON)
OPTION(BINARIES
       "Build and install glc-capture and glc-play"
       ON)
OPTION(HEADERS
       "Install headers"
       ON)
OPTION(HOOK
       "Build and install glc-hook"
       ON)
OPTION(SCRIPTS
       "Install sample scripts."
       OFF)

IF (NOT GIT_DIR)
  SET(GIT_DIR ${PROJECT_SOURCE_DIR}/.git)
ENDIF (NOT GIT_DIR)

ADD_SUBDIRECTORY(src)

IF (SCRIPTS)
  IF (UNIX)
    INSTALL(FILES scripts/capture.sh
    		  scripts/encode.sh
    		  scripts/play.sh
    		  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
                  USE_SOURCE_PERMISSIONS)
  ENDIF (UNIX)
ENDIF (SCRIPTS)

add_custom_command(OUTPUT ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}/glc-capture
  COMMAND cmake -E make_directory ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}
  COMMAND cmake -E copy ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/glc-capture ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}/
  DEPENDS glc-capture)
add_custom_target(install_glc_capture ALL DEPENDS ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}/glc-capture)

add_custom_command(OUTPUT ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}/glc-play
  COMMAND cmake -E make_directory ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}
  COMMAND cmake -E copy ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/glc-play ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}/
  DEPENDS glc-play)
add_custom_target(install_glc_play ALL DEPENDS ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}/glc-play)