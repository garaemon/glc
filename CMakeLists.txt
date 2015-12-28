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
