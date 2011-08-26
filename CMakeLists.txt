# The name of the project is "MOVE3DSTUDIO". CMakeLists files in this project can
# refer to the root source directory of the project as ${MOVE3DSTUDIO_SOURCE_DIR} and
# to the root binary directory of the project as ${MOVE3DSTUDIO_BINARY_DIR}.
cmake_minimum_required (VERSION 2.6)

PROJECT(MOVE3DSTUDIO C CXX)

set( PACKAGE_NAME move3d-studio )
set( PACKAGE_VERSION 1.0 )

# CMake information
message( STATUS "CMake System = ${CMAKE_SYSTEM_NAME}")
message( STATUS "CMake Processor = ${CMAKE_SYSTEM_PROCESSOR}")
message( STATUS "CMake Source Dir = ${CMAKE_SOURCE_DIR}")
message( STATUS "CMake Binary Dir = ${CMAKE_BINARY_DIR}")
message( STATUS "CMake Install Prefix = ${CMAKE_INSTALL_PREFIX}")


IF(UNIX)
  ADD_DEFINITIONS(-DUNIX)
  IF(APPLE)
    ADD_DEFINITIONS(-DMACOSX)
  ELSE(APPLE)
    ADD_DEFINITIONS(-DLINUX)
  ENDIF(APPLE)
ELSE(UNIX)
 MESSAGE (FATAL_ERROR "Not supported Platform")
ENDIF(UNIX)

#Default mode is Debug  --------------------------------------------------------
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

# Get the current move3d-studio version
ADD_DEFINITIONS(${MOVE3D_CFLAGS} -DMOVE3D_STUDIO_VERSION="${PACKAGE_VERSION}" )

#Uncomment next line to get verbose makefiles.
#SET(CMAKE_VERBOSE_MAKEFILE true)

# Pick up our scripts - they are all in the conf subdirectory
SET(MOVE3DSTUDIO_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
SET(CMAKE_MODULE_PATH ${MOVE3DSTUDIO_MODULE_PATH})

INCLUDE("${CMAKE_MODULE_PATH}/TargetDoc.cmake" OPTIONAL)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/script/CMake")
SET(BUILD_SHARED_LIBS true)

FIND_PACKAGE (PkgConfig REQUIRED)

#--------------------- Project ------------------------------------------------

##########################################
##           Options                   ###
##########################################

OPTION (MOVE3D_XFORMS "Use XForms toolkit for interface" OFF)
OPTION (MOVE3D_QT "Use Qt toolkit for interface" OFF)
OPTION (MOVE3D_QT_REMOTE "Use Qt toolkit for remote interface with pocolibs poster reading" OFF)

OPTION (HRI_GUI "Enable Human-Robot Interaction related parts" OFF)
OPTION (MIGHTABILITY_MAPS "Enable MigthabilityMaps interface" OFF)

IF(MOVE3D_XFORMS)
	#-------------- XFORMS --------------------------------------------------------
	ADD_DEFINITIONS(-DMOVE3D_XFORMS)
	ADD_DEFINITIONS(-DWITH_XFORMS)
	FIND_PACKAGE(XFORMS REQUIRED)
	INCLUDE_directories(${XFORMS_INCLUDE_DIR})
	SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${XFORMS_INCLUDE_DIR})
	SET(LIBS ${LIBS} ${XFORMS_LIBRARIES})
ENDIF()

IF(MOVE3D_QT)
	ADD_DEFINITIONS(-DMOVE3D_QT)
ENDIF()

IF(MIGHTABILITY_MAPS)
	ADD_DEFINITIONS(-DUSE_MIGHTABILITY_MAPS)
ENDIF()

IF(HRI_GUI)
	ADD_DEFINITIONS(-DHRI_PLANNER)
	ADD_DEFINITIONS(-DHRI_PLANNER_GUI) #Needed for libmove3d/include/device.h (p3d_rob struct)
	
	### libhri dependency
        IF(DEFINED LIBHRI_PC)
		SET(ENV{PKG_CONFIG_PATH} ${LIBHRI_PC}:$ENV{PKG_CONFIG_PATH})
	ENDIF()
	PKG_CHECK_MODULES(LIBHRI REQUIRED libhri)
	INCLUDE_DIRECTORIES(${LIBHRI_INCLUDE_DIRS})
	LINK_DIRECTORIES( ${LIBHRI_LIBRARY_DIRS} )
	SET(LIBS ${LIBS} ${LIBHRI_LIBRARIES})
ENDIF()

##########################################
##       PKGCONFIG file                ###
##########################################

IF   (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/move3d-studio.pc.in")
    MESSAGE(STATUS "configured ${CMAKE_CURRENT_SOURCE_DIR}/move3d-studio.pc.in --> ${CMAKE_CURRENT_BINARY_DIR}/move3d-studio.pc")
    CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/move3d-studio.pc.in 
      ${CMAKE_CURRENT_BINARY_DIR}/move3d-studio.pc
      @ONLY )

	INSTALL(FILES
		${CMAKE_CURRENT_BINARY_DIR}/move3d-studio.pc
		DESTINATION lib/pkgconfig
	)
ENDIF(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/move3d-studio.pc.in")

##########################################
##       HRI library                 ###
##########################################

ADD_SUBDIRECTORY (src)


