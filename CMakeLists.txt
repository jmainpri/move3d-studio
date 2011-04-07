# The name of the project is "MOVE3DSTUDIO". CMakeLists files in this project can
# refer to the root source directory of the project as ${MOVE3DSTUDIO_SOURCE_DIR} and
# to the root binary directory of the project as ${MOVE3DSTUDIO_BINARY_DIR}.
cmake_minimum_required (VERSION 2.6)

PROJECT(MOVE3DSTUDIO C CXX)

set( PACKAGE_NAME move3d-studio )
set( PACKAGE_VERSION 1.0 )

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

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

##########################################
##           Options                   ###
##########################################

OPTION (WITH_XFORMS "Use XForms toolkit for interface" ON)
OPTION (HRI_GUI "Enable Human-Robot Interaction related parts" ON)
OPTION (MIGHTABILITY_MAPS "Enable MigthabilityMaps interface" OFF)

IF(WITH_XFORMS)
	ADD_DEFINITIONS(-DWITH_XFORMS)
ENDIF()

IF(MIGHTABILITY_MAPS)
	ADD_DEFINITIONS(-DUSE_MIGHTABILITY_MAPS)
ENDIF()

IF(HRI_GUI)
	ADD_DEFINITIONS(-DHRI_PLANNER)
	ADD_DEFINITIONS(-DHRI_PLANNER_GUI) #Needed for libmove3d/include/device.h (p3d_rob struct)
	
	### libhri dependency
        IF(DEFINED LIBHRI_PC)
		SET(ENV{PKG_CONFIG_PATH} $ENV{PKG_CONFIG_PATH}:${LIBHRI_PC})
	ENDIF()
	PKG_CHECK_MODULES(LIBHRI REQUIRED libhri)
	INCLUDE_DIRECTORIES(${LIBHRI_INCLUDE_DIRS})
	LINK_DIRECTORIES( ${LIBHRI_LIBRARY_DIRS} )
	SET(LIBS ${LIBS} ${LIBHRI_LIBRARIES})
ENDIF()

##########################################
##      Others dependencies             ##
##########################################


#--------------------- Move3D -------------------------------------------------
IF(DEFINED LIBMOVE3D_PC)
	SET(ENV{PKG_CONFIG_PATH} $ENV{PKG_CONFIG_PATH}:${LIBMOVE3D_PC})
ENDIF()
pkg_check_modules(MOVE3D REQUIRED "libmove3d")

INCLUDE_DIRECTORIES(${MOVE3D_INCLUDE_DIRS})
SET(LIBS ${LIBS} ${MOVE3D_LIBRARIES})
LINK_DIRECTORIES( ${MOVE3D_LIBRARY_DIRS} )
#MESSAGE( MOVE3D_LIBRARIES = ${MOVE3D_LIBRARIES})

#-------------- XFORMS --------------------------------------------------------

FIND_PACKAGE(XFORMS REQUIRED)
INCLUDE_directories(${XFORMS_INCLUDE_DIR})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${XFORMS_INCLUDE_DIR})
SET(LIBS ${LIBS} ${XFORMS_LIBRARIES})

#--------------- OPENGL -------------------------------------------------------

FIND_PACKAGE(X11 REQUIRED)
INCLUDE_directories(${X11_INCLUDE_DIR})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${X11_INCLUDE_DIR})
SET(LIBS ${LIBS} ${X11_LIBRARIES})

IF(APPLE)
	IF(GL_NATIVE)
  		FIND_PACKAGE(OpenGL REQUIRED)
  		INCLUDE_directories("${OPENGL_INCLUDE_DIR}/Headers")
  		SET(INCLUDE_DIRS ${INCLUDE_DIRS} "${OPENGL_INCLUDE_DIR}/Headers")
  		SET(LIBS ${LIBS} ${OPENGL_LIBRARIES})
  		Message(${OPENGL_INCLUDE_DIR})
  		Message(${OPENGL_LIBRARIES})
  		IF(WITH_XFORMS)
  			FIND_PACKAGE(GLU REQUIRED)
  			INCLUDE_directories(${GLU_INCLUDE_DIR})
  			SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GLU_INCLUDE_DIR})
  			SET(LIBS ${LIBS} ${GLU_LIBRARIES})
  			FIND_PACKAGE(GL REQUIRED)
  			INCLUDE_directories(${GL_INCLUDE_DIR})
  			SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GL_INCLUDE_DIR})
  			SET(LIBS ${LIBS} ${GL_LIBRARIES})
  		ENDIF(WITH_XFORMS)
	ELSE(GL_NATIVE)
		FIND_PACKAGE(GLU REQUIRED)
  		INCLUDE_directories(${GLU_INCLUDE_DIR})
  		SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GLU_INCLUDE_DIR})
  		SET(LIBS ${LIBS} ${GLU_LIBRARIES})
  		FIND_PACKAGE(GL REQUIRED)
  		INCLUDE_directories(${GL_INCLUDE_DIR})
  		SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GL_INCLUDE_DIR})
  		SET(LIBS ${LIBS} ${GL_LIBRARIES})
	ENDIF(GL_NATIVE)
ELSE(APPLE)
  FIND_PACKAGE(OpenGL REQUIRED)
  INCLUDE_directories(${OPENGL_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${OPENGL_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${OPENGL_LIBRARIES})

  FIND_PACKAGE(GLU REQUIRED)
  INCLUDE_directories(${GLU_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GLU_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${GLU_LIBRARIES})
ENDIF(APPLE)

#------------------- softMotion -----------------------------------------------	

pkg_check_modules(SOFTMOTION REQUIRED "softMotion-libs")

INCLUDE_DIRECTORIES(${SOFTMOTION_INCLUDE_DIRS})
SET(LIBS ${LIBS} ${SOFTMOTION_LIBRARIES})
LINK_DIRECTORIES( ${SOFTMOTION_LIBRARY_DIRS} )
MESSAGE( SOFTMOTION_LIBRARIES = ${SOFTMOTION_LIBRARIES})

#------------------------------------------------------------------------------	
			
#MESSAGE( LIBS = ${LIBS} )
#MESSAGE( MOVE3D_LDFLAGS =  "${MOVE3D_LDFLAGS}" )
MESSAGE( MOVE3D_CFLAGS =  "${MOVE3D_CFLAGS}" )

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

#Default mode is Debug
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Debug CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
ENDIF(NOT CMAKE_BUILD_TYPE)

# Get the current move3d-studio version
ADD_DEFINITIONS(${MOVE3D_CFLAGS} -DMOVE3D_STUDIO_VERSION="${PACKAGE_VERSION}" )

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


