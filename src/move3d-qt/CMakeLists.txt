CMAKE_MINIMUM_REQUIRED (VERSION 2.6)
PROJECT (src CXX)

# ------------------------------------------------------------------------------
# --- Default compilation option : ---------------------------------------------
# --- DO NOT MODIFY!!! use ccmake ----------------------------------------------
#-------------------------------------------------------------------------------
#-------------- Modules --------------------------------------------------------
#-------------------------------------------------------------------------------
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/script/CMake")

#-------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
# --- Scripts to search source files -------------------------------------------
# ------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
SET(Move3D_CompilationFlags "")
SET(Move3D_LinkedCompilationFlags "")

MACRO(Move3D_flag flagName description status)
  SET(flagName "${flagName}")
  SET(description "${description}")
  SET(status "${status}")
  OPTION(${flagName} ${description} ${status})
  LIST(APPEND Move3D_CompilationFlags ${flagName})
  FOREACH(linkFlag ${ARGN})
    LIST(APPEND Move3D_LinkedCompilationFlags "${flagName}:${linkFlag}")
  ENDFOREACH(linkFlag)
#  set_property(CACHE ANIMATION PROPERTY VALUE ON)
ENDMACRO(Move3D_flag)

MACRO(Move3D_CheckCompilationFlags)
  SET(Move3D_ActiveFlags "")
  MESSAGE ("Active Flags :")
  FILE(WRITE ${CMAKE_CURRENT_BINARY_DIR}/smpFlags "#Generated File Do not modify!!!\n" )
  FILE(APPEND ${CMAKE_CURRENT_BINARY_DIR}/smpFlags "Move3D_COMPILATION_FLAGS = ")
  FOREACH(flag ${Move3D_CompilationFlags})
    IF(${flag})
      FILE(APPEND ${CMAKE_CURRENT_BINARY_DIR}/Move3DFlags "-D${flag} ")
      MESSAGE("${flag}")
      ADD_DEFINITIONS("-D${flag}")
      LIST(APPEND Move3D_ActiveFlags ${flag})
    ENDIF()
  ENDFOREACH(flag)
ENDMACRO(Move3D_CheckCompilationFlags)

#-------------------------------------------------------------------------------
# ------------------------------------------------------------------------------
# --- Compilation Flags --------------------------------------------------------
# ------------------------------------------------------------------------------
#-------------------------------------------------------------------------------

Move3D_flag(QT_LIBRARY "Sets QtLibrary compiling mode" ON)
Move3D_flag(QT_GL "Sets QtLibrary compiling mode" ON QT_LIBRARY)
Move3D_flag(QT_UI_XML_FILES "Sets QtLibrary compiling mode" ON QT_LIBRARY)

Move3D_flag(P3D_CONSTRAINTS "Compilation with module Constraints" ON)
Move3D_flag(P3D_COLLISION_CHECKING "Compilation with module Collision Checking" ON)
Move3D_flag(P3D_LOCALPATH "Compilation with module Localpath" ON)
Move3D_flag(P3D_PLANNER "Compilation with module Planner" ON)

Move3D_flag(LIGHT_PLANNER "Compilation with module Light Planner" ON)
Move3D_flag(GRASP_PLANNING "Compilation with module Gasp Planner" ON)
Move3D_flag(MULTILOCALPATH "Compilation with module Multi-local Planner" ON)
Move3D_flag(MULTIGRAPH "Compilation with module Multi-Graph" ON)

Move3D_flag(USE_GBM "Compilation with module GBM for IK computation" ON)
Move3D_flag(USE_GLUT "Glut" OFF)

Move3D_flag(MOVE3D_CORE "Compiles the planning API and planners" ON)

Move3D_flag(HRI_COSTSPACE "With the hri work" OFF)

Move3D_flag(PQP "With PQP" ON)

Move3D_CheckCompilationFlags()

#-------------- Qt Library -----------------------------------------------------
IF(QT_LIBRARY)
  FIND_PACKAGE(Qt4 REQUIRED)
  INCLUDE_DIRECTORIES(${QT_INCLUDE_DIR})
  SET(Move3D_COMPILATION_INCLUDES ${Move3D_COMPILATION_INCLUDES} ${QT_INCLUDE_DIR})
  IF(QT_GL)
    SET(QT_USE_QTOPENGL TRUE)
  ENDIF(QT_GL)
  INCLUDE(${QT_USE_FILE})
  SET(LIBS ${LIBS} ${QT_LIBRARIES})
ENDIF(QT_LIBRARY)
FIND_PACKAGE(X11 REQUIRED)
INCLUDE_directories(${X11_INCLUDE_DIR})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${X11_INCLUDE_DIR})
SET(LIBS ${LIBS} ${X11_LIBRARIES})
FIND_PACKAGE(LibXml2 REQUIRED)
INCLUDE_directories(${LIBXML2_INCLUDE_DIR})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${LIBXML2_INCLUDE_DIR})
SET(LIBS ${LIBS} ${LIBXML2_LIBRARIES})
IF(APPLE)
	IF(QT_LIBRARY)
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
	ENDIF(QT_LIBRARY)
ELSE(APPLE)
  FIND_PACKAGE(OpenGL REQUIRED)
  INCLUDE_directories(${OPENGL_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${OPENGL_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${OPENGL_LIBRARIES})
ENDIF(APPLE)


# ------------------- libmove3D-viewer ----------------------------------------------

PKG_CHECK_MODULES(MOVE3D REQUIRED libmove3d-viewer)
INCLUDE_DIRECTORIES(${MOVE3D_INCLUDE_DIRS})
LINK_DIRECTORIES( ${MOVE3D_LIBRARY_DIRS} )
SET(LIBS ${LIBS} ${MOVE3D_LIBRARIES})
ADD_DEFINITIONS(${MOVE3D_CFLAGS})

#MESSAGE( MOVE3D_LIBRARIES = ${MOVE3D_LIBRARIES})

#-------------- Move3D-motionPlanner-libs --------------------------------------

FIND_PACKAGE(MOVE3D-MOTIONPLANNER REQUIRED)
INCLUDE_directories(${MOVE3D-MOTIONPLANNER_INCLUDE_DIR})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${MOVE3D-MOTIONPLANNER_INCLUDE_DIR})
SET(LIBS ${LIBS} ${MOVE3D-MOTIONPLANNER_LIBRARIES})

#-------------- Move3D-Qt-Gui-libs --------------------------------------

FIND_PACKAGE(MOVE3D-QTGUI REQUIRED)
INCLUDE_directories(${MOVE3D-QTGUI_INCLUDE_DIR})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${MOVE3D-QTGUI_INCLUDE_DIR})
SET(LIBS ${LIBS} ${MOVE3D-QTGUI_LIBRARIES})

message(1 LIBS:)
message(${LIBS})

#message(2 LIBS:)
#message(${LIBS})

#message(Includes:)
#message(${INCLUDE_DIRS})
#----------------------------------------------------------------------

SET(CMAKE_CXX_COMPILER g++)
SET(CMAKE_C_COMPILER g++)

IF(UNIX)
  ADD_DEFINITIONS(-DUNIX)
  IF(APPLE)
    ADD_DEFINITIONS(-DMACOSX -x c++)
    SET (CMAKE_SHARED_LINKER_FLAGS "-dynamiclib -Wl,-headerpad_max_install_names,-undefined,dynamic_lookup")
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

#SET(EXECUTABLE_OUTPUT_PATH "${BioMove3D_BINARY_DIR}/${CMAKE_BUILD_TYPE}/bin/$ENV{HOSTTYPE}")
#SET(LIBRARY_OUTPUT_PATH "${BioMove3D_BINARY_DIR}/${CMAKE_BUILD_TYPE}/lib/$ENV{HOSTTYPE}")

#Default Flags
ADD_DEFINITIONS(-DKCD_MOVE3D -DPRINT_ERROR -DPRINT_INFO)

IF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  ADD_DEFINITIONS(-g -Wall -Wno-unknown-pragmas -fstack-protector-all)
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  ADD_DEFINITIONS(-O2 -Wall -Wno-unknown-pragmas)
ENDIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")


#-------------- Eigen ----------------------------------------------------------

FIND_PACKAGE(EIGEN REQUIRED)
INCLUDE_directories(${Eigen_INCLUDE_DIR})
SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${Eigen_INCLUDE_DIR})

#-------------- Glut -----------------------------------------------------------

IF(USE_GLUT)
FIND_PACKAGE (GLUT REQUIRED glut)
INCLUDE_directories (${GLUT_INCLUDE_DIR})
SET (LIBS ${LIBS} ${GLUT_glut_LIBRARY})
ENDIF()

#-------------- Robotpim -------------------------------------------------------

#  FIND_PACKAGE(ROBOPTIM REQUIRED)
#  INCLUDE_directories(${ROBOPTIM_INCLUDE_DIR})
#  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${ROBOPTIM_INCLUDE_DIR})
#  SET(LIBS ${LIBS} ${ROBOPTIM_LIBRARIES})

#-------------- Boost ----------------------------------------------------------

# boost is used e.g. for managing the cost functions (T-RRT).
#  FIND_PACKAGE(Boost REQUIRED)
#  INCLUDE_directories(${BOOST_INCLUDE_DIR})
#  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${BOOST_INCLUDE_DIR})

# boost is used e.g. for managing the cost functions (T-RRT).
FIND_PACKAGE(Boost REQUIRED)
INCLUDE_directories(/opt/local/include/)        
SET(INCLUDE_DIRS ${INCLUDE_DIRS} /opt/local/include/ ${BOOST_INCLUDE_DIRS})

#-------------- GRASP PLANNER --------------------------------------------------
#-------------------------------------------------------------------------------
#IF(GRASP_PLANNING)
#  FIND_PACKAGE(GLPK REQUIRED)
#  INCLUDE_directories(${GLPK_INCLUDE_DIR})
#  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GLPK_INCLUDE_DIR})
#  SET(LIBS ${LIBS} ${GLPK_LIBRARIES})
#
#  FIND_PACKAGE(QHULL REQUIRED)
#  INCLUDE_directories(${QHULL_INCLUDE_DIR})
#  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${QHULL_INCLUDE_DIR})
#  SET(LIBS ${LIBS} ${QHULL_LIBRARIES})
#
#  FIND_PACKAGE(GLIBCONFIG REQUIRED)
#  INCLUDE_directories(${GLIBCONFIG_INCLUDE_DIR})
#  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GLIBCONFIG_INCLUDE_DIR})
#
#  FIND_PACKAGE(GLIB REQUIRED)
#  INCLUDE_directories(${GLIB_INCLUDE_DIR})
#  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GLIB_INCLUDE_DIR})
#  SET(LIBS ${LIBS} ${GLIB_LIBRARIES})
#
#  FIND_PACKAGE(GTS REQUIRED)
#  INCLUDE_directories(${GTS_INCLUDE_DIR})
#  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GTS_INCLUDE_DIR})
#  SET(LIBS ${LIBS} ${GTS_LIBRARIES})
#ENDIF(GRASP_PLANNING)

#-------------- MULTILOCALPATH -------------------------------------------------
#-------------------------------------------------------------------------------
IF(MULTILOCALPATH)
  FIND_PACKAGE(SOFTMOTION REQUIRED)
  INCLUDE_directories(${SOFTMOTION_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${SOFTMOTION_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${SOFTMOTION_LIBRARIES})
ENDIF(MULTILOCALPATH)

#-------------- GBM ------------------------------------------------------------
#-------------------------------------------------------------------------------
IF(USE_GBM)
  FIND_PACKAGE(GBM REQUIRED)
  INCLUDE_directories(${GBM_INCLUDE_DIR})
  SET(INCLUDE_DIRS ${INCLUDE_DIRS} ${GBM_INCLUDE_DIR})
  SET(LIBS ${LIBS} ${GBM_LIBRARIES})
ENDIF(USE_GBM)


IF(QT_LIBRARY)
QT4_WRAP_CPP(MOC_OUTFILES main.hpp)
ADD_EXECUTABLE(Move3D main.cpp move3d.c ${MOC_OUTFILES})
ELSE()
ADD_EXECUTABLE(Move3D main.cpp move3d.c)
ENDIF()


SET_TARGET_PROPERTIES(Move3D PROPERTIES LINKER_LANGUAGE CXX)
TARGET_LINK_LIBRARIES(Move3D ${LIBS} m)

# -------------------------------------------------------------
# --- Installation --------------------------------------------
# -------------------------------------------------------------
IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX "${BioMove3D_BINARY_DIR}/install" CACHE PATH "BioMove3D install prefix" FORCE)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)


INSTALL(TARGETS Move3D
        RUNTIME DESTINATION bin PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE CONFIGURATIONS ${CMAKE_BUILD_TYPE}
        LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE GROUP_READ GROUP_WRITE WORLD_READ CONFIGURATIONS ${CMAKE_BUILD_TYPE})

# -------------------------------------------------------------
# --- Uninstall -----------------------------------------------
# -------------------------------------------------------------
#CONFIGURE_FILE(
#  "${CMAKE_CURRENT_SOURCE_DIR}/script/CMake/cmake_uninstall.cmake.in"
#  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
#  IMMEDIATE @ONLY)
#ADD_CUSTOM_TARGET(uninstall
#  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")
