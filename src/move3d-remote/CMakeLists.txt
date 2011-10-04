#
# Copyright (c) 2010 LAAS/CNRS
# All rights reserved.
#
# Permission to use, copy, modify, and distribute this software for any purpose
# with or without   fee is hereby granted, provided   that the above  copyright
# notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
# REGARD TO THIS  SOFTWARE INCLUDING ALL  IMPLIED WARRANTIES OF MERCHANTABILITY
# AND FITNESS. IN NO EVENT SHALL THE AUTHOR  BE LIABLE FOR ANY SPECIAL, DIRECT,
# INDIRECT, OR CONSEQUENTIAL DAMAGES OR  ANY DAMAGES WHATSOEVER RESULTING  FROM
# LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
# OTHER TORTIOUS ACTION,   ARISING OUT OF OR IN    CONNECTION WITH THE USE   OR
# PERFORMANCE OF THIS SOFTWARE.
#
#                                            Xavier BROQUERE on Fri Aug 26 2011

CMAKE_MINIMUM_REQUIRED (VERSION 2.6.4)

PROJECT (MOVE3D_REMOTE CXX)

file(GLOB source_files main-remote.cpp move3d.c mainwindow-remote.cpp posterreader.cpp genomposter.cpp genomimageposter.cpp myopencv.cpp qtipl.cpp picowebimage.cpp)

file(GLOB image_files images/green-man-xpm.h   images/orange-man-xpm.h   images/red-man-xpm.h   images/yellow-death-xpm.h   images/yellow-man-xpm.h)

file(GLOB header_files duplicatedGenomStruct.hpp main-remote.hpp mainwindow-remote.hpp posterreader.hpp genomposter.hpp genomimageposter.hpp myopencv.hpp qtipl.hpp picowebimage.hpp )

file(GLOB ui_files mainwindow-remote.ui)

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src/move3d-qt-gui-libs/src)
INCLUDE_DIRECTORIES(/home/xbroquer/openrobots/include/pom)
INCLUDE_DIRECTORIES(/home/xbroquer/openrobots/include/viam)


# --- Scripts to search source files -------

SET(Move3DGUI_CompilationFlags "")
SET(Move3DGUI_LinkedCompilationFlags "")

MACRO(Move3DGUI_flag flagName description status)
  SET(flagName "${flagName}")
  SET(description "${description}")
  SET(status "${status}")
  OPTION(${flagName} ${description} ${status})
  LIST(APPEND Move3DGUI_CompilationFlags ${flagName})

  FOREACH(linkFlag ${ARGN})
    LIST(APPEND Move3DGUI_LinkedCompilationFlags "${flagName}:${linkFlag}")
  ENDFOREACH(linkFlag)
#  set_property(CACHE ANIMATION PROPERTY VALUE ON)
ENDMACRO(Move3DGUI_flag)

MACRO(Move3DGUI_CheckCompilationFlags)
  SET(Move3DGUI_ActiveFlags "")
  #MESSAGE ("Active Flags :")
  FILE(WRITE ${CMAKE_CURRENT_BINARY_DIR}/smpFlags "#Generated File Do not modify!!!\n" )
  FILE(APPEND ${CMAKE_CURRENT_BINARY_DIR}/smpFlags "Move3DGUI_COMPILATION_FLAGS = ")

  FOREACH(flag ${Move3DGUI_CompilationFlags})

    IF(${flag})
      FILE(APPEND ${CMAKE_CURRENT_BINARY_DIR}/Move3DGUIFlags "-D${flag} ")
      #MESSAGE("${flag}")
      ADD_DEFINITIONS("-D${flag}")
      LIST(APPEND Move3DGUI_ActiveFlags ${flag})
    ENDIF()
  ENDFOREACH(flag)
ENDMACRO(Move3DGUI_CheckCompilationFlags)


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

Move3D_flag(HRI_COSTSPACE "With the hri work" ON)

Move3D_flag(PQP "With PQP" ON)

Move3D_CheckCompilationFlags()




#################################################################
#### Add here a library to include (custom FindPackage.cmake ####
#### are in MOVE3D_REMOTE/.cmakeScripts. Use the template to     ####
####Â create a custom one)                                    ####
#################################################################
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/script/CMake")

FIND_PACKAGE(MOVE3D-PLANNERS REQUIRED)
INCLUDE_directories(${MOVE3D-PLANNERS_INCLUDE_DIR})
LINK_DIRECTORIES( ${MOVE3D-PLANNERS_LIBRARY_DIRS} )
SET(LIBS ${LIBS} ${MOVE3D-PLANNERS_LIBRARIES})



#-------------- Qt Library -----------------
IF(QT_LIBRARY)
  FIND_PACKAGE(Qt4 REQUIRED)
  INCLUDE_DIRECTORIES(${QT_INCLUDE_DIR})
  SET(Move3DGUI_COMPILATION_INCLUDES ${Move3DGUI_COMPILATION_INCLUDES} ${QT_INCLUDE_DIR})
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
  		#Message(${OPENGL_INCLUDE_DIR})
  		#Message(${OPENGL_LIBRARIES})
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



#-------------- Pocolibs -------------------------------------------------------

Message(Find Pocolibs)
FIND_PACKAGE (PkgConfig)
pkg_check_modules(POCOLIBS REQUIRED "pocolibs")

MESSAGE( "------------------ POCOLIBS -----------------------" )
MESSAGE( "${POCOLIBS_CFLAGS}" )
MESSAGE( "${POCOLIBS_LDFLAGS}" )


# -------------------------------------------------------------
# --- QT4 SUBDIR PROCESS (same but for preprocessing qt4 files)
# -------------------------------------------------------------
SET(SMP_QT_FILES_FOR_MOC "")
MACRO(SMP_QT_GENERATE_MOC)
  FOREACH(file ${ARGV})
    LIST(APPEND SMP_QT_FILES_FOR_MOC "${file}")
  ENDFOREACH(file ${ARGV})
ENDMACRO(SMP_QT_GENERATE_MOC)

SET(SMP_QT_UI_FILES "")
MACRO(SMP_QT_GENERATE_UI_HEADERS)
  FOREACH(file ${ARGV})
    LIST(APPEND SMP_QT_UI_FILES "${file}")
  ENDFOREACH(file ${ARGV})
ENDMACRO(SMP_QT_GENERATE_UI_HEADERS)


# -------------------------------------------------------------
# --- Directories to include ----------------------------------
# -------------------------------------------------------------
SMP_QT_GENERATE_MOC(${header_files})
SMP_QT_GENERATE_UI_HEADERS(${ui_files})

# necessary for the UI headers
BM3D_INC_DIR_PROCESS(${CMAKE_CURRENT_BINARY_DIR})
# generate the UI headers
QT4_WRAP_UI(SMP_QT_UI_HEADERS ${SMP_QT_UI_FILES})
# Qt4 files that need to be preprocessed
QT4_WRAP_CPP(SMP_QT_MOC ${SMP_QT_FILES_FOR_MOC})



# -------------------------------------------------------------
# --- Compiler and Compilation flags --------------------------
# -------------------------------------------------------------
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

SET(EXECUTABLE_OUTPUT_PATH "${CMAKE_BUILD_TYPE}/bin/$ENV{HOSTTYPE}")
SET(LIBRARY_OUTPUT_PATH "${CMAKE_BUILD_TYPE}/lib/$ENV{HOSTTYPE}")

#Default Flags
#ADD_DEFINITIONS(-DKCD_MOVE3D -DPRINT_ERROR -DPRINT_INFO)

IF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  ADD_DEFINITIONS(-g -Wall -Wextra -Wunused -Wno-unknown-pragmas)
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  ADD_DEFINITIONS(-O2 -Wall -Wunused -Wno-unknown-pragmas)
ENDIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")


ADD_EXECUTABLE(move3d-remote ${source_files} ${SMP_QT_MOC} ${SMP_QT_UI_HEADERS})
SET_TARGET_PROPERTIES(move3d-remote PROPERTIES LINKER_LANGUAGE CXX)
SET_TARGET_PROPERTIES(move3d-remote PROPERTIES COMPILE_FLAGS ${POCOLIBS_CFLAGS})
TARGET_LINK_LIBRARIES(move3d-remote Move3D-Qt-Gui ${POCOLIBS_LDFLAGS} ${LIBS} ${QT_QTNETWORK_LIBRARY} m)


# -------------------------------------------------------------
# --- Installation --------------------------------------------
# -------------------------------------------------------------



IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "move3d-remote install prefix" FORCE)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
INSTALL(TARGETS move3d-remote
        RUNTIME DESTINATION bin PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_WRITE GROUP_EXECUTE WORLD_READ WORLD_EXECUTE CONFIGURATIONS ${CMAKE_BUILD_TYPE})


