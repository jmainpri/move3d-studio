# - Check for the presence of MOVE3D-QTGUI
#
# The following variables are set when BioMove3D is found:
#  HAVE_BioMove3D       = Set to true, if all components of BioMove3D
#                          have been found.
#  BioMove3D_INCLUDE_DIR   = Include path for the header files of BioMove3D
#  BioMove3D_LIBRARIES  = Link these to use ooMove3d-motionPlanner

## -----------------------------------------------------------------------------
## Check for the header files

find_path (MOVE3D-QTGUI_INCLUDE_DIR qtLibrary.hpp
 PATHS 
$ENV{HOME}/workspace/Move3D-Qt-Gui-libs/src 
$ENV{HOME}/Devel/Move3D-Qt-Gui-libs/src 
$ENV{ROBOTPKG_BASE}/include/Move3D-Qt-Gui/src/
$ENV{MOVE3D_INSTALL_DIR}/include/Move3D-Qt-Gui/src/
 )

find_library (MOVE3D-QTGUI_LIBRARIES Move3D-Qt-Gui
  PATHS ${MOVE3D-QTGUI_LIB} 
$ENV{HOME}/workspace/Move3D-Qt-Gui-libs/lib/$ENV{HOSTTYPE} 
$ENV{HOME}/Devel/Move3D-Qt-Gui-libs/lib/$ENV{HOSTTYPE} 
$ENV{ROBOTPKG_BASE}/lib
$ENV{MOVE3D_INSTALL_DIR}/lib
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (MOVE3D-QTGUI_INCLUDE_DIR AND MOVE3D-QTGUI_LIBRARIES)
 set (HAVE_MOVE3D-QTGUI TRUE)
else (MOVE3D-QTGUI_INCLUDE_DIR)
 if (NOT MOVE3D-QTGUI_FIND_QUIETLY)
   if (NOT (MOVE3D-QTGUI_INCLUDE_DIR))
     message (STATUS "WARNING : Unable to find Move3D-Qt-Gui header files !")
   endif (NOT (MOVE3D-QTGUI_INCLUDE_DIR))
 endif (NOT MOVE3D-QTGUI_FIND_QUIETLY)
endif (MOVE3D-QTGUI_INCLUDE_DIR AND MOVE3D-QTGUI_LIBRARIES)

if (HAVE_MOVE3D-QTGUI)
 if (NOT MOVE3D-QTGUI_FIND_QUIETLY)
   message (STATUS "Found components for Move3D-Qt-Gui")
   message (STATUS "MOVE3D-QTGUI_INCLUDE_DIR = ${MOVE3D-QTGUI_INCLUDE_DIR}")
   message (STATUS "MOVE3D-QTGUI_LIBRARIES = ${MOVE3D-QTGUI_LIBRARIES}")
 endif (NOT MOVE3D-QTGUI_FIND_QUIETLY)
else (HAVE_MOVE3D-QTGUI)
 if (MOVE3D-QTGUI_FIND_REQUIRED)
   message (FATAL_ERROR "Could not find Move3D-Qt-Gui!")
 endif (MOVE3D-QTGUI_FIND_REQUIRED)
endif (HAVE_MOVE3D-QTGUI)

#mark_as_advanced (
# HAVE_MOVE3D-QTGUI
# MOVE3D-QTGUI_INCLUDE_DIR
# MOVE3D-QTGUI_LIBRARIES
# MOVE3D-QTGUI_SOURCE_DIR
# )
