# - Check for the presence of MOVE3D-MOTIONPLANNER
#
# The following variables are set when BioMove3D is found:
#  HAVE_BioMove3D       = Set to true, if all components of BioMove3D
#                          have been found.
#  BioMove3D_INCLUDE_DIR   = Include path for the header files of BioMove3D
#  BioMove3D_LIBRARIES  = Link these to use ooMove3d-motionPlanner

## -----------------------------------------------------------------------------
## Check for the header files

find_path (MOVE3D-MOTIONPLANNER_INCLUDE_DIR planner/planner.hpp
 PATHS $ENV{ROBOTPKG_BASE}/include/libmove3d/planners 
 )

find_library (MOVE3D-MOTIONPLANNER_LIBRARIES move3d-planners
  PATHS ${MOVE3D-MOTIONPLANNER_LIB}  $ENV{ROBOTPKG_BASE}/lib
  )

## -----------------------------------------------------------------------------
## Actions taken when all components have been found

if (MOVE3D-MOTIONPLANNER_INCLUDE_DIR AND MOVE3D-MOTIONPLANNER_LIBRARIES)
 set (HAVE_MOVE3D-MOTIONPLANNER TRUE)
else (MOVE3D-MOTIONPLANNER_INCLUDE_DIR)
 if (NOT MOVE3D-MOTIONPLANNER_FIND_QUIETLY)
   if (NOT (MOVE3D-MOTIONPLANNER_INCLUDE_DIR))
     message (STATUS "WARNING : Unable to find Move3d-motionPlanner header files !")
   endif (NOT (MOVE3D-MOTIONPLANNER_INCLUDE_DIR))
 endif (NOT MOVE3D-MOTIONPLANNER_FIND_QUIETLY)
endif (MOVE3D-MOTIONPLANNER_INCLUDE_DIR AND MOVE3D-MOTIONPLANNER_LIBRARIES)

if (HAVE_MOVE3D-MOTIONPLANNER)
 if (NOT MOVE3D-MOTIONPLANNER_FIND_QUIETLY)
   message (STATUS "Found components for Move3D-motionPlanner")
   message (STATUS "MOVE3D-MOTIONPLANNER_INCLUDE_DIR = ${MOVE3D-MOTIONPLANNER_INCLUDE_DIR}")
   message (STATUS "MOVE3D-MOTIONPLANNER_LIBRARIES = ${MOVE3D-MOTIONPLANNER_LIBRARIES}")
 endif (NOT MOVE3D-MOTIONPLANNER_FIND_QUIETLY)
else (HAVE_MOVE3D-MOTIONPLANNER)
 if (MOVE3D-MOTIONPLANNER_FIND_REQUIRED)
   message (FATAL_ERROR "Could not find Move3d-motionPlanner!")
 endif (MOVE3D-MOTIONPLANNER_FIND_REQUIRED)
endif (HAVE_MOVE3D-MOTIONPLANNER)

#mark_as_advanced (
# HAVE_MOVE3D-MOTIONPLANNER
# MOVE3D-MOTIONPLANNER_INCLUDE_DIR
# MOVE3D-MOTIONPLANNER_LIBRARIES
# MOVE3D-MOTIONPLANNER_SOURCE_DIR
# )
