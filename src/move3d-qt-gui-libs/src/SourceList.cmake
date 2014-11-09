SET(BM3D_MODULE_NAME src/)

IF(QT_LIBRARY)

BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})

BM3D_SRC_SUBDIR_PROCESS(
planner_handler.cpp
move3d.c
)

IF(USE_GLUT)
BM3D_SRC_SUBDIR_PROCESS(
glutWindow.cpp
)
ENDIF()

BM3D_QT_GENERATE_MOC(
planner_handler.hpp
)

IF(QT_UI_XML_FILES)

include(${PROJECT_SOURCE_DIR}/src/qtBase/SourceList.cmake)
include(${PROJECT_SOURCE_DIR}/src/qtFormRobot/SourceList.cmake)
include(${PROJECT_SOURCE_DIR}/src/qtMainInterface/SourceList.cmake)

include(${PROJECT_SOURCE_DIR}/src/qtPlot/SourceList.cmake)
include(${PROJECT_SOURCE_DIR}/src/qtOpenGL/SourceList.cmake)
include(${PROJECT_SOURCE_DIR}/src/utils/SourceList.cmake)

ENDIF()
ENDIF()
