SET(BM3D_MODULE_NAME_TMP2 ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/sideWidgets)

BM3D_SRC_SUBDIR_PROCESS(
qtCost.cpp
qtMotionPlanner.cpp
qtUtil.cpp
qtRobot.cpp
qtDistanceField.cpp
qtRRTStar.cpp
qtTrajectorySampling.cpp
)

IF(LIGHT_PLANNER AND MULTILOCALPATH)
BM3D_SRC_SUBDIR_PROCESS(
qtReplanning.cpp
qtGraspPlanner.cpp
)
ENDIF()

BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})

BM3D_QT_GENERATE_MOC(
qtCost.hpp
qtMotionPlanner.hpp
qtUtil.hpp
qtRobot.hpp
qtDistanceField.hpp
qtRRTStar.hpp
qtTrajectorySampling.hpp
)

IF(LIGHT_PLANNER AND MULTILOCALPATH)
BM3D_QT_GENERATE_MOC(
qtReplanning.hpp
qtGraspPlanner.hpp
)
ENDIF()

BM3D_QT_GENERATE_UI_HEADERS(
qtCost.ui
qtMotionPlanner.ui
qtUtil.ui
qtRobot.ui
qtDistanceField.ui
qtRRTStar.ui
qtTrajectorySampling.ui
)

IF(LIGHT_PLANNER AND MULTILOCALPATH)
BM3D_QT_GENERATE_UI_HEADERS(
qtGraspPlanner.ui
qtReplanning.ui
)
ENDIF()

IF(HRI_COSTSPACE)
BM3D_QT_GENERATE_MOC(
qtHrics.hpp
qtOtp.hpp
qtNatural.hpp
qtHriGesture.hpp
)
BM3D_QT_GENERATE_UI_HEADERS(
qtHrics.ui
qtOtp.ui
qtNatural.ui
qtHriGesture.ui
)
BM3D_SRC_SUBDIR_PROCESS(
qtHrics.cpp
qtOtp.cpp
qtNatural.cpp
qtHriGesture.cpp
)
ENDIF(HRI_COSTSPACE)


IF(MIGHTABILITY_MAPS)
BM3D_QT_GENERATE_MOC( qtmightability.hpp )
BM3D_QT_GENERATE_UI_HEADERS( qtmightability.ui )
BM3D_SRC_SUBDIR_PROCESS( qtmightability.cpp )
ENDIF(MIGHTABILITY_MAPS)


SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP2})

