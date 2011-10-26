SET(BM3D_MODULE_NAME_TMP ${BM3D_MODULE_NAME})
SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME}/qtFormRobot)

BM3D_SRC_SUBDIR_PROCESS(
moverobot.cpp
qtmovinghuman.cpp
qtConstraints.cpp
sliderfunction.cpp
)
BM3D_INC_DIR_PROCESS (${BM3D_MODULE_NAME})

BM3D_QT_GENERATE_MOC(
moverobot.hpp
qtmovinghuman.hpp
qtConstraints.hpp
sliderfunction.hpp
ui_moverobot.hpp
)

IF(MULTILOCALPATH)
BM3D_QT_GENERATE_MOC(
qtMultiLocalPath.hpp
)
BM3D_SRC_SUBDIR_PROCESS(
qtMultiLocalPath.cpp
)
ENDIF(MULTILOCALPATH)

#BM3D_QT_GENERATE_UI_HEADERS(moverobot.ui)
BM3D_QT_GENERATE_UI_HEADERS(qtmovinghuman.ui)

SET(BM3D_MODULE_NAME ${BM3D_MODULE_NAME_TMP})
