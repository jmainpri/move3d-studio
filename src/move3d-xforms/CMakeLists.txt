
IF(HRI_GUI)
	MESSAGE("Building move3d-studio with HRI support")
	ADD_SUBDIRECTORY(hri-gui)
ENDIF()

FOREACH (srcfile 
	FORMconstraints.cpp 
	FORMdiffusion.cpp 
	FORMenv.cpp 
	FORMfilter.cpp 
	FORMmain.cpp 
	FORMmobcam.cpp 
	FORMoptim.cpp 
	FORMpath_deform.cpp 
	FORMplanner.cpp 
	FORMrobot.cpp 
	FORMsteering.cpp
	FORMmultiLocalPath.cpp 
	FORMsoftMotion.cpp
	FORMgenom.cpp
	g3d_draw_ui.cpp
	g3d_window.cpp
	FORMgraspPlanning.cpp
	FORMuser_appli.cpp
	user_appli.cpp
	)
	LIST(APPEND MOVE3DSTUDIO_SRC ${srcfile})
ENDFOREACH(srcfile)

ADD_LIBRARY(move3d-xforms STATIC ${MOVE3DSTUDIO_SRC})
TARGET_LINK_LIBRARIES(move3d-xforms ${LIBS} ${COMPONENTS})

SET(COMPONENTS ${COMPONENTS} move3d-xforms PARENT_SCOPE)
