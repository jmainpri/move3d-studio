
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

FILE(GLOB_RECURSE INCLUDE_DIRS "include/*.h")

ADD_LIBRARY(move3d-xforms STATIC ${MOVE3DSTUDIO_SRC})
TARGET_LINK_LIBRARIES(move3d-xforms ${MOVE3D_LDFLAGS} ${LIBS})

ADD_EXECUTABLE(move3d-studio move3d.cpp ${INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(move3d-studio move3d-xforms)

INSTALL(TARGETS move3d-studio
        RUNTIME DESTINATION bin 
	CONFIGURATIONS ${CMAKE_BUILD_TYPE})
		
