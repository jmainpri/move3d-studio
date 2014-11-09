prefix=@CMAKE_INSTALL_PREFIX@
exec_prefix=@CMAKE_INSTALL_PREFIX@
libdir=@CMAKE_INSTALL_PREFIX@/lib
includedir=@CMAKE_INSTALL_PREFIX@/include

MPLibsFlags=@Move3DMPLIB_CompilationFlags@
MPLibsIncludes=@Move3DMPLIB_Compilation_includes@
MPLibsLibs=@Move3DMPLIB_Compilation_libs@
 
Name: move3d-qt-gui-libs
Description: Move3D Qt Gui - Headless library
Version: @BIOMOVE3D_VERSION@
Libs: ${MPLibsFlags} -L${libdir} -lMove3D-Qt-Gui
Cflags: -I${includedir}/Move3D-Qt-Gui/src ${MPLibsIncludes} ${MPLibsFlags}
