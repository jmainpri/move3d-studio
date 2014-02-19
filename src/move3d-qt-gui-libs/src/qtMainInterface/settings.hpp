//
//  settings.h
//  move3d-studio
//
//  Created by Jim Mainprice on 27/10/11.
//  Copyright 2011 LAAS-CNRS. All rights reserved.
//

#ifndef MOVE3D_STUDIO_SETTINGS
#define MOVE3D_STUDIO_SETTINGS

#include <string>

// Parameters for algorithms
void qt_saveInterfaceParameters(bool print, std::string fileName);
void qt_loadInterfaceParameters(bool print, std::string fileName, bool opengl = true);

#endif
