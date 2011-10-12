//
//  sparkcamera.cpp
//  move3d-remote-novela
//
//  Created by Jim Mainprice on 12/10/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//

#include "sparkcamera.hpp"
#include "qtOpenGL/glwidget.hpp"
#include "P3d-pkg.h"

#include <iostream>

using namespace std;

void spark_camera_change(bool reset)
{
  G3D_Window *win = qt_get_cur_g3d_win();
  
  if( reset )
  {
    p3d_rob* r = (p3d_rob *) p3d_get_robot_by_name("PR2_ROBOT");
    p3d_jnt* j = r->joints[3];
    p3d_matrix4* m_transf = &(j->abs_pos);
    p3d_matrix4 T0,T1,Tbis,T2,T3,offset;
    
    p3d_mat4Copy(*m_transf,offset);
    //    p3d_mat4Pos(T1, 0, 0, 0, 0, 0.0, 0.4);
    p3d_mat4Pos(T0, 0, 0, 0.3, 0, 0.0, 0.0);
    p3d_mat4Pos(T1, 0, 0, 0, 0, 0.5, 0.0);
    p3d_mat4Pos(T2, -1.3, 0, 0, 0, 0, 0);
    p3d_mat4Mult(T0,T1,Tbis);
    p3d_mat4Mult(Tbis,T2,T3);
    p3d_mat4Mult(*m_transf,T3,offset);
    
    cout << "Change camera" << endl;
    g3d_save_win_camera(win->vs);
    g3d_set_camera_parameters_from_frame(offset, win->vs);
    g3d_set_projection_matrix(win->vs.projection_mode);
    qt_change_mob_frame(win,m_transf);
  }
  else
  {
    cout << "Reset camera" << endl;
    qt_reset_mob_frame(win);
    g3d_restore_win_camera(win->vs);
  }
}
