/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
#include "../perspective.h"

extern void g3d_draw_rob_pos_area ( void );
extern void g3d_draw_obj_pos_area ( p3d_obj *objPt );
extern void g3d_draw_srchball_pos_area ( psp_searchball *srchballpt );
extern int p3d_is_pos_area_showed ( p3d_rob *r );
extern int p3d_show_rob_pos_area ( p3d_rob *r );
extern int p3d_hide_rob_pos_area ( p3d_rob *r );
extern int p3d_set_robot_pos_area ( p3d_rob *r, double min, double max, double angle );
extern int p3d_set_allhumans_standard_pos_area ( p3d_rob *r );
extern int p3d_is_in_pos_area ( p3d_rob *r, double x, double y, int isrand );
extern int p3d_is_in_obj_pos_area ( p3d_obj *o, double x, double y );
extern double linearDistance ( double x1, double y1, double x2, double y2 );
extern double rad_angleOf ( double x1, double y1, double x2, double y2 );
extern double rad_angleOf_PIMED ( double x1, double y1, double x2, double y2 );
extern double rad2_angleOf ( double x1, double y1, double x2, double y2 );
extern double get_robot_angle_rad ( p3d_rob *r );

