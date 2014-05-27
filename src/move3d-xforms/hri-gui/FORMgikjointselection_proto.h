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
/*
 *  FORMgikjointselection_proto.h
 *  XcodeBioMove3D
 *
 *  Created by Akin Sisbot on 25/6/09.
 *
 */

#ifndef __CEXTRACT__

#include "forms.h"

extern void g3d_create_gik_jointsel_form ( void );
extern void g3d_show_gik_jointsel_form ( void );
extern void g3d_hide_gik_jointsel_form ( void );
extern void g3d_delete_gik_jointsel_form ( void );
extern void CB_gik_target_robot_obj(FL_OBJECT *obj, long arg);
extern void CB_gik_vis_obj(FL_OBJECT *obj, long arg);
extern void CB_gik_precision_obj(FL_OBJECT *obj, long arg);
extern void CB_gik_step_obj(FL_OBJECT *obj, long arg);
extern void CB_gik_run_obj(FL_OBJECT *obj, long arg);

#endif /* __CEXTRACT__ */

