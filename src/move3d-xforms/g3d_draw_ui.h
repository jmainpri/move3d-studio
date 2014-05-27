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

#include "forms.h"


#ifndef __CEXTRACT__
extern void g3d_create_form(FL_FORM** form, int w, int h, int type);
extern int g3d_create_labelframe(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame);
extern int g3d_create_button(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame);
extern int g3d_create_choice(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame);
extern int g3d_create_checkbutton(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame);
extern int g3d_create_frame(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame);
extern int g3d_create_input(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame);
extern int g3d_create_valslider(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame);
extern int g3d_create_box(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame);
extern int g3d_create_counter(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame);
extern int g3d_create_xyplot(FL_OBJECT** obj, int type, FL_Coord w, FL_Coord h, const char *label, void** parent, int onRootFrame);
extern int g3d_refresh_form(FL_FORM** form);
extern void g3d_fl_free_object(FL_OBJECT* obj);
extern void g3d_fl_free_form(FL_FORM* obj);
#endif /* __CEXTRACT__ */
