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
#include "P3d-pkg.h"
#include "Localpath-pkg.h"

#include <move3d-gui.h>

extern FL_OBJECT  *steering_obj;

FL_FORM *STEERING_FORM;
static FL_OBJECT  *GROUP;
static FL_OBJECT  *obj1;
static FL_OBJECT  *obj2;
FL_OBJECT  **BUTTON_TAB_OBJ;

static void CB_button_tab_obj(FL_OBJECT *ob, long arg)
{
  p3d_local_set_planner((p3d_localpath_type) arg);
}  


void g3d_create_steering_form(void)
{int i;
 double s;

  s = P3D_NB_LOCAL_PLANNER*20.0 + 20;

  BUTTON_TAB_OBJ = (FL_OBJECT  **) malloc(P3D_NB_LOCAL_PLANNER*sizeof(FL_OBJECT  *));

  STEERING_FORM = fl_bgn_form(FL_UP_BOX,200.0,s+30);

  obj1 = fl_add_frame(FL_ENGRAVED_FRAME,10,15,180,s,""); 
  obj2 = fl_add_box(FL_FLAT_BOX,55,10,90,10,"Steering methods");

  GROUP = fl_bgn_group();

  for(i=0;i<P3D_NB_LOCAL_PLANNER;i++){
    BUTTON_TAB_OBJ[i] = fl_add_checkbutton(FL_RADIO_BUTTON,45,25+i*20.0,55,25,p3d_local_getname_planner((p3d_localpath_type) i));
    fl_set_object_color(BUTTON_TAB_OBJ[i],FL_MCOL,FL_GREEN);
    fl_set_call_back(BUTTON_TAB_OBJ[i],CB_button_tab_obj,i);
  }

  fl_set_button(BUTTON_TAB_OBJ[p3d_local_get_planner()],1);

  //GROUP = 
  fl_end_group();

  fl_end_form();
}


/*****************************************************************/

/* fonctions de destruction des objets forms */
void g3d_delete_steering_form(void)
{int n,i;

 n = 4; 

  if(fl_get_button(steering_obj)){fl_hide_form(STEERING_FORM);}

  fl_free_object(obj1);
  fl_free_object(obj2);

  for(i=0;i<n;i++){
    BUTTON_TAB_OBJ[i]->u_vdata = NULL;
    BUTTON_TAB_OBJ[i]->next = NULL;
    BUTTON_TAB_OBJ[i]->prev = NULL;
    fl_free_object(BUTTON_TAB_OBJ[i]);
  }
  GROUP->next = NULL;
  GROUP->prev = NULL;
  fl_free_object(GROUP);
  fl_free_form(STEERING_FORM);

  free(BUTTON_TAB_OBJ);
}



/*************************************************************************/
