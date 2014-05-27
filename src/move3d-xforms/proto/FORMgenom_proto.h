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

#if defined(GRASP_PLANNING) && defined(MULTILOCALPATH)
extern void g3d_create_genom_form ( void );
extern void g3d_show_genom_form ( void );
extern void g3d_hide_genom_form ( void );
extern void g3d_delete_genom_form ( void );
// extern void genomCleanRoadmap(p3d_rob* robotPt);
// extern int genomArmGotoQ(p3d_rob* robotPt, int cartesian, char* objectName, int lp[], Gb_q6 positions[], int *nbPositions);
// extern int genomSetArmQ(p3d_rob *robot, double q1, double q2, double q3, double q4, double q5, double q6);
// extern int genomGetArmQ(p3d_rob *robot, double *q1, double *q2, double *q3, double *q4, double *q5, double *q6);
// extern int genomSetArmX(p3d_rob *robotPt, double x, double y, double z, double rx, double ry, double rz);
// extern int genomGetArmX(p3d_rob *robotPt, double *x, double *y, double *z, double *rx, double *ry, double *rz);
extern int genomSetInterfaceQuality();
// extern int genomFindSimpleGraspConfiguration(p3d_rob *robotPt, char *object_name, double *q1, double *q2, double *q3, double *q4, double *q5, double *q6);
// extern int genomArmComputePRM(p3d_rob* robotPt, int cartesian);
extern int genomCheckCollisionOnTraj(p3d_rob* robotPt, int cartesian, double* armConfig, int currentLpId, int lp[], Gb_q6 positions[],  int *nbPositions);
extern int genomGetCollideStatus(int status);
// extern int genomSetFreeflyerPose(p3d_rob *robotPt, double x, double y, double z, double rx, double ry, double rz);
// extern int genomSetFreeflyerPoseByName(char *name, double x, double y, double z, double rx, double ry, double rz)
// ;
// extern int genomGetFreeflyerPose(char *object_name, p3d_matrix4 pose);
// 
// extern int genomComputeGraspList(p3d_rob *hand_robotPt, char *object_name);
// extern int genomFindGraspConfiguration(p3d_rob *robotPt, p3d_rob *hand_robotPt, char *object_name, double *q1, double *q2, double *q3, double *q4, double *q5, double *q6);
// extern int genomFindPregraspAndGraspConfiguration(p3d_rob *robotPt, p3d_rob *hand_robotPt, char *object_name, double distance, double *pre_q1, double *pre_q2, double *pre_q3, double *pre_q4, double *pre_q5, double *pre_q6, double *q1, double *q2, double *q3, double *q4, double *q5, double *q6);
// 
// extern int genomPickUp_gotoObject(p3d_rob* robotPt, p3d_rob* hand_robotPt, char* objectName,  int cartesian, int lp[], Gb_q6 positions[], int *nbPositions);
// 
// extern int genomReleaseObject(p3d_rob* robotPt);
// extern int genomGrabObject(p3d_rob* robotPt, char* objectName);
// 
// extern int genomPrintConstraintInfo(p3d_rob *robotPt);
// extern p3d_obj * genomGetObjectByName(char *object_name);
// extern int genomSetObjectPoseWrtEndEffector(p3d_rob *robotPt, p3d_rob *object, double x, double y, double z, double rx, double ry, double rz);
// extern int genomDynamicGrasping(char *robot_name, char *hand_robot_name, char *object_name) ;

#ifdef DPG
extern int genomCheckCollisionOnTraj(p3d_rob* robotPt, int cartesian, int currentLpId);
extern int genomReplanCollidingTraj(p3d_rob* robotPt, int cartesian, double* armConfig, int currentLpId, int lp[], Gb_q6 positions[],  int *nbPositions);
#endif

#endif
