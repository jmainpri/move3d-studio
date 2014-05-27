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
 *    File generated automatically. Do not edit by hand.
 */
#ifndef __CEXTRACT__

#include "P3d-pkg.h"

extern void openChainPlannerOptions(void);
extern void closedChainPlannerOptions(void);

extern void viewTraj(void);
#ifdef DPG
extern int checkForCollidingLpAlongPath(void);
#endif
extern void showConfig(configPt conf);
extern void computeOfflineOpenChain(p3d_rob* robot, p3d_matrix4 objectInitPos);
extern void computeOfflineClosedChain(p3d_rob* robot, p3d_matrix4 objectInitPos);

/** ////////// MISC /////////////*/
extern void globalPlanner(void);
extern void findPath(void);
#ifdef MULTIGRAPH
extern void p3d_specificSuperGraphLearn(void);
extern void p3d_globalSuperGraphLearn(void);
#endif
extern void p3d_computeTests(void);

#ifdef DPG
extern int checkForColPath(p3d_rob* robot, p3d_traj* traj, p3d_graph* mainGraph, configPt current, p3d_localpath* currentLp, int optimized);
#endif
// extern void p3dAddTrajToGraph(p3d_rob* robot, p3d_graph* graph, p3d_traj* traj);
// extern p3d_node* p3d_addConfToGraph(p3d_rob* robot, p3d_graph* graph, configPt q, int* ikSol);
extern p3d_node* p3d_findInsertConnectTrajConfigInGraph(p3d_rob* robot, p3d_graph* graph, p3d_traj* traj, configPt q, p3d_localpath* currentLp);
extern p3d_localpath* p3d_findConfigLocalPathInTraj(p3d_rob* robot, p3d_traj* traj, configPt q);
/** ////////// MISC /////////////*/

extern void fixJoint(p3d_rob * robot, p3d_jnt * joint,  p3d_matrix4 initPos);
extern void unFixJoint(p3d_rob * robot, p3d_jnt * joint);

extern void nbLocalPathPerSecond(void);
extern void nbCollisionPerSecond(void);

#endif /* __CEXTRACT__ */
