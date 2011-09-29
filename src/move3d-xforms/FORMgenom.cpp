#include <list>
#include <string>
#include <iostream>

//#include <Manipulation.h>
#include <Collision-pkg.h>
#include <Graphic-pkg.h>
#include <Util-pkg.h>
#include <P3d-pkg.h>
#include <Planner-pkg.h>
#include <Localpath-pkg.h>
#include <GraspPlanning-pkg.h>
#include <gbM/Proto_gbModeles.h>
//#include <lightPlanner.h>
//#include <lightPlannerApi.h>
#include <lightPlanner/proto/ManipulationViaConfPlanner.hpp>
#include <LightPlanner-pkg.h>
#include <graspPlanning/proto/gp_grasp_generation_proto.h>
#ifdef DPG
#include <p3d_chanEnv_proto.h>
#endif

#ifdef CXX_PLANNER
#include <planner_cxx/plannerFunctions.hpp>
#endif

#include "proto/FORMrobot_proto.h"

// #if defined(MULTILOCALPATH) && defined(GRASP_PLANNING) && defined(LIGHT_PLANNER)

//#define OBJECT_NAME "DUPLO_OBJECT"
//#define OBJECT_NAME "WOODEN_OBJECT"
// #define OBJECT_NAME "SURPRISE_BOX"
#define OBJECT_NAME "GREY_TAPE"
//#define OBJECT_NAME "YELLOW_BOTTLE"
// #define OBJECT_NAME "LOTR_TAPE"
// #define OBJECT_NAME "GREY_K7"

// #define SUPPORT_NAME "HRP2TABLE"
#define SUPPORT_NAME "IKEA_SHELF"
//#define SUPPORT_NAME "SHELF"

// #define PLACEMENT_NAME "IKEA_SHELF"
#define PLACEMENT_NAME "HRP2TABLE"


#define HUMAN_NAME "ACHILE_HUMAN1"
#define CAMERA_JNT_NAME "Tilt"
#define CAMERA_FOV 80.0

static ManipulationPlanner *manipulation= NULL;

static double QCUR[6]= {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static double QGOAL[6]= {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static double XCUR[6]= {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static double XGOAL[6]= {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


static int FORMGENOM_CARTESIAN = 0;
static int FORMGENOM_OBJECTGRABED = 0;
/* --------- FORM VARIABLES ------- */
FL_FORM  * GENOM_FORM = NULL;
static FL_OBJECT * GENOMGROUP = NULL;


static FL_OBJECT * BT_ARM_GOTO_Q_OBJ = NULL;
static FL_OBJECT * BT_ARM_EXTRACT_OBJ = NULL;
static FL_OBJECT * BT_ARM_GOTO_X_OBJ = NULL;
static FL_OBJECT * BT_SIMPLE_GRASP_PLANNER_OBJ = NULL;
static FL_OBJECT * BT_FIND_GRASP_CONFIG_AND_COMP_TRAJ_OBJ = NULL;
static FL_OBJECT * BT_COMP_TRAJ_CONFIGS_OBJ = NULL;
static FL_OBJECT  *INPUT_OBJ1, *INPUT_OBJ2 = NULL;
static FL_FORM *INPUT_FORM = NULL;
static FL_OBJECT *DELETE_GRAPHS = NULL;
static FL_OBJECT * BT_SET_CARTESIAN = NULL;
static FL_OBJECT * BT_GRAB_OBJECT = NULL;
static FL_OBJECT * BT_RELEASE_OBJECT = NULL;

static FL_OBJECT * BT_PICK_UP_TAKE = NULL;
static FL_OBJECT * BT_TAKE_TO_PLACE = NULL;
static FL_OBJECT * BT_PICK_UP_TAKE_XYZ = NULL;
static FL_OBJECT * BT_PLACE = NULL;
static FL_OBJECT * BT_CONSTRUCTPRM = NULL;
static FL_OBJECT * BT_ESCAPE_OBJECT = NULL;
static FL_OBJECT * BT_PLANVIAPOINTS = NULL;
static FL_OBJECT * BT_SEQUENCE = NULL;

#ifdef DPG
static FL_OBJECT * BT_CHECKCOLONTRAJ = NULL;
static FL_OBJECT * BT_REPLANCOLTRAJ = NULL;
#endif
/* ---------- FUNCTION DECLARATIONS --------- */
static void initManipulationGenom();
static void g3d_create_genom_group(void);
static void genomDraw();
static void genomKey();

static void CB_genomArmGotoQ_obj(FL_OBJECT *obj, long arg);
static void CB_genomArmExtract_obj(FL_OBJECT *obj, long arg);
static void CB_genomArmGotoX_obj(FL_OBJECT *obj, long arg);
static void CB_genomFindSimpleGraspConfiguration_obj(FL_OBJECT *obj, long arg);

static void CB_genomPickUp_gotoObject(FL_OBJECT *obj, long arg);

static void CB_genomCleanRoadmap_obj(FL_OBJECT *obj, long arg);
static void CB_genomArmComputePRM_obj(FL_OBJECT *obj, long arg);
static void CB_set_cartesian(FL_OBJECT *obj, long arg);
static void CB_grab_object(FL_OBJECT *obj, long arg);
static void CB_release_object(FL_OBJECT *obj, long arg);

static void CB_genomPickUp_takeObject(FL_OBJECT *obj, long arg);
static void CB_genomPickUp_takeObjectToXYZ(FL_OBJECT *obj, long arg);
static void CB_genomTakeToPlace(FL_OBJECT *obj, long arg);
static void CB_genomPlaceObject(FL_OBJECT *obj, long arg);
static void CB_genomEscapeObject(FL_OBJECT *obj, long arg);

static void CB_genomPlanTrajViaPoints(FL_OBJECT *obj, long arg);
static void CB_genomSequence(FL_OBJECT *obj, long arg);
#ifdef DPG
static void CB_checkColOnTraj(FL_OBJECT *obj, long arg);
static void CB_replanColTraj(FL_OBJECT *obj, long arg);
#endif
/* -------------------- MAIN FORM CREATION GROUP --------------------- */
void g3d_create_genom_form(void) {
	GENOM_FORM = fl_bgn_form(FL_UP_BOX, 150, 890);
	g3d_create_genom_group();
	fl_end_form();
}

void g3d_show_genom_form(void) {
	fl_show_form(GENOM_FORM, FL_PLACE_SIZE, TRUE, "Genom Requests");
}

void g3d_hide_genom_form(void) {
	fl_hide_form(GENOM_FORM);
}

void g3d_delete_genom_form(void) {
	fl_free_form(GENOM_FORM);
}

/* -------------------- MAIN GROUP --------------------- */
static void initManipulationGenom() {
  if (manipulation == NULL) {
// 	p3d_rob * robotPt= p3d_get_robot_by_name("JIDOKUKA_ROBOT");
//  p3d_rob * robotPt= p3d_get_robot_by_name("PR2_ROBOT");
	manipulation= new ManipulationPlanner(XYZ_ROBOT);
//         manipulation->setArmType(GP_LWR); // set the arm type
  }
  return;
}

static void g3d_create_genom_group(void) {
	int x, y, dy, w, h;
	FL_OBJECT *obj;

	obj = fl_add_labelframe(FL_ENGRAVED_FRAME, 5, 15, 140, 870, "Genom Requests");

	GENOMGROUP = fl_bgn_group();

	x= 15;
	y= 30;
	w= 120;
	h= 40;
	dy= h + 10;

         BT_SET_CARTESIAN = fl_add_button(FL_RADIO_BUTTON, x, y, w, h, "Set Cartesian");
	fl_set_call_back(BT_SET_CARTESIAN, CB_set_cartesian, 1);
	fl_set_object_color(BT_SET_CARTESIAN,FL_MCOL,FL_GREEN);
	fl_set_button(BT_SET_CARTESIAN, FALSE);


	 y+= dy;
         BT_GRAB_OBJECT = fl_add_button(FL_RADIO_BUTTON, x, y, w, h, "Grab Object");
	fl_set_call_back(BT_GRAB_OBJECT, CB_grab_object, 1);
	fl_set_object_color(BT_GRAB_OBJECT,FL_MCOL,FL_GREEN);
	fl_set_button(BT_GRAB_OBJECT, FALSE);

		 y+= dy;
         BT_RELEASE_OBJECT = fl_add_button(FL_RADIO_BUTTON, x, y, w, h, "Release Object");
	fl_set_call_back(BT_RELEASE_OBJECT, CB_release_object, 1);
	fl_set_object_color(BT_RELEASE_OBJECT,FL_MCOL,FL_GREEN);
	fl_set_button(BT_RELEASE_OBJECT, FALSE);
	
	y+= dy;
	DELETE_GRAPHS =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Delete Graphs");
	fl_set_call_back(DELETE_GRAPHS, CB_genomCleanRoadmap_obj, 1);

        y+= dy;
	BT_ARM_GOTO_Q_OBJ =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Arm Goto Q");
//	BT_PLOT_Q_WRITE_FILE_OBJ= fl_add_button(FL_RADIO_BUTTON, x, y + 1*dy, w, h, "Write file and plot qi");
	fl_set_call_back(BT_ARM_GOTO_Q_OBJ, CB_genomArmGotoQ_obj, 1);
  
        y+= dy;
  BT_ARM_EXTRACT_OBJ =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Arm Extract");
  fl_set_call_back(BT_ARM_EXTRACT_OBJ, CB_genomArmExtract_obj, 1);
  
        y+= dy;
  BT_ARM_GOTO_X_OBJ =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Arm Goto X");
//  BT_PLOT_Q_WRITE_FILE_OBJ= fl_add_button(FL_RADIO_BUTTON, x, y + 1*dy, w, h, "Write file and plot qi");
  fl_set_call_back(BT_ARM_GOTO_X_OBJ, CB_genomArmGotoX_obj, 1);
  
	y+= dy;
        BT_FIND_GRASP_CONFIG_AND_COMP_TRAJ_OBJ =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Pick Up");
        fl_set_call_back(BT_FIND_GRASP_CONFIG_AND_COMP_TRAJ_OBJ, CB_genomPickUp_gotoObject, 1);


	y+= dy;
        BT_PICK_UP_TAKE =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Take To Conf");
        fl_set_call_back(BT_PICK_UP_TAKE, CB_genomPickUp_takeObject, 1);

  y+= dy;
        BT_PICK_UP_TAKE_XYZ =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Take To XYZ");
        fl_set_call_back(BT_PICK_UP_TAKE_XYZ, CB_genomPickUp_takeObjectToXYZ, 1);
  y+= dy;
        BT_TAKE_TO_PLACE =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Take To Place");
        fl_set_call_back(BT_TAKE_TO_PLACE, CB_genomTakeToPlace, 1);
	y+= dy;
        BT_PLACE =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Place From Free");
        fl_set_call_back(BT_PLACE, CB_genomPlaceObject, 1);
  y+= dy;
        BT_ESCAPE_OBJECT =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Escape object");
        fl_set_call_back(BT_ESCAPE_OBJECT, CB_genomEscapeObject, 1);

  y+= dy;
  BT_CONSTRUCTPRM =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Build PRM");
  fl_set_call_back(BT_CONSTRUCTPRM, CB_genomArmComputePRM_obj, 0);


  y+= dy;
  BT_PLANVIAPOINTS =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Plan Via Conf traj");
  fl_set_call_back(BT_PLANVIAPOINTS, CB_genomPlanTrajViaPoints, 0);
  
  y+= dy;
  BT_SEQUENCE =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Sequence");
  fl_set_call_back(BT_SEQUENCE, CB_genomSequence, 0);
  
#ifdef DPG
  y+= dy;
  BT_CHECKCOLONTRAJ =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Check Col Traj");
  fl_set_call_back(BT_CHECKCOLONTRAJ, CB_checkColOnTraj, 0);
  y+= dy;
  BT_REPLANCOLTRAJ =  fl_add_button(FL_NORMAL_BUTTON, x, y, w, h, "Replan Col Traj");
  fl_set_call_back(BT_REPLANCOLTRAJ, CB_replanColTraj, 0);
  y+= dy;
#endif
  
  fl_end_group();
}

static void CB_cancel(FL_OBJECT *ob, long arg) {
  fl_hide_form(INPUT_FORM);
  fl_free_object(INPUT_OBJ1);
  fl_free_object(INPUT_OBJ2);
  fl_free_form(INPUT_FORM);
}


static void CB_genomArmGotoQ_obj(FL_OBJECT *obj, long arg) {

//	int nbPositions = 0;
	
	if (manipulation== NULL) {
	  initManipulationGenom();
	}

	if(FORMGENOM_CARTESIAN == 1) {
	  for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
      manipulation->setArmCartesian(i,true);
	  }
	} else {
	  for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
 	    manipulation->setArmCartesian(i,false);
	  }
	}
	
	std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
	std::vector <SM_TRAJ> smTrajs;
  std::vector <double>  objStart, objGoto;
  gpGrasp grasp;
	manipulation->armPlanTask(ARM_FREE,0,manipulation->robotStart(),manipulation->robotGoto(), objStart, objGoto, (char*)"" , (char*)"", (char*)"", grasp, confs, smTrajs);
        return;
}

static void CB_genomArmExtract_obj(FL_OBJECT *obj, long arg) {

//  int nbPositions = 0;

  if (manipulation== NULL) {
    initManipulationGenom();
  }

  if(FORMGENOM_CARTESIAN == 1) {
    for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
      manipulation->setArmCartesian(i,true);
    }
  } else {
    for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
      manipulation->setArmCartesian(i,false);
    }
  }

  std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
  std::vector <SM_TRAJ> smTrajs;
  std::vector <double>  objStart, objGoto;
  gpGrasp grasp;
  manipulation->armPlanTask(ARM_EXTRACT,0,manipulation->robotStart(),manipulation->robotGoto(), objStart, objGoto, (char*)"", (char*)"", (char*)"", grasp, confs, smTrajs);
        return;
}



//! Plans a path to go from the currently defined ROBOT_POS config to the specified end effector pose defined for the arm only.
//! End effector pose is given in world coordinates (lenghts in meters, angles in radians).
//! \return 0 in case of success, !=0 otherwise
static void CB_genomArmGotoX_obj(FL_OBJECT *obj, long arg)
{
  if (manipulation== NULL) {
	  initManipulationGenom();
  }

  if(FORMGENOM_CARTESIAN == 1) {
    for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
      manipulation->setArmCartesian(i,true);
    }
  } else {
    for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
      manipulation->setArmCartesian(i,false);
    }
  }

  std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
  std::vector <SM_TRAJ> smTrajs;
  std::vector <double>  objStart, objGoto;
  objStart.push_back(P3D_HUGE);
  objStart.push_back(P3D_HUGE);
  objStart.push_back(P3D_HUGE);
  objStart.push_back(P3D_HUGE);
  objStart.push_back(P3D_HUGE);
  objStart.push_back(P3D_HUGE);
    objGoto.push_back(4.63);
    objGoto.push_back(-2.78);
    objGoto.push_back(1.15);
  objGoto.push_back(P3D_HUGE);
  objGoto.push_back(P3D_HUGE);
  objGoto.push_back(P3D_HUGE);
  gpGrasp grasp;
  manipulation->armPlanTask(ARM_FREE,0,manipulation->robotStart(),manipulation->robotGoto(), objStart, objGoto, (char*)"", (char*)"", (char*)"", grasp, confs, smTrajs);
        return;
}

static void CB_genomCleanRoadmap_obj(FL_OBJECT *obj, long arg){
	if (manipulation== NULL) {
	  initManipulationGenom();
	}
       manipulation->cleanRoadmap();
       manipulation->cleanTraj();
}

static void CB_genomArmComputePRM_obj(FL_OBJECT *obj, long arg){
  if (manipulation== NULL) {
	  initManipulationGenom();
  }
  manipulation->armComputePRM(300);
}

#ifdef DPG
void CB_checkColOnTraj(FL_OBJECT *obj, long arg){
  if (manipulation== NULL) {
	  initManipulationGenom();
	}
  manipulation->checkCollisionOnTraj();
}
void CB_replanColTraj(FL_OBJECT *obj, long arg){
  if (manipulation== NULL) {
	  initManipulationGenom();
	}

  std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
  std::vector <SM_TRAJ> smTrajs;
  manipulation->replanCollidingTraj(0, confs, smTrajs );
}
#endif


static void CB_set_cartesian(FL_OBJECT *obj, long arg) {
	FORMGENOM_CARTESIAN= !FORMGENOM_CARTESIAN;

	if(FORMGENOM_CARTESIAN)
	{  fl_set_button(BT_SET_CARTESIAN, TRUE);
fl_set_object_color(BT_SET_CARTESIAN,FL_MCOL,FL_GREEN);
	}
	else
	{  fl_set_button(BT_SET_CARTESIAN, FALSE);
fl_set_object_color(BT_SET_CARTESIAN, FL_INACTIVE_COL,FL_INACTIVE_COL);
	}
	printf("cartesian = %d\n",FORMGENOM_CARTESIAN);
}

static void CB_grab_object(FL_OBJECT *obj, long arg) {
	fl_set_button(BT_GRAB_OBJECT, TRUE);
	fl_set_object_color(BT_GRAB_OBJECT,FL_MCOL,FL_GREEN);
	fl_set_button(BT_RELEASE_OBJECT, FALSE);
  fl_set_object_color(BT_RELEASE_OBJECT, FL_INACTIVE_COL,FL_INACTIVE_COL);

	if (manipulation== NULL) {
	  initManipulationGenom();
	}
	FORMGENOM_OBJECTGRABED = 1;
	p3d_set_object_to_carry_to_arm(XYZ_ROBOT, 0, (char*)OBJECT_NAME);
  

	printf("grab = %d\n",FORMGENOM_OBJECTGRABED);
	return;
}

static void CB_release_object(FL_OBJECT *obj, long arg) {
	fl_set_button(BT_GRAB_OBJECT, FALSE);
        fl_set_object_color(BT_GRAB_OBJECT, FL_INACTIVE_COL,FL_INACTIVE_COL);
	fl_set_button(BT_RELEASE_OBJECT, TRUE);
	fl_set_object_color(BT_RELEASE_OBJECT,FL_MCOL,FL_GREEN);
	
        FORMGENOM_OBJECTGRABED = 0;
	if (manipulation== NULL) {
	  initManipulationGenom();
	}
  p3d_set_object_to_carry_to_arm(XYZ_ROBOT, 0, NULL);
  XYZ_ROBOT->isCarryingObject = FALSE;
	printf("grab = %d\n",FORMGENOM_OBJECTGRABED);
	return;
}


void genomDraw()
{
 	if (manipulation== NULL) {
	  initManipulationGenom();
	}
// 	manipulation->draw(0);
}


void genomKey()
{
  if (manipulation== NULL) {
	  initManipulationGenom();
  }
//   manipulation->setCapture(!(manipulation->getCapture()));
//   printf("CAPTURE= %d\n", manipulation->getCapture());

//   manipulation->displayPlacements= !manipulation->displayPlacements;
//   manipulation->centerCamera();
  g3d_draw_allwin_active();
}


static void CB_genomFindSimpleGraspConfiguration_obj(FL_OBJECT *obj, long arg) {
//  double pre_q1, pre_q2, pre_q3, pre_q4, pre_q5, pre_q6;

 double q1, q2, q3, q4, q5, q6;
 q1= q2= q3= q4= q5= q6= 0.0;
 p3d_rob *curRobotPt= NULL, *robotPt = NULL, *hand_robotPt= NULL;

 	if (manipulation== NULL) {
	  initManipulationGenom();
	}

 curRobotPt=  (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
 robotPt= p3d_get_robot_by_name(GP_ROBOT_NAME);
// set the object
//  manipulation->findSimpleGraspConfiguration(&q1, &q2, &q3, &q4, &q5, &q6);

 hand_robotPt= p3d_get_robot_by_name(GP_GRIPPER_ROBOT_NAME);

//  genomFindGraspConfiguration(robotPt, hand_robotPt, OBJECT_NAME, &q1, &q2, &q3, &q4, &q5, &q6);
// genomFindPregraspAndGraspConfiguration(robotPt, hand_robotPt, (char*)OBJECT_NAME, 0.0, &pre_q1, &pre_q2, &pre_q3, &pre_q4, &pre_q5, &pre_q6, &q1, &q2, &q3, &q4, &q5, &q6);
// manipulation->findPregraspAndGraspConfiguration()
//     manipulation->setArmQ(q1, q2, q3, q4, q5, q6);
    g3d_win *win= NULL;
    win= g3d_get_cur_win();

    win->fct_draw2= &(genomDraw);
    win->fct_key2= &(genomKey);

    g3d_draw_allwin();
    g3d_draw_allwin_active();


 
   p3d_get_robot_config_into(robotPt, &robotPt->ROBOT_POS);
   XYZ_ENV->cur_robot= curRobotPt;
}



int genomSetInterfaceQuality() {
  g3d_win * win = NULL;
  win = g3d_get_cur_win();

  if(win->vs.displayFloor == FALSE) {
          win->vs.displayShadows = TRUE;
          win->vs.displayFloor = TRUE;
          win->vs.displayTiles = TRUE;
          win->vs.displayWalls = TRUE;
  } else {
          win->vs.displayShadows = FALSE;
          win->vs.displayFloor = FALSE;
          win->vs.displayTiles = FALSE;
          win->vs.displayWalls = FALSE;

  }
  return 0;
}


static void CB_genomGraspObject(FL_OBJECT *obj, long arg) {
     double distance = 0.1;
     configPt qgrasp = NULL, qpregrasp = NULL;

      if (manipulation== NULL) {
	  initManipulationGenom();
      }

       qgrasp = p3d_alloc_config(manipulation->robot());
       qpregrasp = p3d_alloc_config(manipulation->robot());
//        manipulation->setObjectToManipulate((char*)OBJECT_NAME);
/*       if(manipulation->isObjectGraspable(0, (char*)OBJECT_NAME) == false) {
	  std::cout << "this object is not graspable " << std::endl;
	  return;
       }*/

//        manipulation->findPregraspAndGraspConfiguration(0, distance, &qpregrasp, &qgrasp);

       p3d_set_and_update_this_robot_conf(manipulation->robot(), qgrasp);
//        manipulation->setArmQ(q1, q2, q3, q4, q5, q6);

       p3d_destroy_config (manipulation->robot(), qpregrasp );
       p3d_destroy_config (manipulation->robot(), qgrasp );
       g3d_draw_allwin_active();
}


static void CB_genomPickUp_gotoObject(FL_OBJECT *obj, long arg) {
    
//         double x, y, theta;
	if (manipulation== NULL) {
	  initManipulationGenom();
	}
  int armId = 0;
  if(FORMGENOM_CARTESIAN == 1) {
    for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
      if(armId == i){
        manipulation->setArmCartesian(i,true);
      }else{
        manipulation->setArmCartesian(i,false);
      }
    }
  } else {
    for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
      manipulation->setArmCartesian(i,false);
    }
  }

  std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
  std::vector <SM_TRAJ> smTrajs;
  std::vector <double>  objStart, objGoto;
  gpGrasp grasp;
  manipulation->armPlanTask(ARM_PICK_GOTO,0,manipulation->robotStart(), manipulation->robotGoto(), objStart, objGoto,(char*)OBJECT_NAME, (char*)"", (char*)"", grasp,  confs, smTrajs);

//         g3d_win *win= NULL;
//         win= g3d_get_cur_win();
//         win->fct_draw2= &(genomDraw);
//         win->fct_key1= &(genomKey);
// 	g3d_draw_allwin_active();
	return;
}

static void CB_genomPickUp_takeObject(FL_OBJECT *obj, long arg) {
	if (manipulation== NULL) {
	  initManipulationGenom();
	}

  if(FORMGENOM_CARTESIAN == 1) {
    for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
     manipulation->setArmCartesian(i,true);
    }
  } else {
    for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
      manipulation->setArmCartesian(i,false);
    }
  }
  
	std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
  std::vector<SM_TRAJ> smTrajs;
  std::vector<double> objStart, objGoto;
  for(unsigned int i = 0; i < 6; i++){
    objStart.push_back(P3D_HUGE);
    objGoto.push_back(P3D_HUGE);
  }
  gpGrasp grasp;
  manipulation->armPlanTask(ARM_TAKE_TO_FREE,0,manipulation->robotStart(), manipulation->robotGoto(), objStart, objGoto,(char*)OBJECT_NAME, (char*)SUPPORT_NAME, (char*)"", grasp, confs, smTrajs);

	g3d_draw_allwin_active();
	return;
}

static void CB_genomPickUp_takeObjectToXYZ(FL_OBJECT *obj, long arg) {
  if (manipulation== NULL) {
    initManipulationGenom();
  }

  if(FORMGENOM_CARTESIAN == 1) {
    for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
     manipulation->setArmCartesian(i,true);
    }
  } else {
    for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
      manipulation->setArmCartesian(i,false);
    }
  }
  
  std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
  std::vector<SM_TRAJ> smTrajs;
  std::vector<double> objStart, objGoto;
    objStart.push_back(P3D_HUGE);
    objStart.push_back(P3D_HUGE);
    objStart.push_back(P3D_HUGE);
    objStart.push_back(P3D_HUGE);
    objStart.push_back(P3D_HUGE);
    objStart.push_back(P3D_HUGE);
    objGoto.push_back(4.19);
    objGoto.push_back(-2.05);
    objGoto.push_back(1.2);
    objGoto.push_back(0.0);
    objGoto.push_back(0.0);
    objGoto.push_back(P3D_HUGE);
    gpGrasp grasp;
    manipulation->armPlanTask(ARM_TAKE_TO_FREE,0,manipulation->robotStart(), manipulation->robotGoto(), objStart, objGoto,(char*)OBJECT_NAME, (char*)SUPPORT_NAME, (char*)"", grasp, confs, smTrajs);

  g3d_draw_allwin_active();
  return;
}

static void CB_genomPlaceObject(FL_OBJECT *obj, long arg) {

  if (manipulation== NULL) {
    initManipulationGenom();
  }

  if(FORMGENOM_CARTESIAN == 1) {
    for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
     manipulation->setArmCartesian(i,true);
    }
  } else {
    for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
      manipulation->setArmCartesian(i,false);
    }
  }

  std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
  std::vector <SM_TRAJ> smTrajs;
  std::vector<double> objStart, objGoto;
  objGoto.push_back(4.39);
  objGoto.push_back(-2.25);
  objGoto.push_back(0.73);
  objGoto.push_back(0.0);
  objGoto.push_back(0.0);
  objGoto.push_back(P3D_HUGE);
  gpGrasp grasp;

  manipulation->armPlanTask(ARM_PLACE_FROM_FREE,0,manipulation->robotStart(), manipulation->robotGoto(), objStart, objGoto,(char*)OBJECT_NAME, (char*)"",(char*)PLACEMENT_NAME, grasp, confs, smTrajs);

  g3d_draw_allwin_active();

  return;
}

static void CB_genomTakeToPlace(FL_OBJECT *obj, long arg) {

  if (manipulation== NULL) {
    initManipulationGenom();
  }

  if(FORMGENOM_CARTESIAN == 1) {
    for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
     manipulation->setArmCartesian(i,true);
    }
  } else {
    for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
      manipulation->setArmCartesian(i,false);
    }
  }

  std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
  std::vector <SM_TRAJ> smTrajs;
  std::vector<double> objStart, objGoto;
  objGoto.push_back(4.39);
  objGoto.push_back(-2.25);
  objGoto.push_back(0.73);
  objGoto.push_back(0.0);
  objGoto.push_back(0.0);
  objGoto.push_back(P3D_HUGE);
  gpGrasp grasp;

  manipulation->armPlanTask(ARM_TAKE_TO_PLACE,0,manipulation->robotStart(), manipulation->robotGoto(), objStart, objGoto, (char*)OBJECT_NAME, (char*)SUPPORT_NAME,(char*)PLACEMENT_NAME, grasp, confs, smTrajs);

  g3d_draw_allwin_active();

  return;
}


static void CB_genomEscapeObject(FL_OBJECT *obj, long arg) {

  if (manipulation== NULL) {
    initManipulationGenom();
  }

  if(FORMGENOM_CARTESIAN == 1) {
    for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
     manipulation->setArmCartesian(i,true);
    }
  } else {
    for(int i=0; i<manipulation->robot()->armManipulationData->size(); i++) {
      manipulation->setArmCartesian(i,false);
    }
  }

  std::vector <MANPIPULATION_TRAJECTORY_CONF_STR> confs;
  std::vector <SM_TRAJ> smTrajs;
  std::vector<double> objStart, objGoto;

  manipulation->armPlanTask(ARM_ESCAPE_OBJECT,0,manipulation->robotStart(), manipulation->robotGoto(), objStart, objGoto,(char*)OBJECT_NAME, (char*)"", (char*)"", confs, smTrajs);

  g3d_draw_allwin_active();

  return;
}

static void CB_genomPlanTrajViaPoints(FL_OBJECT *obj, long arg) {

 p3d_rob * robotPt= XYZ_ROBOT;//p3d_get_robot_by_name("JIDOKUKA_ROBOT");//justin//JIDOKUKA_ROBOT
 ManipulationViaConfPlanner m_viaConfPlan(robotPt);
 std::vector<SM_TRAJ> smTrajs;
 
 if(FORMGENOM_CARTESIAN == 1) {
   for(int i=0; i<m_viaConfPlan.robot()->armManipulationData->size(); i++) {
     m_viaConfPlan.setArmCartesian(i,true);
   }
 } else {
   for(int i=0; i<m_viaConfPlan.robot()->armManipulationData->size(); i++) {
     m_viaConfPlan.setArmCartesian(i,false);
   }
 }
 m_viaConfPlan.planTrajFromConfigArrayInRobotTheForm(smTrajs);
}

static void CB_genomSequence(FL_OBJECT *obj, long arg) {

  if (manipulation== NULL) {
    initManipulationGenom();
  }
  
  fixAllJointsWithoutArm(XYZ_ROBOT, 0);
  
  CB_genomPickUp_gotoObject(obj, arg);
  p3d_copy_config_into(XYZ_ROBOT, XYZ_ROBOT->ROBOT_GOTO, &XYZ_ROBOT->ROBOT_POS);
  p3d_copy_config_into(XYZ_ROBOT, XYZ_ROBOT->conf[0]->q, &XYZ_ROBOT->ROBOT_GOTO);
 
  fixAllJointsWithoutArm(XYZ_ROBOT, 0);
  
  CB_genomPickUp_takeObject(obj, arg);
  p3d_copy_config_into(XYZ_ROBOT, XYZ_ROBOT->ROBOT_GOTO, &XYZ_ROBOT->ROBOT_POS);
  
  fixAllJointsWithoutArm(XYZ_ROBOT, 0);
  
  CB_genomPlaceObject(obj, arg);
  p3d_copy_config_into(XYZ_ROBOT, XYZ_ROBOT->ROBOT_GOTO, &XYZ_ROBOT->ROBOT_POS);

//   CB_genomTakeToPlace(obj, arg);
//   p3d_copy_config_into(XYZ_ROBOT, XYZ_ROBOT->ROBOT_GOTO, &XYZ_ROBOT->ROBOT_POS);
  
//   CB_genomEscapeObject(obj, arg);
//   p3d_copy_config_into(XYZ_ROBOT, XYZ_ROBOT->ROBOT_GOTO, &XYZ_ROBOT->ROBOT_POS);
//   p3d_copy_config_into(XYZ_ROBOT, XYZ_ROBOT->conf[0]->q, &XYZ_ROBOT->ROBOT_GOTO);
  
//   CB_genomArmGotoQ_obj(obj, arg);
//   p3d_copy_config_into(XYZ_ROBOT, XYZ_ROBOT->ROBOT_GOTO, &XYZ_ROBOT->ROBOT_POS);
  return;
}

// #endif


