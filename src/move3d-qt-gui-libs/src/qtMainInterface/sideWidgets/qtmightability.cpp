#include "qtmightability.hpp"
#include "ui_qtmightability.h"


#include <Util-pkg.h>
#include <P3d-pkg.h>
#include <Planner-pkg.h>
#include <Localpath-pkg.h>
#include <Collision-pkg.h>
#include <Graphic-pkg.h>

#include <libmove3d/hri/hri.h>

using namespace std;

extern int SHOW_TASKABILITIES;
extern show_taskability_params curr_params_for_show_taskability;//It includes params for manipulability also

qtMightability::qtMightability(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::qtMightability)
{
    ui->setupUi(this);
}

qtMightability::~qtMightability()
{
    delete ui;
}

void qtMightability::changeEvent(QEvent *e)
{
    QWidget::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

void qtMightability::on_pushButtonInitialiaze_clicked()
{
    cout << "##########################################" << endl;
    cout << "Begin the initailization" << endl;
    cout << "##########################################" << endl;
//    p3d_env * env = (p3d_env *) p3d_get_desc_curid(P3D_ENV);
//    int dimx,dimy,dimz;
//    int i;
//    double objx, objy, objz;

    GLOBAL_AGENTS = hri_create_agents();
    hri_assign_source_agent((char*)"JIDOKUKA", GLOBAL_AGENTS);

    GLOBAL_ENTITIES = hri_create_entities();
    hri_link_agents_with_entities(GLOBAL_ENTITIES, GLOBAL_AGENTS);
    hri_initialize_all_agents_knowledge(GLOBAL_ENTITIES, GLOBAL_AGENTS);

//    int SELECTED_BTSET = ui->SpinBoxSELECTED_BTSET->value();

//    /* NAVIGATION */
//    if(SELECTED_BTSET==1){
//      if(BTSET != NULL)
//        hri_bt_destroy_bitmapset(BTSET);
//
//      dimx = (int)((env->box.x2 - env->box.x1)/BT_SAMPLING);
//      dimy = (int)((env->box.y2 - env->box.y1)/BT_SAMPLING);
//      dimz = 1;
//      BTSET = hri_bt_create_bitmaps();
//      hri_bt_init_bitmaps(BTSET,dimx,dimy,dimz,BT_SAMPLING);
//      hri_bt_change_bitmap_position(BTSET,env->box.x1,env->box.y1, BTSET->robot->joints[ROBOTj_BASE]->dof_data[2].v);
//
//      ACBTSET = BTSET;
//      fl_set_object_color(BT_MOTION_INIT_OBJ,FL_GREEN,FL_COL1);
//    }
//    /* MANIPULATION */
//    if(SELECTED_BTSET==2){
//      if(INTERPOINT != NULL)
//        hri_bt_destroy_bitmapset(INTERPOINT);
//
//      INTERPOINT = hri_exp_init();
//      HRI_GIK = hri_gik_create_gik();
//      ACBTSET = INTERPOINT;
//      fl_set_object_color(BT_MOTION_INIT_OBJ,FL_GREEN,FL_COL1);
//    }
//    /* OBJECT REACH */
//    if(SELECTED_BTSET==3){
//      if(OBJSET != NULL)
//        hri_bt_destroy_bitmapset(OBJSET);
//
//      for(i=0; i<env->nr; i++){
//        if( strcasestr(env->robot[i]->name,"BOTTLE") )
//          break;
//      }
//      if(i==env->nr){
//        printf("No bottle in the environment\n");
//        return;
//      }
//      objx =  env->robot[i]->joints[1]->abs_pos[0][3];
//      objy =  env->robot[i]->joints[1]->abs_pos[1][3];
//      objz =  env->robot[i]->joints[1]->abs_pos[2][3];
//
//      OBJSET = hri_object_reach_init(objx,objy,objz);
//      ACBTSET = OBJSET;
//      fl_set_object_color(BT_MOTION_INIT_OBJ,FL_GREEN,FL_COL1);
//    }
//
//    if(!HUMAN_FORM_CREATED){
//      for(i=0; i<ACBTSET->human_no; i++)
//        fl_addto_choice(BT_HUMAN_ACTUAL_OBJ, ACBTSET->human[i]->HumanPt->name);
//      if(ACBTSET->human_no > 0){
//        for(i=0; i<ACBTSET->human[ACBTSET->actual_human]->states_no; i++)
//          fl_addto_choice(BT_HUMAN_STATE_OBJ, ACBTSET->human[ACBTSET->actual_human]->state[i].name);
//        fl_addto_choice(BT_HUMAN_EXISTS_OBJ,"not exist");
//        fl_addto_choice(BT_HUMAN_EXISTS_OBJ,"exist");
//      }
//      HUMAN_FORM_CREATED = TRUE;
//    } else {
//      fl_set_choice(BT_HUMAN_EXISTS_OBJ, 1); // 0 =nothing, 1 =not exist
//    }
//    fl_set_button(BT_SHOWBT_GNUPLOT_OBJ,0);
//    fl_set_button(BT_SHOWBT_DIST_OBJ,0);
//    fl_set_button(BT_SHOWBT_VIS_OBJ,0);
//    fl_set_button(BT_SHOWBT_HZAC_OBJ,0);
//    fl_set_button(BT_SHOWBT_OBS_OBJ,0);
//    fl_set_button(BT_SHOWBT_COMB_OBJ,0);
//    fl_set_button(BT_SHOWBT_PATH_OBJ,0);
//
//    CB_motion_obj(BT_MOTION_INIT_OBJ, SELECTED_BTSET);

    cout << "##########################################" << endl;
    cout << "End of the initailization" << endl;
    cout << "##########################################" << endl;

}


void qtMightability::on_checkBox_toggled(bool checked)
{
    if (checked)
    {

        cout << "##########################################" << endl;
        cout << "Show taskabilities" << endl;
        cout << "##########################################" << endl;

        curr_params_for_show_taskability.show_all_taskability_graph=1;

        curr_params_for_show_taskability.show_taskability_node=0;
        curr_params_for_show_taskability.show_TN_by_node_ID=0;
        curr_params_for_show_taskability.show_TN_by_agent=0;

        SHOW_TASKABILITIES=1;

//      curr_params_for_show_taskability.MN_ID=CURRENT_MANIPULABILITY_NODE_ID_TO_SHOW;
//      curr_params_for_show_taskability.TN_ID=CURRENT_TASKABILITY_NODE_ID_TO_SHOW;
//      curr_params_for_show_taskability.MN_perf_ag=CURRENT_TASK_PERFORMED_BY;
//       curr_params_for_show_taskability.MN_targ_obj=CURRENT_OBJECT_TO_MANIPULATE_INDEX;
//       curr_params_for_show_taskability.TN_perf_ag=CURRENT_TASK_PERFORMED_BY;
//       curr_params_for_show_taskability.TN_targ_ag=CURRENT_TASK_PERFORMED_FOR;
//       curr_params_for_show_taskability.TN_task=CURRENT_HRI_MANIPULATION_TASK;

//       g3d_draw_allwin_active();

    }
    else
    {
        cout << "##########################################" << endl;
        cout << "Hide taskabilities" << endl;
        cout << "##########################################" << endl;

        SHOW_TASKABILITIES = 0;
    }

}

void qtMightability::on_pushButtonInitialiazeCreateAgents_clicked()
{
    cout << "##########################################" << endl;
    cout << "begin Create Agents For MA_ASA" << endl;
    cout << "##########################################" << endl;

#ifdef MIGHTABILITY_MAPS
    init_agents_for_MA_and_ASA();
#else
    cout<< "Flag MIGHTABILITY_MAPS not on" <<endl;
#endif

    cout << "##########################################" << endl;
    cout << "End of Create Agents For MA_ASA" << endl;
    cout << "##########################################" << endl;
}

void qtMightability::on_pushButtonInitialiazePrepareState_clicked()
{

    cout << "##########################################" << endl;
    cout << "Begin Prepare For State Analysis" << endl;
    cout << "##########################################" << endl;


    char threshold_file_path[150];
    char *home_dir=getenv("HOME");
    cout << " Home directory is" << home_dir << endl;
    strcpy(threshold_file_path,home_dir);
    char threshold_file_name[50]="/Human_State_Analysis_Thresholds.txt";
    strcat(threshold_file_path,threshold_file_name);

    //"/home/akpandey/Human_State_Analysis_Thresholds.txt";
    prepare_for_Agent_State_Analysis(threshold_file_path);

    cout << "##########################################" << endl;
    cout << "End of Prepare For State Analysis" << endl;
    cout << "##########################################" << endl;
}

void qtMightability::on_pushButtonInitialiazeComputeMM_clicked()
{
    cout << "##########################################" << endl;
    cout << "Begin Compute MMs" << endl;
    cout << "##########################################" << endl;


    ////XFORM_update_func=fl_check_forms;
     ////char MM_around_object[50]="HRP2TABLE";
     char MM_around_object[50]="TABLE_4";
      /////char MM_around_object[50]="IKEA_SHELF";
//     default_drawtraj_fct_ptr=default_drawtraj_fct_with_XFORM;
#ifdef MIGHTABILITY_MAPS
    int MA_init_res=Create_and_init_Mightability_Maps(MM_around_object);
#else
    cout<< "Flag MIGHTABILITY_MAPS not on" <<endl;
#endif
//    fflush();
    fflush(stdout);

//    if(MA_init_res==1)
//    {
//    add_agents_for_HRI_task();
//    add_objects_for_HRI_task();
//
//      add_effort_levels_for_agents();
//    }

    cout << "##########################################" << endl;
    cout << "End of Compute MMs" << endl;
    cout << "##########################################" << endl;
}

void qtMightability::on_pushButtonTaskability_clicked()
{
    cout << "##########################################" << endl;
    cout << "Begin compute Taskalities" << endl;
    cout << "##########################################" << endl;

#ifdef MIGHTABILITY_MAPS
    find_taskability_graph();
    print_taskability_graph();
#else
    cout<< "Flag MIGHTABILITY_MAPS not on"<<endl;
#endif
//    update_taskability_graph_list();

    cout << "##########################################" << endl;
    cout << "End of compute Taskalities" << endl;
    cout << "##########################################" << endl;
}
