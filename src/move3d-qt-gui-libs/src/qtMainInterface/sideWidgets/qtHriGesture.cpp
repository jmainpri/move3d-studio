/*
 *  qtCost.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "qtHriGesture.hpp"
#include "ui_qtHriGesture.h"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <boost/bind.hpp>
#include <sys/time.h>
#include <QMessageBox>

#include "qtBase/SpinBoxSliderConnector_p.hpp"

#include "qtLibrary.hpp"
#include "planner_handler.hpp"

#ifdef USE_QWT
#include "qtPlot/basicPlot.hpp"
#include "qtPlot/multiPlot.hpp"
#include "qtPlot/tempWin.hpp"
#endif

#include "qtMainInterface/mainwindow.hpp"
//#include "qtMainInterface/mainwindowGenerated.hpp"

#include "qtMotionPlanner.hpp"

#include "hri_costspace/HRICS_costspace.hpp"
#include "hri_costspace/gestures/HRICS_record_motion.hpp"
#include "hri_costspace/gestures/HRICS_workspace_occupancy.hpp"
#include "hri_costspace/gestures/HRICS_classify_motion.hpp"
#include "hri_costspace/gestures/HRICS_human_prediction_cost_space.hpp"
#include "hri_costspace/gestures/HRICS_gest_parameters.hpp"
#include "hri_costspace/HRICS_parameters.hpp"
#include "hri_costspace/human_trajectories/HRICS_ioc.hpp"

#include "planner/planEnvironment.hpp"
#include "planner/AStar/AStarPlanner.hpp"

#include "utils/ConfGenerator.h"
#include "API/project.hpp"
#include "API/ConfigSpace/configuration.hpp"

using namespace QtShiva;
using namespace Move3D;
using std::cout;
using std::endl;
MOVE3D_USING_SHARED_PTR_NAMESPACE

extern Eigen::Vector3d global_DrawnSphere;

HriGestureWidget::HriGestureWidget(QWidget *parent) :
    QWidget(parent),
    m_ui(new Ui::HriGestureWidget)
{
    m_ui->setupUi(this);
}

HriGestureWidget::~HriGestureWidget()
{
    delete m_ui;
    //#ifdef USE_QWT
    //    delete this->plot;
    //#endif
}

void HriGestureWidget::init()
{
    //this->initHRI();
    //this->initHumanLike();

    initRecordedMotion();
    initWorkspaceOccupancy();
    initGestureRecognition();
    initHriIOC();
    initLegibleCost();
    initClassification();
}

//--------------------------------------------------------------------
// Recorded motion
//--------------------------------------------------------------------
// How to :
// Run spark-genom and record the complete motion in n-xml-files
// Then load the motion and extract and save each motion in n-xml-files
// Load the folder and convert to CSV to make one single motion file
void HriGestureWidget::initRecordedMotion()
{
    //    global_motionRecorder = new HRICS::RecordMotion( "HERAKLES_HUMAN1" );
    connect(this, SIGNAL(selectedPlanner(QString)), global_plannerHandler, SLOT(startPlanner(QString)));

    new connectCheckBoxToEnv( m_ui->checkBoxInitGestureModule,  GestEnv->getObject(GestParam::init_module_at_start) );

    connect( m_ui->pushButtonLoadRecordedMotion,SIGNAL(clicked()),this,SLOT(loadRecordedMotion()));
    connect( m_ui->pushButtonShowRecordedMotion,SIGNAL(clicked()),this,SLOT(showRecordedMotion()));

    connect( m_ui->spinBoxShowConfigurationRecordedMotion, SIGNAL(valueChanged(int)), this, SLOT(getIthConfigurationInMotion()), Qt::QueuedConnection);
    connect( m_ui->pushButtonSetBeginRM,SIGNAL(clicked()),this,SLOT(setSourceRM()));
    connect( m_ui->pushButtonSetEndRM,SIGNAL(clicked()),this,SLOT(setTargetRM()));
    connect( m_ui->pushButtonExtarctAndSaveRM,SIGNAL(clicked()),this,SLOT(extractAndSaveRM()));
    connect( m_ui->pushButtonLoadFolder,SIGNAL(clicked()),this,SLOT(loadFolder()));
    connect( m_ui->pushButtonSaveToCSV,SIGNAL(clicked()),this,SLOT(convertFolderToCSV()));
    connect( m_ui->pushButtonLoadFromCSV,SIGNAL(clicked()),this,SLOT(loadFromCSV()));

    connect( m_ui->pushButtonLoadTwoFolders,SIGNAL(clicked()),this,SLOT(loadFolderTwoHumans()));

    m_id_source_rm = 0;
    m_id_target_rm = 0;
    m_saved_file_id = 0;
}

void HriGestureWidget::loadRecordedMotion()
{
    cout << "Load recorded motion" << endl;

    if( global_motionRecorders.empty() ) {
        cout << "recorder not initialized" << endl;
        return;
    }

    global_motionRecorders[0]->loadMotionFromMultipleFiles( "/home/jmainpri/workspace/move3d/libmove3d/statFiles/recorded_motion/motion_saved_" , 37 );
    //m_recorder->loadFromXml("/home/jmainpri/workspace/move3d/libmove3d/statFiles/recorded_motion/motion_saved_00000.xml");
}

void HriGestureWidget::showRecordedMotion()
{
    cout << "HriGestureWidget::showRecordedMotion" << endl;
    emit(selectedPlanner(QString("ShowRecordedMotion")));
}

void HriGestureWidget::getIthConfigurationInMotion()
{
    int ith = m_ui->spinBoxShowConfigurationRecordedMotion->value();

    if ( !global_motionRecorders.empty() )
    {
        global_motionRecorders[0]->setRobotToConfiguration( ith );
        global_motionRecorders[0]->setShowMotion( ith );
    }
    else {
        cout << "global_motionRecorder is not initilized" << endl;
    }
    m_mainWindow->drawAllWinActive();
}

void HriGestureWidget::setSourceRM()
{
    m_id_source_rm = m_ui->spinBoxShowConfigurationRecordedMotion->value();
    cout << "Set source id : " << m_id_source_rm << endl;
}

void HriGestureWidget::setTargetRM()
{
    m_id_target_rm = m_ui->spinBoxShowConfigurationRecordedMotion->value();
    cout << "Set target id : " << m_id_source_rm << endl;
}

void HriGestureWidget::extractAndSaveRM()
{
    if ( !global_motionRecorders.empty() )
    {
        int begin = m_id_source_rm;
        int end   = m_id_target_rm;
        cout << "Extract motion between " << begin << " and " << end << endl;
        motion_t part = global_motionRecorders[0]->extractSubpart( begin, end );

        std::string home(getenv("HOME_MOVE3D"));
        std::string filepath;
        std::ostringstream filename;

        filename << "/statFiles/recorded_cut/motion_";
        filename << std::setw( 5 ) << std::setfill( '0' ) << m_saved_file_id++ << ".xml";
        filepath = home+filename.str();

        global_motionRecorders[0]->saveToXml( filepath, part );
        cout << "Save file to : " << filepath << endl;
    }
    else {
        cout << "global_motionRecorder is not initilized" << endl;
    }
}

//std::string foldername = "/home/jmainpri/workspace/move3d/data/Library_Untouched/Sorted/8";
std::string foldername = "/home/jmainpri/workspace/move3d/libmove3d/statFiles/collaboration/recorded_motion_01_09_13";

void HriGestureWidget::loadFolder()
{
    if ( global_motionRecorders.empty() )
    {
        cout << "global_motionRecorder is not initilized" << endl;

        Robot* human = global_Project->getActiveScene()->getRobotByName( "HERAKLES_HUMAN1" );
        if( human == NULL )
        {
            cout << "human or robot not defined" << endl;
            return;
        }

        global_motionRecorders.push_back( new HRICS::RecordMotion( human ) );
    }

    //    global_motionRecorder->loadXMLFolder();
    global_motionRecorders[0]->loadCSVFolder( foldername );
}

void HriGestureWidget::convertFolderToCSV()
{
    if ( global_motionRecorders.empty() )
    {
        cout << "global_motionRecorder is not initilized" << endl;
        return;
    }

    global_motionRecorders[0]->saveStoredToCSV( foldername + "/compound.csv", false );
}

void HriGestureWidget::loadFromCSV()
{
    if ( global_motionRecorders.empty() )
    {
        cout << "global_motionRecorder is not initilized" << endl;

        Robot* human = global_Project->getActiveScene()->getRobotByName( "HERAKLES_HUMAN1" );
        if( human == NULL )
        {
            cout << "human or robot not defined" << endl;
            return;
        }

        global_motionRecorders.push_back( new HRICS::RecordMotion( human ) );
    }


    std::string foldername = "/home/jmainpri/Dropbox/workspace/gesture-recognition/gmm/gmm-gmr-gesture-recognition/";
    global_motionRecorders[0]->loadRegressedFromCSV( foldername );
}

void HriGestureWidget::loadFolderTwoHumans()
{
    if ( !global_motionRecorders.empty() )
    {
        for(int i=0;i<int(global_motionRecorders.size());i++)
        {
            delete global_motionRecorders[i];
        }

        global_motionRecorders.clear();
    }

    Scene* sce = global_Project->getActiveScene();
    Robot* human1 = sce->getRobotByName( "HERAKLES_HUMAN1" );
    Robot* human2 = sce->getRobotByName( "HERAKLES_HUMAN2" );
    if( human1 == NULL || human2 == NULL )
    {
        cout << "No humans HERAKLES in the the scene" << endl;
        return;
    }

    global_motionRecorders.push_back( new HRICS::RecordMotion( human1 ) );
    global_motionRecorders.push_back( new HRICS::RecordMotion( human2 ) );

    global_motionRecorders[0]->loadCSVFolder( foldername + "/human0" );
    global_motionRecorders[1]->loadCSVFolder( foldername + "/human1");
}

//-------------------------------------------------------------------
// Workspace Occupancy
//-------------------------------------------------------------------
void HriGestureWidget::initWorkspaceOccupancy()
{
    connect(m_ui->pushButtonSetMotionsAndComputeOccupancy,SIGNAL(clicked()),this,SLOT(computeWorkspaceOccupancy()));
    connect(m_ui->pushButtonClassifyMotion,SIGNAL(clicked()),this,SLOT(classifyMotion()));
    connect(m_ui->spinBoxClassToDraw, SIGNAL(valueChanged(int)),this,SLOT(setClassToDraw(int)));

    new connectCheckBoxToEnv( m_ui->checkBoxPlayNext,  GestEnv->getObject(GestParam::play_next) );
    new connectCheckBoxToEnv( m_ui->checkBoxRepeat,  GestEnv->getObject(GestParam::play_repeat) );

    new connectCheckBoxToEnv( m_ui->checkBoxDrawHumanSampledPoints,  GestEnv->getObject(GestParam::draw_human_sampled_points) );
    new connectCheckBoxToEnv( m_ui->checkBoxDrawRobotSampledPoints,  GestEnv->getObject(GestParam::draw_robot_sampled_points) );
    new connectCheckBoxToEnv( m_ui->checkBoxDrawVoxelOccupancy,      GestEnv->getObject(GestParam::draw_ws_occupancy) );
    new connectCheckBoxToEnv( m_ui->checkBoxDrawOneClassOnly,        GestEnv->getObject(GestParam::draw_single_class) );
    new connectCheckBoxToEnv( m_ui->checkBoxDrawHumanTrajectory,     GestEnv->getObject(GestParam::draw_recorded_motion) );
    new connectCheckBoxToEnv( m_ui->checkBoxDrawCurrentOccupancy,    GestEnv->getObject(GestParam::draw_current_occupancy) );

    new connectCheckBoxToEnv( m_ui->checkBoxMultipleStomps,          GestEnv->getObject(GestParam::with_multiple_stomps) );
    new connectCheckBoxToEnv( m_ui->checkBoxParallelize,             GestEnv->getObject(GestParam::parallelize_stomp) );

    connect( m_ui->pushButtonStartGestureSimulation, SIGNAL(clicked()), this, SLOT(startGestureSimulation()));
}

void HriGestureWidget::computeWorkspaceOccupancy()
{
    if( global_workspaceOccupancy == NULL || global_motionRecorders.empty() )
    {
        cout << "global_workspaceOccupancy or global_motionRecorder are not initilized" << endl;
        return;
    }

    emit(selectedPlanner(QString("WorkspaceOccupancy")));
}

void HriGestureWidget::setClassToDraw(int id)
{
    if( global_workspaceOccupancy == NULL || global_motionRecorders.empty() )
    {
        cout << "global_workspaceOccupancy or global_motionRecorder are not initilized" << endl;
        return;
    }

    global_workspaceOccupancy->setClassToDraw(id);
    m_mainWindow->drawAllWinActive();
}

void HriGestureWidget::classifyMotion()
{
    emit(selectedPlanner(QString("ClassifyMotions")));
}


void HriGestureWidget::startGestureSimulation()
{
    emit(selectedPlanner(QString("PredictionSimulation")));
}

//-------------------------------------------------------------------
// Gesture Recognition
//-------------------------------------------------------------------
void HriGestureWidget::initGestureRecognition()
{
    //    global_classifyMotion = new HRICS::ClassifyMotion();

    //    if( global_classifyMotion->load_model() )
    //        cout << "load GMMs successfully!!!!" << endl;
    //    else
    //        cout << "ERROR loading GMMs!!!!" << endl;
}

//-------------------------------------------------------------------
// IOC
//-------------------------------------------------------------------
void HriGestureWidget::initHriIOC()
{
    new connectCheckBoxToEnv( m_ui->checkBoxInitHriIOC,          GestEnv->getObject(GestParam::init_module_ioc) );
    new connectCheckBoxToEnv( m_ui->checkBoxInitSphereCost,      HriEnv->getObject(HricsParam::init_spheres_cost) );
    new connectCheckBoxToEnv( m_ui->checkBoxInitHumanTrajCost,   HriEnv->getObject(HricsParam::init_human_trajectory_cost) );
    new connectCheckBoxToEnv( m_ui->checkBoxIocExitAfterRun,     HriEnv->getObject(HricsParam::ioc_exit_after_run) );

    new connectCheckBoxToEnv( m_ui->checkBoxGestureDebug,        GestEnv->getObject(GestParam::print_debug) );
    new connectCheckBoxToEnv( m_ui->checkBoxSingleIteration,     HriEnv->getObject(HricsParam::ioc_single_iteration) );

    new connectCheckBoxToEnv( m_ui->checkBoxDrawDemonstrations,  HriEnv->getObject(HricsParam::ioc_draw_demonstrations) );
    new connectCheckBoxToEnv( m_ui->checkBoxDrawSamples,         HriEnv->getObject(HricsParam::ioc_draw_samples) );
    new connectCheckBoxToEnv( m_ui->checkBoxLoadSampleFromFile,  HriEnv->getObject(HricsParam::ioc_load_samples_from_file) );
    new connectCheckBoxToEnv( m_ui->checkBoxSampleAroundDemo,    HriEnv->getObject(HricsParam::ioc_sample_around_demo) );
    new connectCheckBoxToEnv( m_ui->checkBoxUseSimulationDemos,  HriEnv->getObject(HricsParam::ioc_use_simulation_demos) );
    new connectCheckBoxToEnv( m_ui->checkBoxUserSetPelvisBounds, HriEnv->getObject(HricsParam::ioc_user_set_pelvis_bounds) );

    new connectCheckBoxToEnv( m_ui->checkBoxNoReplanning, HriEnv->getObject(HricsParam::ioc_no_replanning) );
    new connectCheckBoxToEnv( m_ui->checkBoxUseBaseline, HriEnv->getObject(HricsParam::ioc_use_baseline) );

    connect( m_ui->pushButtonRunIoc, SIGNAL(clicked()), this, SLOT(runIoc()) );

    new connectComboBoxToEnv( m_ui->comboBoxIoc, HriEnv->getObject(HricsParam::ioc_phase) );
    new connectComboBoxToEnv( m_ui->comboBoxPlannerType, HriEnv->getObject(HricsParam::ioc_planner_type) );
    new connectComboBoxToEnv( m_ui->comboBoxIK, HriEnv->getObject(HricsParam::ioc_ik) );

    new SpinBoxConnector( this, m_ui->doubleSpinBoxSpherePower, HriEnv->getObject(HricsParam::ioc_spheres_power) );
    new SpinBoxConnector( this, m_ui->spinBoxSampleIteration, HriEnv->getObject(HricsParam::ioc_sample_iteration) );
    new SpinBoxConnector( this, m_ui->spinBoxNbOfWayPoints, HriEnv->getObject(HricsParam::ioc_nb_of_way_points) );

    new SpinBoxConnector( this, m_ui->doubleSpinBoxStandardDev, HriEnv->getObject(HricsParam::ioc_sample_std_dev) );
    new SpinBoxConnector( this, m_ui->doubleSpinBoxStandardDevIK, HriEnv->getObject(HricsParam::ioc_sample_std_dev_ik) );

    new SpinBoxConnector( this, m_ui->spinBoxSphereLayerToDraw, HriEnv->getObject(HricsParam::ioc_spheres_to_draw) );
    connect( m_ui->spinBoxSphereLayerToDraw, SIGNAL(valueChanged(int)), m_mainWindow, SLOT(drawAllWinActive()));

    connect( m_ui->pushButtonDetours,SIGNAL(clicked()),this,SLOT(runDetours()));
    //    m_ui->comboBoxIoc->setCurrentIndex(  );

    //    global_classifyMotion = new HRICS::ClassifyMotion();

    //    if( global_classifyMotion->load_model() )
    //        cout << "load GMMs successfully!!!!" << endl;
    //    else
    //        cout << "ERROR loading GMMs!!!!" << endl;
}

void HriGestureWidget::setCurrentPhase(int phase)
{
    HriEnv->setInt( HricsParam::ioc_phase, phase );
    cout << "set current phase to : " << phase << endl;
}

void HriGestureWidget::runIoc()
{
    cout << "HriGestureWidget::runIoc" << endl;
    // emit(selectedPlanner(QString("HumanIOC")));
    emit(selectedPlanner(QString("RunIOC")));
}

void HriGestureWidget::runDetours()
{
    emit(selectedPlanner(QString("Detours")));
}

//-------------------------------------------------------------------
// Legible Costs
//-------------------------------------------------------------------
void HriGestureWidget::initLegibleCost()
{
    new connectCheckBoxToEnv( m_ui->checkBoxUseLegibleCost,  PlanEnv->getObject(PlanParam::useLegibleCost) );
}

//-------------------------------------------------------------------
// Classification
//-------------------------------------------------------------------
void HriGestureWidget::initClassification()
{
    connect( m_ui->pushButtonClassifyBad,SIGNAL(clicked()),this,SLOT(classifyBad()));
    connect( m_ui->pushButtonClassifyGood,SIGNAL(clicked()),this,SLOT(classifyGood()));
    connect( m_ui->pushButtonClassifyBothBad,SIGNAL(clicked()),this,SLOT(classifyBothBad()));
    connect( m_ui->pushButtonClassifyInit,SIGNAL(clicked()),this,SLOT(classifyInit()));
}

void HriGestureWidget::classifyInit()
{
    emit(selectedPlanner(QString("ExtractAllTrajectories")));
}

void HriGestureWidget::classifyGood()
{

    cout << "classify good" << endl;
}

void HriGestureWidget::classifyBad()
{
    cout << "classify bad" << endl;
}

void HriGestureWidget::classifyBothBad()
{
    cout << "classify both bad" << endl;
}
