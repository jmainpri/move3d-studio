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
#include <tr1/memory>
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
#include "hri_costspace/Gestures/HRICS_RecordMotion.hpp"
#include "hri_costspace/Gestures/HRICS_WorkspaceOccupancy.hpp"
#include "hri_costspace/Gestures/HRICS_ClassifyMotion.hpp"
#include "hri_costspace/Gestures/HRICS_HumanPredictionCostSpace.hpp"
#include "hri_costspace/Gestures/HRICS_GestParameters.hpp"
#include "hri_costspace/HRICS_parameters.hpp"

#include "planner/planEnvironment.hpp"
#include "utils/ConfGenerator.h"

using namespace std;
using namespace QtShiva;
MOVE3D_USING_SHARED_PTR_NAMESPACE

extern Eigen::Vector3d global_DrawnSphere;

HriGestureWidget::HriGestureWidget(QWidget *parent) :
    QWidget(parent),
    m_ui(new Ui::HriGestureWidget)
{
    m_ui->setupUi(this);

    //this->initHRI();
    //this->initHumanLike();

    initRecordedMotion();
    initWorkspaceOccupancy();
    initGestureRecognition();
    initHriIOC();
    initLegibleCost();
}

HriGestureWidget::~HriGestureWidget()
{
    delete m_ui;
    //#ifdef USE_QWT
    //    delete this->plot;
    //#endif
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

    m_id_source_rm = 0;
    m_id_target_rm = 0;
    m_saved_file_id = 0;
}

void HriGestureWidget::loadRecordedMotion()
{
    cout << "Load recorded motion" << endl;

    if(!global_motionRecorder) {
        cout << "recorder not initialized" << endl;
        return;
    }

    global_motionRecorder->loadMotionFromMultipleFiles( "/home/jmainpri/workspace/move3d/libmove3d/statFiles/recorded_motion/motion_saved_" , 37 );
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

    if ( global_motionRecorder ) {
        global_motionRecorder->setConfiguration( ith );
        global_motionRecorder->setShowMotion( ith );
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
    if ( global_motionRecorder )
    {
        int begin = m_id_source_rm;
        int end   = m_id_target_rm;
        cout << "Extract motion between " << begin << " and " << end << endl;
        motion_t part = global_motionRecorder->extractSubpart( begin, end );

        string home(getenv("HOME_MOVE3D")); string filepath; ostringstream filename;
        filename << "/statFiles/recorded_cut/motion_";
        filename << std::setw( 5 ) << std::setfill( '0' ) << m_saved_file_id++ << ".xml";
        filepath = home+filename.str();

        global_motionRecorder->saveToXml( filepath, part );
        cout << "Save file to : " << filepath << endl;
    }
    else {
        cout << "global_motionRecorder is not initilized" << endl;
    }
}

std::string foldername = "/home/jmainpri/workspace/move3d/data/Library_Untouched/Sorted/8";

void HriGestureWidget::loadFolder()
{
    if ( global_motionRecorder == NULL )
    {
        cout << "global_motionRecorder is not initilized" << endl;

        Robot* human = global_Project->getActiveScene()->getRobotByName( "HERAKLES_HUMAN1" );
        if( human == NULL )
        {
            cout << "human or robot not defined" << endl;
            return;
        }

        global_motionRecorder =  new HRICS::RecordMotion( human );
    }

//    global_motionRecorder->loadXMLFolder();
    global_motionRecorder->loadCSVFolder( foldername );
}

void HriGestureWidget::convertFolderToCSV()
{
    if ( global_motionRecorder == NULL )
    {
        cout << "global_motionRecorder is not initilized" << endl;
        return;
    }

    global_motionRecorder->saveStoredToCSV( foldername + "/compound.csv" );
}

void HriGestureWidget::loadFromCSV()
{
    if ( global_motionRecorder == NULL )
    {
        cout << "global_motionRecorder is not initilized" << endl;

        Robot* human = global_Project->getActiveScene()->getRobotByName( "HERAKLES_HUMAN1" );
        if( human == NULL )
        {
            cout << "human or robot not defined" << endl;
            return;
        }

        global_motionRecorder =  new HRICS::RecordMotion( human );
    }

    global_motionRecorder->loadRegressedFromCSV();
}

//-------------------------------------------------------------------
// Workspace Occupancy
//-------------------------------------------------------------------
void HriGestureWidget::initWorkspaceOccupancy()
{
    connect(m_ui->pushButtonSetMotionsAndComputeOccupancy,SIGNAL(clicked()),this,SLOT(computeWorkspaceOccupancy()));
    connect(m_ui->pushButtonClassifyMotion,SIGNAL(clicked()),this,SLOT(classifyMotion()));
    connect(m_ui->spinBoxClassToDraw, SIGNAL(valueChanged(int)),this,SLOT(setClassToDraw(int)));

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
    if( global_workspaceOccupancy == NULL || global_motionRecorder == NULL )
    {
        cout << "global_workspaceOccupancy or global_motionRecorder are not initilized" << endl;
        return;
    }

    emit(selectedPlanner(QString("WorkspaceOccupancy")));
}

void HriGestureWidget::setClassToDraw(int id)
{
    if( global_workspaceOccupancy == NULL || global_motionRecorder == NULL )
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
    new connectCheckBoxToEnv( m_ui->checkBoxInitHriIOC,  GestEnv->getObject(GestParam::init_module_ioc) );
    new connectCheckBoxToEnv( m_ui->checkBoxInitSphereCost,  HriEnv->getObject(HricsParam::init_spheres_cost) );

//    global_classifyMotion = new HRICS::ClassifyMotion();

//    if( global_classifyMotion->load_model() )
//        cout << "load GMMs successfully!!!!" << endl;
//    else
//        cout << "ERROR loading GMMs!!!!" << endl;
}

//-------------------------------------------------------------------
// Legible Costs
//-------------------------------------------------------------------
void HriGestureWidget::initLegibleCost()
{
    new connectCheckBoxToEnv( m_ui->checkBoxUseLegibleCost,  PlanEnv->getObject(PlanParam::useLegibleCost) );
}


