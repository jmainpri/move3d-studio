/*
 *  qtCost.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "qtHrics.hpp"
#include "ui_qtHrics.h"

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
#include "qtMainInterface/mainwindowGenerated.hpp"

#include "qtMotionPlanner.hpp"

#include "HRI_costspace/HRICS_costspace.hpp"
#include "HRI_costspace/Gestures/HRICS_RecordMotion.hpp"
#include "HRI_costspace/Gestures/HRICS_WorkspaceOccupancy.hpp"

#include "planner/planEnvironment.hpp"
#include "utils/ConfGenerator.h"

using namespace std;
MOVE3D_USING_SHARED_PTR_NAMESPACE

extern Eigen::Vector3d global_DrawnSphere;

HricsWidget::HricsWidget(QWidget *parent) :
    QWidget(parent),
    m_ui(new Ui::HricsWidget)
{
    m_ui->setupUi(this);

    //this->initHRI();
    //this->initHumanLike();
}

HricsWidget::~HricsWidget()
{
    delete m_ui;
    //#ifdef USE_QWT
    //    delete this->plot;
    //#endif
}

//---------------------------------------------------------------------
// HRI
//---------------------------------------------------------------------
void HricsWidget::initHRI()
{	  
    connect(this, SIGNAL(selectedPlanner(QString)), global_plannerHandler, SLOT(startPlanner(QString)));

    m_mainWindow->connectCheckBoxToEnv(m_ui->enableHri_2,										Env::enableHri);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawGrid,							Env::drawGrid);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxEntireGrid,						Env::drawEntireGrid);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawDistance,					Env::drawDistance);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawRandPoints,				Env::drawPoints);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawGaze,							Env::drawGaze);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawVectorField,				Env::drawVectorField);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawOnlyOneLine,				Env::drawOnlyOneLine);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawBox,               Env::drawBox);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawReachable,         PlanParam::drawReachableGrid);

    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawHumanColorFromConf,PlanParam::hriSetColorFromConfig);

    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxHRICS_MOPL,						Env::HRIPlannerWS);
    //	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxBBDist,								Env::useBoxDist);
    //	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxBallDist,							Env::useBallDist);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxHriPlannerTRRT,				Env::HRIPlannerTRRT);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxHriPathDistance,				Env::HRIPathDistance);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxCameraBehindHuman,			Env::HRIcameraBehindHuman);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxHRINoRobot,						Env::HRINoRobot);
    m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxAutoLoadGrid,					Env::HRIAutoLoadGrid);

    connect(m_ui->checkBoxDrawGrid,SIGNAL(clicked()),m_mainWindow,SLOT(drawAllWinActive()));
    connect(m_ui->checkBoxEntireGrid,SIGNAL(clicked()),m_mainWindow,SLOT(drawAllWinActive()));

    connect(m_ui->pushButtonInitHRICS,SIGNAL(clicked()),this,SLOT(make3DHriGrid()));

    connect(m_ui->pushButtonInitGrids,SIGNAL(clicked()),this,SLOT(initGrids()));
    connect(m_ui->pushButtonDeleteGrids,SIGNAL(clicked()),this,SLOT(deleteGrids()));
    connect(m_ui->pushButtonComputeAllCellCost,SIGNAL(clicked()),this,SLOT(computeAllCellCost()));

    // Load and Save agent grids
    connect(m_ui->pushButtonLoad,SIGNAL(clicked()),this,SLOT(loadGrid()));
    connect(m_ui->pushButtonSave,SIGNAL(clicked()),this,SLOT(saveGrid()));

    // Simple Otp test
    connect(m_ui->pushButtonComputeOtpConfig,SIGNAL(clicked()),this,SLOT(computeOtpConfig()));

    // A*
    connect(m_ui->pushButtonAStarInGrid,SIGNAL(clicked()),this,SLOT(computeAStarGrid()));

    // -------------------------------
    // K Sliders
    // -------------------------------
    m_k_distance = new QtShiva::SpinBoxSliderConnector(this,
                                                       m_ui->doubleSpinBoxDistance,
                                                       m_ui->horizontalSliderDistance,
                                                       Env::Kdistance);


    m_k_visbility = new QtShiva::SpinBoxSliderConnector(this,
                                                        m_ui->doubleSpinBoxVisibility,
                                                        m_ui->horizontalSliderVisibility,
                                                        Env::Kvisibility );


    m_k_naturality = new QtShiva::SpinBoxSliderConnector(this,
                                                         m_ui->doubleSpinBoxNatural,
                                                         m_ui->horizontalSliderNatural,
                                                         Env::Knatural );

    m_k_reachability = new QtShiva::SpinBoxSliderConnector(this,
                                                           m_ui->doubleSpinBoxReachable,
                                                           m_ui->horizontalSliderReachable,
                                                           Env::Kreachable );

    //	connect(m_k_distance,SIGNAL(valueChanged(double)),this,SLOT(KDistance(double)));
    //	connect(m_k_visbility,SIGNAL(valueChanged(double)),this,SLOT(KVisibility(double)));

    // -------------------------------
    // Cells
    // -------------------------------
    new QtShiva::SpinBoxSliderConnector(this,
                                        m_ui->doubleSpinBoxCellSize,
                                        m_ui->horizontalSliderCellSize,
                                        Env::CellSize);

    new QtShiva::SpinBoxSliderConnector(this,
                                        m_ui->doubleSpinBoxColor1,
                                        m_ui->horizontalSliderColor1,
                                        Env::colorThreshold1);

    new QtShiva::SpinBoxSliderConnector(this,
                                        m_ui->doubleSpinBoxColor2,
                                        m_ui->horizontalSliderColor2,
                                        Env::colorThreshold2);

    // -------------------------------
    // Wich Test
    // -------------------------------
    connect(m_ui->whichTestBox, SIGNAL(currentIndexChanged(int)),ENV.getObject(Env::hriCostType), SLOT(set(int)), Qt::DirectConnection);
    connect(ENV.getObject(Env::hriCostType), SIGNAL(valueChanged(int)),this, SLOT(setWhichTestSlot(int)), Qt::DirectConnection);
    m_ui->whichTestBox->setCurrentIndex(ENV.getInt(Env::hriCostType));

    // -------------------------------
    // All widgets
    // -------------------------------
    initVectorField();
    initDrawOneLineInGrid();
    initRecordedMotion();
    initWorkspaceOccupancy();
    //  initGrids();
    //  initObjectTransferPoint();

    //  setGroupBoxDisabled(true);
}

void HricsWidget::setGroupBoxDisabled(bool disable)
{
    m_ui->groupBoxCostFunctions->setDisabled( disable );
    m_ui->groupBoxDraw->setDisabled( disable );
    m_ui->groupBoxGrids->setDisabled( disable );
    m_ui->groupBoxHRICSPlanner->setDisabled( disable );
    m_ui->groupBoxMisc->setDisabled( disable );
}

void HricsWidget::drawAllWinActive()
{
    m_mainWindow->drawAllWinActive();
}

/*!
 * Zone size (protective bubble) changed
 */
void HricsWidget::zoneSizeChanged()
{
    if(ENV.getBool(Env::HRIPlannerWS))
    {
        HRICS_activeDist = HRICS_MotionPL->getDistance();
        HRICS_activeDist->parseHumans();
    }
    else if(ENV.getBool(Env::HRIPlannerCS))
    {
        HRICS_activeDist = HRICS_MotionPL->getDistance();
        HRICS_activeDist->parseHumans();
    }

    m_mainWindow->drawAllWinActive();
    cout << "Zone Size Changed" << endl;
}

void HricsWidget::initVectorField()
{
    connect(m_ui->pushButtonVectorField,SIGNAL(clicked()),this,SLOT(computeVectorField()));
}

void HricsWidget::computeVectorField()
{
    dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getGrid()->computeVectorField();
    API_activeGrid = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getGrid();
    ENV.setBool(Env::drawGrid,true);
    ENV.setBool(Env::drawVectorField,true);
    m_mainWindow->drawAllWinActive();
}

//-------------------------------------------------------------
// Object Transfer Point
//-------------------------------------------------------------
void HricsWidget::initObjectTransferPoint()
{
    connect(m_k_distance,		SIGNAL(valueChanged(double)),this,SLOT(computeObjectTransferPoint()));
    connect(m_k_visbility,		SIGNAL(valueChanged(double)),this,SLOT(computeObjectTransferPoint()));
    connect(m_k_reachability,	SIGNAL(valueChanged(double)),this,SLOT(computeObjectTransferPoint()));
}

// Compute the object transfer point
void HricsWidget::computeObjectTransferPoint()
{
    qDebug() << "HricsWidget::computeObjectTransferPoint";

    ENV.setBool(Env::HRIComputeOTP,true);

    if ( ENV.getBool(Env::HRIComputeOTP) )
    {
        Eigen::Vector3d WSPoint;
        qDebug() << "HricsWidget::computeObjectTransferPoint Eigen";
        if( dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->computeBestTransferPoint(WSPoint) )
        {
            HRICS::Natural* reachSpace = HRICS_MotionPL->getReachability();

            reachSpace->computeIsReachableAndMove( WSPoint, reachSpace->getGrid()->isReachableWithLA(WSPoint) );

            Robot* Object = global_Project->getActiveScene()->getRobotByNameContaining("OBJECT");

            confPtr_t q_curr = Object->getCurrentPos();
            (*q_curr)[6] = WSPoint[0];
            (*q_curr)[7] = WSPoint[1];
            (*q_curr)[8] = WSPoint[2];
            // qDebug() << "| z : "<< WSPoint[0] <<"| z : "<<  WSPoint[1] <<  "| z : "<< WSPoint[2] ;

            Object->setAndUpdate(*q_curr);
            cout << "Set and update : " << Object->getName() << endl << WSPoint << endl;
            m_mainWindow->drawAllWinActive();
        }
    }
}

//-------------------------------------------------------------
// Select One Slice
//-------------------------------------------------------------
void HricsWidget::initDrawOneLineInGrid()
{
    connect(m_ui->radioButtonXline, SIGNAL(toggled(bool)), this, SLOT(set_X_line(bool)));
    connect(m_ui->radioButtonYline, SIGNAL(toggled(bool)), this, SLOT(set_Y_line(bool)));
    connect(m_ui->radioButtonZline, SIGNAL(toggled(bool)), this, SLOT(set_Z_line(bool)));

    QtShiva::SpinBoxSliderConnector* connector =  new QtShiva::SpinBoxSliderConnector(
                this, m_ui->doubleSpinBoxWhichGridLine, m_ui->horizontalSliderWhichGridLine , Env::hriShownGridLine );

    connect(connector,SIGNAL(valueChanged(double)),m_mainWindow,SLOT(drawAllWinActive()));

}

void HricsWidget::set_X_line(bool enable)
{	
    if (enable && API_activeGrid)
    {
        unsigned int maxLines = dynamic_cast<API::ThreeDGrid*>(API_activeGrid)->getXNumberOfCells();
        m_ui->doubleSpinBoxWhichGridLine->setMaximum(maxLines);
    }

    ENV.setInt(Env::lineToShow,0);
}

void HricsWidget::set_Y_line(bool enable)
{
    if (enable && API_activeGrid)
    {
        unsigned int maxLines = dynamic_cast<API::ThreeDGrid*>(API_activeGrid)->getYNumberOfCells();
        m_ui->doubleSpinBoxWhichGridLine->setMaximum(maxLines);
    }

    ENV.setInt(Env::lineToShow,1);
}


void HricsWidget::set_Z_line(bool enable)
{
    if (enable && API_activeGrid)
    {
        unsigned int maxLines = dynamic_cast<API::ThreeDGrid*>(API_activeGrid)->getZNumberOfCells();
        m_ui->doubleSpinBoxWhichGridLine->setMaximum(maxLines);
    }

    ENV.setInt(Env::lineToShow,2);
}


//-------------------------------------------------------------
// IK Grids
//-------------------------------------------------------------
void HricsWidget::setWhichTestSlot(int test)
{
    cout << "Change test to :" << test << endl;

    if(ENV.getBool(Env::HRIPlannerTS))
    {
#ifdef HRI_PLANNER
        //hriSpace->changeTest(test);
        cout << "HricsWidget::setWhichTestSlot::WARNING" << endl;
#else
        cout << "HRI Planner not compiled nor linked" << endl;
#endif
    }
}

//-------------------------------------------------------------
// Plan Workspace HRI
//-------------------------------------------------------------
void HricsWidget::make3DHriGrid()
{
    HRICS_init();
    m_mainWindow->Ui()->tabCost->initCostFunctions();
}

void HricsWidget::delete3DHriGrid()
{
    ENV.setBool(Env::drawGrid,false);
    ENV.setBool(Env::HRIPlannerWS,false);

    delete HRICS_MotionPL;
    HRICS_MotionPL = NULL;

    //m_ui->HRICSPlanner->setDisabled(true);

    m_mainWindow->drawAllWinActive();

    m_ui->pushButtonInitHRICS->setDisabled(false);
    m_ui->pushButtonDeleteGrid->setDisabled(true);
}

//-------------------------------------------------------------
// Human Cost Space
//-------------------------------------------------------------
void HricsWidget::initGrids()
{
    // Get the HRICS workspace
    HRICS::Workspace* ws = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL);

    if( ws == NULL ) {
        cout << "HRICS::Workspace is not initialized" << endl;
    }

    // If a costspace allready exists delete it!
    if(HRICS_humanCostMaps!=NULL)
        delete HRICS_humanCostMaps;

    // Build a costspace from the Robot and Humans in the workspace
    // This costspace will hold the agents grids
    HRICS_humanCostMaps = new HRICS::HumanCostSpace(ws->getRobot(),ws->getHumans(), HRICS_activeNatu, ENV.getDouble(Env::CellSize));
}

void HricsWidget::deleteGrids()
{
    delete HRICS_humanCostMaps;
    HRICS_humanCostMaps = NULL;
}

void HricsWidget::computeAllCellCost()
{
    emit(selectedPlanner(QString("ComputeAgentGridCost")));
}

void HricsWidget::loadGrid()
{
    emit(selectedPlanner(QString("LoadAgentGrid")));
}

void HricsWidget::saveGrid()
{
    emit(selectedPlanner(QString("SaveAgentGrid")));
}

//-------------------------------------------------------------
// OTHERS
//-------------------------------------------------------------
void HricsWidget::resetRandomPoints()
{
    ENV.setBool(Env::drawPoints,false);
    if(PointsToDraw != NULL)
    {
        delete PointsToDraw;
    }
}

void HricsWidget::AStarIn3DGrid()
{
    dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->computeAStarIn3DGrid();
    ENV.setBool(Env::drawTraj,true);
    m_mainWindow->drawAllWinActive();
}

void HricsWidget::HRICSRRT()
{
    //    std::string str = "runHRICSRRT";
    //    write(qt_fl_pipe[1],str.c_str(),str.length()+1);

    dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->initHriRRT();
    ENV.setBool(Env::drawTraj,true);
    m_mainWindow->drawAllWinActive();
}

void HricsWidget::computeOtpConfig()
{
    Scene* sce = global_Project->getActiveScene();

    Robot* rob = sce->getRobotByNameContaining("ROBOT");
    Robot* hum = sce->getRobotByNameContaining("HUMAN");

    ConfGenerator generator( rob, hum );

    Eigen::Vector3d point = hum->getJoint("rPalm")->getVectorPos();
    point[2] += 0.10;

    configPt q; // = q_rob->getConfigStruct();

    timeval tim;
    gettimeofday(&tim, NULL);
    double t_init = tim.tv_sec+(tim.tv_usec/1000000.0);

    if( generator.computeRobotIkForGrabing( q, point ) )
    {
        gettimeofday(&tim, NULL);
        double dt = tim.tv_sec+(tim.tv_usec/1000000.0) - t_init;
        cout << "Ik computed in : " << dt << " sec" << endl;

        confPtr_t q_rob(new Configuration(rob,q));
        rob->setAndUpdate(*q_rob);
        m_mainWindow->drawAllWinActive();
    }
    else
    {
        cout << "Could not find a robot OTP configuration" << endl;
    }
}

void HricsWidget::on_pushButton_initColPos_clicked()
{
    HRICS_activeNatu->setRobotToConfortPosture();
    HRICS_activeNatu->setRobotColorFromConfiguration(false);
    m_mainWindow->drawAllWinActive();
}

void HricsWidget::on_checkBoxPosOr_toggled(bool checked)
{
    if (checked)
    {
        m_mh = new MovingHuman();
        m_mh->setMainWindow(m_mainWindow);
        m_mh->show();
    }
    else
    {
        delete m_mh;
    }
}

void HricsWidget::computeAStarGrid()
{
    emit(selectedPlanner(QString("ComputeAStar")));
}

//--------------------------------------------------------------------
// Recorded motion
//--------------------------------------------------------------------
// How to :
// Run spark-genom and record the complete motion in n-xml-files
// Then load the motion and extract and save each motion in n-xml-files
// Load the folder and convert to CSV to make one single motion file
void HricsWidget::initRecordedMotion()
{
    global_motionRecorder = new RecordMotion( "HERAKLES_HUMAN1" );

    connect(m_ui->pushButtonLoadRecordedMotion,SIGNAL(clicked()),this,SLOT(loadRecordedMotion()));
    connect(m_ui->pushButtonShowRecordedMotion,SIGNAL(clicked()),this,SLOT(showRecordedMotion()));

    connect(m_ui->spinBoxShowConfigurationRecordedMotion, SIGNAL(valueChanged(int)), this, SLOT(getIthConfigurationInMotion()), Qt::QueuedConnection);
    connect(m_ui->pushButtonSetBeginRM,SIGNAL(clicked()),this,SLOT(setSourceRM()));
    connect(m_ui->pushButtonSetEndRM,SIGNAL(clicked()),this,SLOT(setTargetRM()));
    connect(m_ui->pushButtonExtarctAndSaveRM,SIGNAL(clicked()),this,SLOT(extractAndSaveRM()));
    connect(m_ui->pushButtonLoadFolder,SIGNAL(clicked()),this,SLOT(loadFolder()));
    connect(m_ui->pushButtonSaveToCSV,SIGNAL(clicked()),this,SLOT(convertFolderToCSV()));
    connect(m_ui->pushButtonLoadFromCSV,SIGNAL(clicked()),this,SLOT(loadFromCSV()));


    m_id_source_rm = 0;
    m_id_target_rm = 0;
    m_saved_file_id = 0;
}

void HricsWidget::loadRecordedMotion()
{
    cout << "Load recorded motion" << endl;

    if(!global_motionRecorder) {
        cout << "recorder not initialized" << endl;
        return;
    }

    global_motionRecorder->loadMotionFromMultipleFiles( "/home/jmainpri/workspace/move3d/libmove3d/statFiles/recorded_motion/motion_saved_" , 37 );
    //m_recorder->loadFromXml("/home/jmainpri/workspace/move3d/libmove3d/statFiles/recorded_motion/motion_saved_00000.xml");
}

void HricsWidget::showRecordedMotion()
{
    emit(selectedPlanner(QString("ShowRecordedMotion")));
}

void HricsWidget::getIthConfigurationInMotion()
{
    int ith = m_ui->spinBoxShowConfigurationRecordedMotion->value();

    if ( global_motionRecorder ) {
        global_motionRecorder->setConfiguration( ith );
    }
    else {
        cout << "global_motionRecorder is not initilized" << endl;
    }
    m_mainWindow->drawAllWinActive();
}

void HricsWidget::setSourceRM()
{
    m_id_source_rm = m_ui->spinBoxShowConfigurationRecordedMotion->value();
    cout << "Set source id : " << m_id_source_rm << endl;
}

void HricsWidget::setTargetRM()
{
    m_id_target_rm = m_ui->spinBoxShowConfigurationRecordedMotion->value();
    cout << "Set target id : " << m_id_source_rm << endl;
}

void HricsWidget::extractAndSaveRM()
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

/**
void HricsWidget::loadFolder()
{
    if ( global_motionRecorder == NULL )
    {
        cout << "global_motionRecorder is not initilized" << endl;
        return;
    }

    string foldername = "/home/jmainpri/workspace/move3d/libmove3d/statFiles/recorded_motion/";
    string command = "ls " + foldername;
    cout << "Load Folder : " << foldername << endl;
    //system("echo $PATH");
    FILE *fp;
    char path[PATH_MAX];

    fp = popen(command.c_str(), "r");
    if (fp == NULL) {
        cout << "ERROR in system call" << endl;
        return;
    }

    global_motionRecorder->reset();

    int i=0;
    while ( fgets( path, PATH_MAX, fp) != NULL ) {
        string filename(path);
        filename.erase(std::remove(filename.begin(), filename.end(), '\n'), filename.end());
        if ( filename.find(".xml") == string::npos )
            continue;
        cout << "file path is : " << filename << endl; i++;

        vector<motion_t> motions;
        motions.push_back( global_motionRecorder->loadFromXml( foldername + filename ) );
        global_motionRecorder->storeMotion( motions.back() );
    }

    pclose(fp);

    cout << "End Load " << i <<  " files in the folder" << endl;
}
**/

/**
void HricsWidget::convertFolderToCSV()
{
    if ( global_motionRecorder == NULL )
    {
        cout << "global_motionRecorder is not initilized" << endl;
        return;
    }

    string foldername = "/home/jmainpri/workspace/move3d/libmove3d/statFiles/recorded_cut/record_2/";
    string command = "ls " + foldername;
    cout << "Load Folder : " << foldername << endl;
    //system("echo $PATH");
    FILE *fp;
    char path[PATH_MAX];

    fp = popen(command.c_str(), "r");
    if (fp == NULL) {
        cout << "ERROR in system call" << endl;
        return;
    }

    global_motionRecorder->reset();

    int i=0;
    while ( fgets( path, PATH_MAX, fp) != NULL ) {

        string filename(path);
        filename.erase( remove(filename.begin(), filename.end(), '\n'), filename.end());
        if ( filename.find(".xml") == string::npos )
            continue;

        cout << "file path is : " << filename << endl; i++;
        motion_t motion = global_motionRecorder->loadFromXml( foldername + filename );

        global_motionRecorder->addToCurrentMotion( motion );
        global_motionRecorder->storeMotion( motion );

//        filename.erase( filename.end()-4, filename.end());
//        filename = filename + ".csv" ;
//        cout << "save file to " << filename << endl;
//        global_motionRecorder->saveToCSV( foldername + "csv_files/" + filename, motion );
    }

    pclose(fp);

    cout << "End Load " << i <<  " files in the folder" << endl;

    global_motionRecorder->saveStoredToCSV( foldername + "csv_files/compound.csv" );
}
**/

void HricsWidget::loadFolder()
{
    if ( global_motionRecorder == NULL )
    {
        cout << "global_motionRecorder is not initilized" << endl;
        return;
    }

    string foldername = "/home/jmainpri/workspace/move3d/libmove3d/statFiles/recorded_motion/";
    cout << "Load Folder : " << foldername << endl;

    string command = "ls " + foldername;
    FILE* fp = popen( command.c_str(), "r");
    if (fp == NULL) {
        cout << "ERROR in system call" << endl;
        return;
    }
    char path[PATH_MAX]; int max_number_of_motions=0;
    while ( fgets( path, PATH_MAX, fp) != NULL ) max_number_of_motions++;
    pclose(fp);

    if( max_number_of_motions == 0) cout << "no file in folder" << endl;

    // Set the motion number you want to load
    int first_motion = 76;
    max_number_of_motions = 25;
    int number_of_motions_loaded = 0;
    const int max_number_of_files = 500;

    for( int i=first_motion; i<(first_motion+max_number_of_motions); i++ )
    {
        for( int j=0; j<max_number_of_files; j++ )
        {
            ostringstream filename;
            filename << foldername << "motion_saved_";
            filename << std::setw( 5 ) << std::setfill( '0' ) << i << "_";
            filename << std::setw( 5 ) << std::setfill( '0' ) << j << ".xml";

            ifstream file_exists( filename.str().c_str() );
            if( file_exists )
            {
                cout << "Load File : " << filename.str() << endl;
                motion_t partial_motion = global_motionRecorder->loadFromXml( filename.str() );
                global_motionRecorder->storeMotion( partial_motion, j == 0 );

                if( j == 0 ) {
                    number_of_motions_loaded++;
                }
            }
            else {
                break;
            }
        }
    }
    cout << "Number of motion loaded : " << number_of_motions_loaded << endl;
}

void HricsWidget::convertFolderToCSV()
{
    if ( global_motionRecorder == NULL )
    {
        cout << "global_motionRecorder is not initilized" << endl;
        return;
    }

    string foldername = "/home/jmainpri/workspace/move3d/libmove3d/statFiles/recorded_motion/";
    global_motionRecorder->saveStoredToCSV( foldername + "csv_files/compound.csv" );
}

void HricsWidget::loadFromCSV()
{
    if ( global_motionRecorder == NULL )
    {
        cout << "global_motionRecorder is not initilized" << endl;
        return;
    }

    global_motionRecorder->loadRegressedFromCSV();
}

//-------------------------------------------------------------------
// Workspace Occupancy
//-------------------------------------------------------------------
void HricsWidget::initWorkspaceOccupancy()
{
    vector<double> size = global_Project->getActiveScene()->getBounds();
    global_workspaceGrid = new HRICS::WorkspaceOccupancyGrid( 0.10, size );

    connect(m_ui->pushButtonSetMotionsAndComputeOccupancy,SIGNAL(clicked()),this,SLOT(computeWorkspaceOccupancy()));
}

void HricsWidget::computeWorkspaceOccupancy()
{
    if( global_workspaceGrid == NULL || global_motionRecorder == NULL )
    {
        cout << "global_workspaceGrid or global_motionRecorder are not initilized" << endl;
        return;
    }

    cout << "Loading regressed motion and computing the occupancy" << endl;
    global_motionRecorder->loadRegressedFromCSV();
    global_workspaceGrid->setRegressedMotions( global_motionRecorder->getStoredMotions() );
    global_workspaceGrid->computeOccpancy();
}
