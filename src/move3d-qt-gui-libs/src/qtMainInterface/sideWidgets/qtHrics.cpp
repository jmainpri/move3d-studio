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
#include <boost/bind.hpp>
#include <sys/time.h>
#include <QMessageBox>

#include "qtBase/SpinBoxSliderConnector_p.hpp"

#include "qtLibrary.hpp"
#include "planner_handler.hpp"

#ifdef USE_QWT
#include "qtPlot/basicPlot.hpp"
#endif

#ifdef USE_QWT_5
#include "qtPlot/multiPlot.hpp"
#include "qtPlot/tempWin.hpp"
#endif

#include "qtMainInterface/mainwindow.hpp"
#include "qtMainInterface/mainwindowGenerated.hpp"
#include "qtMotionPlanner.hpp"

#include "hri_costspace/HRICS_costspace.hpp"
#include "planner/planEnvironment.hpp"
#include "utils/ConfGenerator.h"
#include "API/project.hpp"
#include "API/ConfigSpace/configuration.hpp"

using namespace std;
using namespace QtShiva;
using namespace Move3D;

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
    //#ifdef USE_QWT_5
    //    delete this->plot;
    //#endif
}

//---------------------------------------------------------------------
// HRI
//---------------------------------------------------------------------
void HricsWidget::initHRI()
{	  
    connect(this, SIGNAL(selectedPlanner(QString)), global_plannerHandler, SLOT(startPlanner(QString)));

    new connectCheckBoxToEnv(m_ui->enableHri_2,                 ENV.getObject(Env::enableHri));
    new connectCheckBoxToEnv(m_ui->checkBoxDrawGrid,			ENV.getObject(Env::drawGrid));
    new connectCheckBoxToEnv(m_ui->checkBoxEntireGrid,			ENV.getObject(Env::drawEntireGrid));
    new connectCheckBoxToEnv(m_ui->checkBoxDrawDistance,		ENV.getObject(Env::drawDistance));
    new connectCheckBoxToEnv(m_ui->checkBoxDrawRandPoints,		ENV.getObject(Env::drawPoints));
    new connectCheckBoxToEnv(m_ui->checkBoxDrawGaze,			ENV.getObject(Env::drawGaze));
    new connectCheckBoxToEnv(m_ui->checkBoxDrawVectorField,		ENV.getObject(Env::drawVectorField));
    new connectCheckBoxToEnv(m_ui->checkBoxDrawOnlyOneLine,		ENV.getObject(Env::drawOnlyOneLine));
    new connectCheckBoxToEnv(m_ui->checkBoxDrawBox,             ENV.getObject(Env::drawBox));
    new connectCheckBoxToEnv(m_ui->checkBoxDrawReachable,         PlanEnv->getObject(PlanParam::drawReachableGrid));
    new connectCheckBoxToEnv(m_ui->checkBoxDrawHumanColorFromConf, PlanEnv->getObject(PlanParam::drawNaturalColor));

    new connectCheckBoxToEnv(m_ui->checkBoxHRICS_MOPL,			ENV.getObject(Env::HRIPlannerWS));
    //	new connectCheckBoxToEnv(m_ui->checkBoxBBDist,			Env::useBoxDist);
    //	new connectCheckBoxToEnv(m_ui->checkBoxBallDist,		Env::useBallDist);
    new connectCheckBoxToEnv(m_ui->checkBoxHriPlannerTRRT,		ENV.getObject(Env::HRIPlannerTRRT));
    new connectCheckBoxToEnv(m_ui->checkBoxHriPathDistance,		ENV.getObject(Env::HRIPathDistance));
    new connectCheckBoxToEnv(m_ui->checkBoxCameraBehindHuman,	ENV.getObject(Env::HRIcameraBehindHuman));
    new connectCheckBoxToEnv(m_ui->checkBoxHRINoRobot,			ENV.getObject(Env::HRINoRobot));
    new connectCheckBoxToEnv(m_ui->checkBoxAutoLoadGrid,		ENV.getObject(Env::HRIAutoLoadGrid));

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
                                                       ENV.getObject(Env::Kdistance));


    m_k_visbility = new QtShiva::SpinBoxSliderConnector(this,
                                                        m_ui->doubleSpinBoxVisibility,
                                                        m_ui->horizontalSliderVisibility,
                                                        ENV.getObject(Env::Kvisibility));


    m_k_naturality = new QtShiva::SpinBoxSliderConnector(this,
                                                         m_ui->doubleSpinBoxNatural,
                                                         m_ui->horizontalSliderNatural,
                                                         ENV.getObject(Env::Knatural));

    m_k_reachability = new QtShiva::SpinBoxSliderConnector(this,
                                                           m_ui->doubleSpinBoxReachable,
                                                           m_ui->horizontalSliderReachable,
                                                           ENV.getObject(Env::Kreachable));

    //	connect(m_k_distance,SIGNAL(valueChanged(double)),this,SLOT(KDistance(double)));
    //	connect(m_k_visbility,SIGNAL(valueChanged(double)),this,SLOT(KVisibility(double)));

    // -------------------------------
    // Cells
    // -------------------------------
    new QtShiva::SpinBoxSliderConnector(this,
                                        m_ui->doubleSpinBoxCellSize,
                                        m_ui->horizontalSliderCellSize,
                                        ENV.getObject(Env::CellSize));

    new QtShiva::SpinBoxSliderConnector(this,
                                        m_ui->doubleSpinBoxColor1,
                                        m_ui->horizontalSliderColor1,
                                        ENV.getObject(Env::colorThreshold1));

    new QtShiva::SpinBoxSliderConnector(this,
                                        m_ui->doubleSpinBoxColor2,
                                        m_ui->horizontalSliderColor2,
                                        ENV.getObject(Env::colorThreshold2));

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
                this, m_ui->doubleSpinBoxWhichGridLine, m_ui->horizontalSliderWhichGridLine , ENV.getObject(Env::hriShownGridLine) );

    connect(connector,SIGNAL(valueChanged(double)),m_mainWindow,SLOT(drawAllWinActive()));

}

void HricsWidget::set_X_line(bool enable)
{	
    if (enable && API_activeGrid)
    {
        unsigned int maxLines = dynamic_cast<Move3D::ThreeDGrid*>(API_activeGrid)->getXNumberOfCells();
        m_ui->doubleSpinBoxWhichGridLine->setMaximum(maxLines);
    }

    ENV.setInt(Env::lineToShow,0);
}

void HricsWidget::set_Y_line(bool enable)
{
    if (enable && API_activeGrid)
    {
        unsigned int maxLines = dynamic_cast<Move3D::ThreeDGrid*>(API_activeGrid)->getYNumberOfCells();
        m_ui->doubleSpinBoxWhichGridLine->setMaximum(maxLines);
    }

    ENV.setInt(Env::lineToShow,1);
}


void HricsWidget::set_Z_line(bool enable)
{
    if (enable && API_activeGrid)
    {
        unsigned int maxLines = dynamic_cast<Move3D::ThreeDGrid*>(API_activeGrid)->getZNumberOfCells();
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


