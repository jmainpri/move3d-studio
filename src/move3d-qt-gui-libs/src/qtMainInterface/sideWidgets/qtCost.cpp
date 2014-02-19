/*
 *  qtCost.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 LAAS-CNRS. All rights reserved.
 *
 */

#include "qtCost.hpp"
#include "ui_qtCost.h"

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <tr1/memory>

#include "qtBase/SpinBoxSliderConnector_p.hpp"

#include "planner/planEnvironment.hpp"
#include "planner/cost_space.hpp"
#include "planner/TrajectoryOptim/Stomp/stompOptimizer.hpp"

#include "collision_space/CollisionSpace.hpp"

#if defined(HRI_COSTSPACE)
#include "hri_costspace/HRICS_costspace.hpp"
#endif
#include "API/Search/Dijkstra/dijkstra.hpp"

#ifdef USE_QWT
#include "qtPlot/basicPlot.hpp"
#include "qtPlot/multiPlot.hpp"
#include "qtPlot/tempWin.hpp"
#endif

#include "qtMainInterface/mainwindow.hpp"
#include "qtMainInterface/mainwindowGenerated.hpp"

#ifdef HRI_PLANNER
#include "qtHrics.hpp"
#include "ui_qtHrics.h"
#include "qtNatural.hpp"
#include "qtHriGesture.hpp"
#endif

#include "qtMotionPlanner.hpp"

#include "Util-pkg.h"
#include "Planner-pkg.h"

using namespace std;
MOVE3D_USING_SHARED_PTR_NAMESPACE
using namespace QtShiva;

// import most common Eigen types 
//USING_PART_OF_NAMESPACE_EIGEN
using namespace Eigen;

CostWidget::CostWidget(QWidget *parent) :
    QWidget(parent),
    m_ui(new Ui::CostWidget)
{
    m_ui->setupUi(this);

#if defined(LIGHT_PLANNER) && defined(MULTILOCALPATH)
    // Initialize the replanning tab
    m_tabReplan = new ReplanningWidget(m_ui->Replanning);
    m_tabReplan->setObjectName(QString::fromUtf8("tabReplan"));
    m_ui->replanningLayout->addWidget(m_tabReplan);
#endif
#ifdef HRI_PLANNER
    // Initialize the hri, otp tab and natural tabs
    m_tabHri = new HricsWidget(m_ui->CostHri);
    m_tabHri->setObjectName(QString::fromUtf8("tabHri"));
    m_ui->hriLayout->addWidget(m_tabHri);

    m_tabOtp = new OtpWidget(m_ui->ObjectTransferPoint);
    m_tabOtp->setObjectName(QString::fromUtf8("tabOTP"));
    m_ui->otpLayout->addWidget(m_tabOtp);

#ifdef MIGHTABILITY_MAPS
    m_tabMightabiliby = new qtMightability(m_ui->Mightabilitytab);
    m_ui->MightLayout->addWidget(m_tabMightabiliby);
#endif

    m_tabNatural = new NaturalWidget(m_ui->Natural);
    m_tabNatural->setObjectName(QString::fromUtf8("tabNatural"));
    m_ui->naturalLayout->addWidget(m_tabNatural);

    m_tabGesture = new HriGestureWidget(m_ui->Gesture);
    m_tabGesture->setObjectName(QString::fromUtf8("tabGesture"));
    m_ui->gestureLayout->addWidget(m_tabGesture);
#endif

    m_tabRRTStar = new RRTStarWidget(m_ui->RRTStar);
    m_tabRRTStar->setObjectName(QString::fromUtf8("tabRRTStar"));
    m_ui->starLayout->addWidget(m_tabRRTStar);
}

CostWidget::~CostWidget()
{
    delete m_ui;
    //#ifdef USE_QWT
    //    delete this->plot;
    //#endif
}

void CostWidget::setMainWindow(MainWindow *ptrMW)
{
    m_mainWindow = ptrMW;

#if defined(LIGHT_PLANNER) && defined(MULTILOCALPATH)
    m_tabReplan->setMainWindow( m_mainWindow );
#endif
#ifdef HRI_PLANNER
    m_tabHri->setMainWindow( m_mainWindow );
#ifdef MIGHTABILITY_MAPS
    m_tabMightabiliby->setMainWindow( m_mainWindow );
#endif
    m_tabNatural->setMainWindow( m_mainWindow );
    m_tabGesture->setMainWindow( m_mainWindow );
#endif
    m_tabRRTStar->setMainWindow( m_mainWindow );
}

#ifdef HRI_COSTSPACE
HricsWidget* CostWidget::getHriWidget()
{ 
    return m_tabHri;
}

OtpWidget* CostWidget::getOtpWidget()
{
    return m_tabOtp;
}

HriGestureWidget* CostWidget::getGestureWidget()
{
    return m_tabGesture;
}
#endif

#if defined(LIGHT_PLANNER) && defined(MULTILOCALPATH)
ReplanningWidget* CostWidget::getReplanningWidget()
{
    return m_tabReplan;
}
#endif

DistFieldWidget* CostWidget::getDistFieldWidget()
{
    return m_ui->tabDistField;
}

void CostWidget::envUseTRRTValueChanged( bool state )
{
    if(state && !m_ui->isCostSpaceCopy->isChecked())
    {
        m_ui->isCostSpaceCopy->setChecked(true);
    }
}

void CostWidget::envIsCostSpaceValueChanged( bool state )
{
    if(!state && m_ui->checkBoxUseTRRT->isChecked())
    {
        m_ui->checkBoxUseTRRT->setChecked(false);
    }
}
//---------------------------------------------------------------------
// COST
//---------------------------------------------------------------------
void CostWidget::initCost()
{
    if(ENV.getBool(Env::isCostSpace))
    {
        GlobalCostSpace::initialize();
        this->initCostFunctions();
    }

    new connectCheckBoxToEnv(m_ui->isCostSpaceCopy,			    ENV.getObject(Env::isCostSpace));
    connect(m_ui->isCostSpaceCopy, SIGNAL(toggled( bool )), SLOT(envIsCostSpaceValueChanged( bool ) ) );
    new connectCheckBoxToEnv(m_ui->checkBoxUseTRRT,             ENV.getObject(Env::useTRRT));
    connect(m_ui->checkBoxUseTRRT, SIGNAL(toggled( bool )), SLOT(envUseTRRTValueChanged( bool ) ) );
    new connectCheckBoxToEnv(m_ui->checkBoxCostBefore,		    ENV.getObject(Env::costBeforeColl));
    new connectCheckBoxToEnv(m_ui->checkBoxCostExpandToGoal,	ENV.getObject(Env::costExpandToGoal));
    new connectCheckBoxToEnv(m_ui->checkBoxCostWithGradient,	ENV.getObject(Env::tRrtComputeGradient));
    new connectCheckBoxToEnv(m_ui->checkBoxPrintAndComputeCostAfterPlannif,	PlanEnv->getObject(PlanParam::trajComputeCostAfterPlannif));

    connect(m_ui->pushButtonInitCostSpace,SIGNAL(clicked()),this,SLOT(initCostSpace()));

    new SpinBoxSliderConnector( this, m_ui->doubleSpinBoxInitTemp, m_ui->horizontalSliderInitTemp , ENV.getObject(Env::initialTemperature) );
    new SpinBoxSliderConnector( this, m_ui->doubleSpinBoxTempRate, m_ui->horizontalSliderTempRate , ENV.getObject(Env::temperatureRate) );
    new SpinBoxSliderConnector( this, m_ui->doubleSpinBoxLengthWeight, m_ui->horizontalSliderLengthWeight , ENV.getObject(Env::KlengthWeight) );
    new SpinBoxSliderConnector( this, m_ui->doubleSpinBoxResolution, m_ui->horizontalSliderResolution , PlanEnv->getObject(PlanParam::costResolution) );
    new SpinBoxSliderConnector( this, m_ui->doubleSpinBoxMinConnectGap, m_ui->horizontalSliderMinConnectGap , ENV.getObject(Env::minimalFinalExpansionGap) );
    new SpinBoxSliderConnector( this, m_ui->doubleSpinBoxMaxCost, m_ui->horizontalSliderMaxCost , ENV.getObject(Env::costMax) );

#ifdef USE_QWT
    connect(m_ui->pushButtonShowTrajCost,SIGNAL(clicked()),this,SLOT(showTrajCost()));
    connect(m_ui->pushButtonShowCostProfile,SIGNAL(clicked()),this,SLOT(showCostProfile()));
    connect(m_ui->pushButtonShowHRITrajCost,SIGNAL(clicked()),this,SLOT(showHRITrajCost()));
    connect(m_ui->pushButtonShowSTOMPCost,SIGNAL(clicked()),this,SLOT(showSTOMPTrajCost()));
    connect(m_ui->pushButtonShowTemp,SIGNAL(clicked()),this,SLOT(showTemperature()));
    new connectCheckBoxToEnv(m_ui->checkBoxRescale, ENV.getObject(Env::initPlot));

    m_plot = new BasicPlotWindow();
#endif

    qRegisterMetaType< std::vector<double> > ("std::vector<double>");
    connect(ENV.getObject(Env::costAlongTraj), SIGNAL(valueChanged(std::vector<double>)), this, SLOT(setPlotedVector(std::vector<double>)));
    //    connect(m_ui->pushButtonShowTrajCost,SIGNAL(clicked()),this->plot,SLOT(show()));
    connect(m_ui->pushButtonGridInGraph,SIGNAL(clicked()),this,SLOT(putGridInGraph()));
    //    connect(m_ui->pushButtonAStar,SIGNAL(clicked()),this,SLOT(computeAStar()));

    // costCriterium
    connect(m_ui->comboBoxTrajCostEstimation, SIGNAL(currentIndexChanged(int)), this, SLOT(setCostCriterium(int)));
    m_ui->comboBoxTrajCostEstimation->setCurrentIndex( 0 );

    connect(m_ui->comboBoxDistanceEstimation, SIGNAL(currentIndexChanged(int)), this, SLOT(setDistanceCriterium(int)));
    m_ui->comboBoxDistanceEstimation->setCurrentIndex( 0 );

    //connect(m_ui->pushButton2DAstar,SIGNAL(clicked()),this,SLOT(computeGridAndExtract()));
    connect(m_ui->pushButton2DDijkstra,SIGNAL(clicked()),this,SLOT(graphSearchTest()));

    connect(m_ui->pushButtonRecomputeGraph,SIGNAL(clicked()),this,SLOT(newGraphAndReComputeCost()));
    connect(m_ui->pushButtonExtractBestPath,SIGNAL(clicked()),this,SLOT(extractBestPath()));
    connect(m_ui->pushButtonStonesGraph,SIGNAL(clicked()),this,SLOT(stonesGraph()));
}


extern void* GroundCostObj;

void CostWidget::initCostSpace()
{  
    if (ENV.getBool(Env::isCostSpace) && global_costSpace == NULL )
    {
        GlobalCostSpace::initialize();
    }

    //  GlobalCostSpace::initialize();

    if (ENV.getBool(Env::enableHri))
    {
#ifdef HRI_PLANNER
        //    HRICS_init();

        this->resetCostFunctions();
        //this->setCostFunction("costHRI");
        this->setCostFunction("costHumanGrids");

        m_tabHri->setGroupBoxDisabled(false);

        //  m_tabHri->Ui()->HRICSPlanner->setDisabled(false);
        //  m_tabHri->Ui()->pushButtonMakeGrid->setDisabled(true);
        //  m_tabHri->Ui()->pushButtonDeleteGrid->setDisabled(false);
        //  m_tabHri->Ui()->HRICSNatural->setDisabled(false);
        //  m_tabHri->Ui()->pushButtonNewNaturalCostSpace->setDisabled(true);

        m_tabOtp->initSliders();

        cout << "HRI_COSTSPACE OK -----------------------" << endl;
#endif
    }
    else
    {
        this->resetCostFunctions();
    }
}

void CostWidget::resetCostFunctions()
{
    vector<string> AllCost = global_costSpace->getAllCost();
    m_ui->comboBoxCostFunctions->clear();
    m_ui->comboBoxCostFunctions->blockSignals(true);

    for (unsigned int i=0; i<AllCost.size(); i++)
    {
        m_ui->comboBoxCostFunctions->addItem( QString(AllCost[i].c_str()) );
    }

    string function = global_costSpace->getSelectedCostName();
    vector<string>::iterator it = std::find( AllCost.begin() , AllCost.end() , function );
    m_ui->comboBoxCostFunctions->setCurrentIndex( it - AllCost.begin() );
    m_ui->comboBoxCostFunctions->blockSignals(false);

    connect(m_ui->comboBoxCostFunctions, SIGNAL(currentIndexChanged(int)),this, SLOT(setCostFunction(int)));
}

// This function is called by the mainwindow
// Constructor after initialzing all widgets
void CostWidget::initCostFunctions()
{
    vector<string> AllCost = global_costSpace->getAllCost();
    m_ui->comboBoxCostFunctions->clear();
    m_ui->comboBoxCostFunctions->blockSignals(true);

    for (unsigned int i=0; i<AllCost.size(); i++)
    {
        QString name(AllCost[i].c_str());

        m_ui->comboBoxCostFunctions->addItem(name);
    }

    m_ui->comboBoxCostFunctions->blockSignals(false);

    connect(m_ui->comboBoxCostFunctions, SIGNAL(currentIndexChanged(int)),this, SLOT(setCostFunction(int)));
    m_ui->comboBoxCostFunctions->setCurrentIndex( 0 );

    string function;
    if( GroundCostObj != NULL )
    {
        function = "costMap2D";
    }
    else {
        function = "costDistToObst";
    }
    vector<string>::iterator it = std::find( AllCost.begin() , AllCost.end() , function );
    m_ui->comboBoxCostFunctions->setCurrentIndex( it - AllCost.begin() );
}

void CostWidget::setCostFunction(std::string function)
{
    vector<string> costFunc  = global_costSpace->getAllCost();
    vector<string>::iterator it = std::find( costFunc.begin() , costFunc.end() , function);
    if (it == costFunc.end())
    {
        cout << "Wrong cost function name" << endl;
        return;
    }
    m_ui->comboBoxCostFunctions->setCurrentIndex( it - costFunc.begin() );
}

void CostWidget::setCostFunction(int costFunctionId)
{
    vector<string> AllCost = global_costSpace->getAllCost();
    if (costFunctionId< int(AllCost.size()) && costFunctionId>= 0)
    {
        PlanEnv->setString( PlanParam::active_cost_function, AllCost[costFunctionId] );
        global_costSpace->setCost(AllCost[costFunctionId]);
        cout << "Cost Function is now :  " << PlanEnv->getString( PlanParam::active_cost_function ) << endl;
    }
}

void CostWidget::setCostCriterium(int criterion)
{
    if (global_costSpace == NULL)
    {
        cout << "Warning : No cost space" << endl;
    }
    else
    {
        switch (criterion)
        {
        case 0: global_costSpace->setDeltaStepMethod(cs_integral); break;
        case 1: global_costSpace->setDeltaStepMethod(cs_mechanical_work); break;
        case 2: global_costSpace->setDeltaStepMethod(cs_average); break;
        }
    }
}

void CostWidget::setDistanceCriterium(int distance)
{
    if (global_costSpace == NULL)
    {
        cout << "Warning : No cost space" << endl;
    }
    else
    {
        switch (distance)
        {
        case 0: global_costSpace->setDistanceMethod(cs_classic); break;
        case 1: global_costSpace->setDistanceMethod(cs_pr2_manip); break;
        }
    }
}

void CostWidget::extractBestPath()
{
    //	p3d_ExtractBestTraj(XYZ_GRAPH);
    Graph* myGraph = new Graph(XYZ_GRAPH);
    Robot* myRobot = myGraph->getRobot();

    myGraph->extractBestTraj(myRobot->getInitPos(),myRobot->getGoalPos());

    m_mainWindow->drawAllWinActive();
}

void CostWidget::newGraphAndReComputeCost()
{
    if (!XYZ_GRAPH)
    {
        cout << "No XYZ_GRAPH!!!!!!" << endl;
    }
    else
    {
        Graph* myGraph = new Graph(new Robot(XYZ_GRAPH->rob),XYZ_GRAPH);
        myGraph->recomputeCost();
        cout << "All graph cost recomputed XYZ_GRAPH is uptodate" << endl;
    }

}

void CostWidget::stonesGraph()
{
    if (!XYZ_GRAPH)
    {
        p3d_del_graph(XYZ_GRAPH);
    }
    double du(0.0), ds(0.0);
    const char* file = std::string("/Users/jmainpri/workspace/BioMove3DDemos/PathDeform/simple/StonesACR.graph").c_str();
    p3d_readGraph(file, DEFAULTGRAPH);
    ENV.setBool(Env::isCostSpace,true);
    ChronoOn();
    this->newGraphAndReComputeCost();
    ChronoTimes(&du,&ds);
    cout << "Time to compute cost on graph : " << ds << endl;
    ENV.setBool(Env::drawGraph,true);
    m_mainWindow->drawAllWinActive();
    ChronoOff();
}


void CostWidget::graphSearchTest()
{
    cout << "Extracting Grid" << endl;

    p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);

#ifdef P3D_PLANNER
    if(XYZ_GRAPH)
    {
        p3d_del_graph(XYZ_GRAPH);
    }

    cout << "Creating Dense Roadmap" << endl;
    p3d_CreateDenseRoadmap(robotPt);
#endif

    Graph* ptrGraph = new Graph(XYZ_GRAPH);

    shared_ptr<Configuration> Init = ptrGraph->getRobot()->getInitPos();
    shared_ptr<Configuration> Goal = ptrGraph->getRobot()->getGoalPos();

    cout << "Dijkstra graph search on graph" << endl;
    Dijkstra graphSearch(ptrGraph);

    cout << "graphSearch.extractTrajectory" << endl;
    API::Trajectory* traj = graphSearch.extractTrajectory(Init,Goal);

    cout << "-------------------------------" << endl;
    cout << "Trajectory Cost = "<< traj->cost() << endl;
    cout << "   nl = "<< traj->getNbOfPaths() << endl;
    cout << "   length = "<< traj->getRangeMax() << endl;

    ENV.setBool(Env::drawGraph,false);
    ENV.setBool(Env::drawTraj,true);

    traj->replaceP3dTraj();

    m_mainWindow->drawAllWinActive();

    delete traj;
}


void CostWidget::showTrajCost()
{
#if defined(USE_QWT)
    cout << "showTrajCost" << endl;

    BasicPlot* myPlot = new BasicPlot( m_plot );
    myPlot->setGeometry( m_plot->getPlot()->geometry() );
    int nbSample = myPlot->getPlotSize();

    Scene* sc = global_Project->getActiveScene();
    Robot* robot = sc->getActiveRobot();

    // If simbot exist use it for traj cost
    Robot* simbot = sc->getRobotByNameContaining("_ROBOT");
    if( simbot != NULL )
    {
        robot = simbot;
    }

    cout << "Show traj cost of robot : " << robot->getName() << endl;

    API::Trajectory traj = robot->getCurrentTraj();

    double step = traj.getRangeMax() / double(nbSample);

    vector<double> cost;

    // Print all costs
    traj.costDeltaAlongTraj();

    TrajectoryStatistics traj_statistics;
    traj.costStatistics( traj_statistics );

    cout << "--- stats on traj ---" << endl;
    cout << " length = " << traj_statistics.length << endl;
    cout << " max = " << traj_statistics.max << endl;
    cout << " average = " << traj_statistics.average << endl;
    cout << " integral = " << traj_statistics.integral << endl;
    cout << " mecha_work = " << traj_statistics.mecha_work << endl;
    cout << "---------------------" << endl;

    int nb_points = PlanEnv->getInt( PlanParam::nb_pointsOnTraj );
    double cost_n_points = traj.costNPoints( nb_points );
    cout << "Cost for ( nb points : " << nb_points << " ) : " << cost_n_points << endl;

    cout << "Is trajectory valid : " << traj.isValid() << endl;

    if (global_costSpace)
    {
        for( double param=0; param<traj.getRangeMax(); param = param + step)
        {
            cost.push_back( traj.configAtParam(param)->cost() );
        }
        myPlot->setTitle("Cost Space along trajectory");
    }
    else if (global_optimizer)
    {
        global_optimizer->getTrajectoryCost(cost,step);
        myPlot->setTitle("STOMP/CHOMP traj cost profile");
    }

    myPlot->setData(cost);
    delete m_plot->getPlot();
    m_plot->setPlot(myPlot);
    m_plot->show();
#endif
}

void CostWidget::showCostProfile()
{
#if defined(USE_QWT)
    cout << "showCostProfile" << endl;
    p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
    p3d_traj* CurrentTrajPt = robotPt->tcur;

    BasicPlot* myPlot = new BasicPlot( m_plot );
    myPlot->setGeometry( m_plot->getPlot()->geometry() );
    int nbSample = myPlot->getPlotSize();

    API::Trajectory traj(new Robot(robotPt),CurrentTrajPt);

    //	cout << "nbSample : " << nbSample << endl;
    const double step = traj.getRangeMax() / (double) nbSample;

    std::vector< std::pair<double,double> > profile = traj.getCostProfile();

    unsigned int i = 0;
    unsigned int j = 0;

    std::vector<double> cost;

    for( double param=0.0; param< traj.getRangeMax(); param += step)
    {
        while( profile[i].first < param )
        {
            if( ( profile.size() - 1 ) > i  )
            {
                i++;
            }
            else
            {
                break;
            }
            //cout << "profile[i].first = " << profile[i].first << endl;
        }

        //cout << "param = " << param << endl;
        //		cout << "cost.first = " << profile[i].first << endl;
        //		cout << "cost.second = " << profile[i].second << endl;
        cost.push_back( profile[i].second );

        cout << "cost[" << j++ << "] = " << cost.back() << endl;
    }

    myPlot->setData(cost);
    delete m_plot->getPlot();
    m_plot->setPlot(myPlot);
    m_plot->show();
#endif
}

int count_hri_traj = 0;
void CostWidget::showHRITrajCost()
{
    if ( HRICS_humanCostMaps == NULL )
    {
        cerr << "HRICS_humanCostMaps is not initialized!!!" << endl;
        return;
    }

#if defined(USE_QWT)
    cout << "--------------------------------" << endl;
    Robot* robot = HRICS_humanCostMaps->getRobot();
    API::Trajectory traj( robot->getCurrentTraj() );

    MultiPlot* myPlot = new MultiPlot( m_plot );
    myPlot->setGeometry( m_plot->getPlot()->geometry() );

    int nbSample = myPlot->getPlotSize();

    double step = traj.getRangeMax() / double(nbSample-1);

    cout << "Traj cost = " << traj.costDeltaAlongTraj() << endl;
    // cout << "Traj param max = " << traj.getRangeMax() << endl;
    // cout << "Traj step = " << step << endl;

    std::ostringstream oss;
    oss << getenv("HOME_MOVE3D") << "/statFiles/CostAlongTraj_" << std::setw(3) << std::setfill('0') << count_hri_traj++ << ".csv";

    const char *res = oss.str().c_str();

    std::ofstream s;
    s.open(res);

    cout << "Opening save file : " << res << endl;

    s << "Distance"  << ";";
    s << "Visibility"  << ";";
    s << "Reachability"  << ";";
    s << "Combine"  << ";";
    s << "Grouping"  << ";";
    s << endl;

    // cost and vector variables
    double dCost(0.0), vCost(0.0), rCost(0.0), cCost(0.0), gCost(0.0), tCost(0.0);
    vector<double> costDistance, costVisibility, costReachable, costCombine, costGrouping, costTotal;


    double p=0.0;

    for( int i=0; i<(nbSample-1); i++ )
    {
        confPtr_t q = traj.configAtParam(p);

        p += step;

        vector<double> costs;

#ifdef HRI_COSTSPACE
        HRICS_humanCostMaps->getCompleteCost( *q, costs );

        dCost = costs[0];
        vCost = costs[1];
        rCost = costs[2];

        cCost = costs[3];
        gCost = costs[4];
        tCost = costs[5];

        // Put 3 costs in vector
        costDistance.push_back( dCost );
        costVisibility.push_back( vCost );
        costReachable.push_back( rCost );

        costCombine.push_back( cCost );
        costGrouping.push_back( gCost );
        costTotal.push_back( tCost );

        // Save to file
        s << dCost << ";";
        s << vCost << ";";
        s << rCost << ";";
        s << cCost << ";";
        s << gCost << ";";
        s << tCost << ";";
        s << endl;
#endif
    }

    cout << "Closing save file" << endl;
    s.close();

    vector< vector<double> > curves;
    curves.push_back( costDistance );
    curves.push_back( costVisibility );
    //curves.push_back( costReachable );
    curves.push_back( costCombine );
    curves.push_back( costGrouping );
    curves.push_back( costTotal );

    vector< string > plotNames;
    plotNames.push_back( "Distance" );
    plotNames.push_back( "Visibility" );
    //plotNames.push_back( "Reachability" );
    plotNames.push_back( "Combine" );
    plotNames.push_back( "Grouping" );
    plotNames.push_back( "Total" );

    myPlot->setData( plotNames , curves );

    delete m_plot->getPlot();
    m_plot->setPlot(myPlot);
    m_plot->show();
#endif
}


void CostWidget::showSTOMPTrajCost()
{
#ifdef USE_QWT
    cout << "showSTOMPTrajCost" << endl;

    std::vector<double> smoothness_cost;
    std::vector<double> collision_cost;
    std::vector<double> general_cost;

    if ( global_optimizer ) {
        API::Trajectory traj( global_Project->getActiveScene()->getActiveRobot() );
        global_optimizer->getCostProfiles( smoothness_cost, collision_cost, general_cost );
        global_optimizer->setGroupTrajectoryToApiTraj( traj );
        traj.replaceP3dTraj();
    }
    else {
        return;
    }

    MultiPlot* myPlot = new MultiPlot( m_plot );
    myPlot->setGeometry( m_plot->getPlot()->geometry() );

    int nb_sample = myPlot->getPlotSize();

    std::vector<double> smoothness_cost_curve;
    std::vector<double> collision_cost_curve;
    std::vector<double> general_cost_curve;

    double inc = double(smoothness_cost.size())/double(nb_sample);
    double k =0;

    for (int j=0; j<nb_sample; j++)
    {
        smoothness_cost_curve.push_back(smoothness_cost[floor(k)]);
        collision_cost_curve.push_back(collision_cost[floor(k)]);
        general_cost_curve.push_back(general_cost[floor(k)]);

        k += inc;
    }

    for( int i=0;i<int(collision_cost.size());i++)
    {
        cout << "collision_cost[" << i << "] = " << collision_cost[i] << endl;
    }

    std::vector< std::vector<double> > curves;
    curves.push_back( smoothness_cost_curve );
    curves.push_back( collision_cost_curve );
    curves.push_back( general_cost_curve );

    std::vector< std::string > plotNames;
    plotNames.push_back( "Smoothness" );
    plotNames.push_back( "Obstacle" );
    //plotNames.push_back( "Reachability" );
    plotNames.push_back( "General" );

    myPlot->setData( plotNames , curves );

    delete m_plot->getPlot();
    m_plot->setPlot(myPlot);
    m_plot->show();
#endif
}

void CostWidget::setPlotedVector(vector<double> v)
{
    cout << "PLOTTING ------------------------------------------" << endl;
#ifdef USE_QWT
    BasicPlot* myPlot = dynamic_cast<BasicPlot*>(m_plot->getPlot());
    vector<double> cost = ENV.getVector(Env::costAlongTraj);
    cost.resize(myPlot->getPlotSize());
    myPlot->setData(cost);
    m_plot->show();
#endif
}

void CostWidget::showTemperature()
{
#ifdef USE_QWT
    TempWin* window = new TempWin();
    window->show();
#endif
}

void CostWidget::computeAStar()
{
    if(!ENV.getBool(Env::isCostSpace))
    {
        return;
    }
#ifdef P3D_PLANNER
    if(!(XYZ_GRAPH->start_nodePt))
    {

        XYZ_GRAPH->search_start = XYZ_GRAPH->nodes->N;
        XYZ_GRAPH->search_goal = XYZ_GRAPH->last_node->N;
        cout << "p3d_initSearch" << endl;
        p3d_initSearch(XYZ_GRAPH);

        cout << "Number Of Graph nodes = " << XYZ_GRAPH->nnode << endl;
#ifdef CXX_PLANNNER
        GraphState* InitialState = new GraphState(XYZ_GRAPH->nodes->N);

        //        N = new Node(ptrGraph,rob->getGoalPos());
        //        ptrGraph->insertNode(N);
        //        ptrGraph->linkNode(N);

        API::AStar search;
        vector<API::State*> path = search.solve(InitialState);

        if(path.size() == 0 )
        {
            return;
        }

        API::Trajectory* traj = new API::Trajectory(new Robot(XYZ_ROBOT));

        for (unsigned int i=0;i<path.size();i++)
        {
            configPt conf = dynamic_cast<GraphState*>(path[i])->getGraphNode()->q;
            shared_ptr<Configuration> q(new Configuration(new Robot(XYZ_ROBOT),conf));
            traj->push_back(q);
        }

        traj->replaceP3dTraj();
#endif
        m_mainWindow->drawAllWinActive();

        cout << "solution : End Search" << endl;
    }
    else
    {
        cout << "No start node" << endl;
    }
#endif
}

void CostWidget::putGridInGraph()
{
    cout << "Computing Grid" << endl;

#ifdef CXX_PLANNNER
    Vector3i     gridSize;

    gridSize[0] = 10;
    gridSize[1] = 10;
    gridSize[2] = 10;

    //    vector<double>  envSize(6);
    //    envSize[0] = XYZ_ENV->box.x1; envSize[1] = XYZ_ENV->box.x2;
    //    envSize[2] = XYZ_ENV->box.y1; envSize[3] = XYZ_ENV->box.y2;
    //    envSize[4] = XYZ_ENV->box.z1; envSize[5] = XYZ_ENV->box.z2;

    //    Grid myGrid(gridSize,envSize);
    //    myGrid.createAllCells();
    //
    //    for(int i=0;i<myGrid.getNumberOfCells();i++)
    //    {
    //        vector<double> center = myGrid.getCell(i)->getCenter();
    //        cout << i << " =  ("<< center[0] << "," << center[1] << "," << center[2] << ")" << endl;
    //    }
    //---------------

    GridToGraph theGrid(gridSize);
    theGrid.putGridInGraph();
#endif

    m_mainWindow->drawAllWinActive();
}


