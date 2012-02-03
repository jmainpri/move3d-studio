/*
 *  qtCost.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 __MyCompanyName__. All rights reserved.
 *
 */

#include "qtCost.hpp"
#include "ui_qtCost.h"

#include <iostream>
#include <algorithm>
#include <fstream>
#include <tr1/memory>
#include <boost/function.hpp>
#include <boost/bind.hpp>

#include "qtBase/SpinBoxSliderConnector_p.hpp"

#include "planner/cost_space.hpp"
#include "planner/Greedy/CollisionSpace.hpp"
#include "planner/TrajectoryOptim/Stomp/stompOptimizer.hpp"

#if defined(HRI_COSTSPACE)
#include "HRI_costspace/HRICS_costspace.hpp"
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
#endif

#include "qtMotionPlanner.hpp"

#include "Util-pkg.h"
#include "Planner-pkg.h"

using namespace std;
using namespace tr1;

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
  
  NaturalWidget* tabNatural = new NaturalWidget(m_ui->Natural);
  tabNatural->setObjectName(QString::fromUtf8("tabNatural"));
  m_ui->naturalLayout->addWidget(tabNatural);
#endif
}

CostWidget::~CostWidget()
{
	delete m_ui;
	//#ifdef USE_QWT
	//    delete this->plot;
	//#endif
}

#ifdef HRI_COSTSPACE
HricsWidget* CostWidget::getHriWidget()
{ 
	cout << "Warning : not compiling well HRICS interface" << endl;
	return m_tabHri; 
}

OtpWidget* CostWidget::getOtpWidget()
{
  cout << "Warning : not compiling well OTP interface" << endl;
  return m_tabOtp;
}
#endif

#if defined(LIGHT_PLANNER) && defined(MULTILOCALPATH)
ReplanningWidget* CostWidget::getReplanningWidget()
{
  cout << "Warning : not compiling well DistField interface" << endl;
  return m_tabReplan;
}
#endif

DistFieldWidget* CostWidget::getDistFieldWidget()
{
  cout << "Warning : not compiling well DistField interface" << endl;
  return m_ui->tabDistField;
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
  
	m_mainWindow->connectCheckBoxToEnv(m_ui->isCostSpaceCopy,			Env::isCostSpace);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxCostBefore,		Env::costBeforeColl);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxCostExpandToGoal,	Env::costExpandToGoal);
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxCostWithGradient,	Env::tRrtComputeGradient);
  
  connect(m_ui->pushButtonInitCostSpace,SIGNAL(clicked()),this,SLOT(initCostSpace()));
		
	new QtShiva::SpinBoxSliderConnector(
																			this, m_ui->doubleSpinBoxInitTemp, m_ui->horizontalSliderInitTemp , Env::initialTemperature );
	
	new QtShiva::SpinBoxSliderConnector(
																			this, m_ui->doubleSpinBoxNFailMax, m_ui->horizontalSliderNFailMax , Env::temperatureRate );
	
	new QtShiva::SpinBoxSliderConnector(
																			this, m_ui->doubleSpinBoxLengthWeight, m_ui->horizontalSliderLengthWeight , Env::KlengthWeight );
	
	new QtShiva::SpinBoxSliderConnector(
																			this, m_ui->doubleSpinBoxMinConnectGap, m_ui->horizontalSliderMinConnectGap , Env::minimalFinalExpansionGap );
	
#ifdef USE_QWT
	connect(m_ui->pushButtonShowTrajCost,SIGNAL(clicked()),this,SLOT(showTrajCost()));
	connect(m_ui->pushButtonShowCostProfile,SIGNAL(clicked()),this,SLOT(showCostProfile()));
	connect(m_ui->pushButtonShowHRITrajCost,SIGNAL(clicked()),this,SLOT(showHRITrajCost()));
	connect(m_ui->pushButtonShowTemp,SIGNAL(clicked()),this,SLOT(showTemperature()));
	m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxRescale, Env::initPlot);
	
	m_plot = new BasicPlotWindow();
#endif
  
	qRegisterMetaType< std::vector<double> > ("std::vector<double>");
	connect(ENV.getObject(Env::costAlongTraj), SIGNAL(valueChanged(std::vector<double>)), this, SLOT(setPlotedVector(std::vector<double>)));
	//    connect(m_ui->pushButtonShowTrajCost,SIGNAL(clicked()),this->plot,SLOT(show()));
	connect(m_ui->pushButtonGridInGraph,SIGNAL(clicked()),this,SLOT(putGridInGraph()));
	//    connect(m_ui->pushButtonAStar,SIGNAL(clicked()),this,SLOT(computeAStar()));
	
	// costCriterium 2
	connect(m_ui->comboBoxTrajCostExtimation_2, SIGNAL(currentIndexChanged(int)), m_motionWidget, SLOT(setCostCriterium(int)));
//	connect(ENV.getObject(Env::costDeltaMethod), SIGNAL(valueChanged(int)),m_ui->comboBoxTrajCostExtimation_2, SLOT(setCurrentIndex(int)));
	m_ui->comboBoxTrajCostExtimation_2->setCurrentIndex( MECHANICAL_WORK /*INTEGRAL*/ );
//	
	//connect(m_ui->pushButton2DAstar,SIGNAL(clicked()),this,SLOT(computeGridAndExtract()));
	connect(m_ui->pushButton2DDijkstra,SIGNAL(clicked()),this,SLOT(graphSearchTest()));
	
	connect(m_ui->pushButtonRecomputeGraph,SIGNAL(clicked()),this,SLOT(newGraphAndReComputeCost()));
	connect(m_ui->pushButtonExtractBestPath,SIGNAL(clicked()),this,SLOT(extractBestPath()));
	connect(m_ui->pushButtonStonesGraph,SIGNAL(clicked()),this,SLOT(stonesGraph()));
}


extern void* GroundCostObj;

void CostWidget::initCostSpace()
{  
  if (ENV.getBool(Env::isCostSpace) && global_costSpace ) 
  {
    cout << "Nothing to do" << endl;
    return;
  }

  GlobalCostSpace::initialize();
  
  if (ENV.getBool(Env::enableHri))
  {
#ifdef HRI_PLANNER
    HRICS_init();
    
    this->initCostFunctions();
    this->setCostFunction("costHRI");
    
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
    this->initCostFunctions();
  }
}

// This function is called by the mainwindow
// Constructor after initialzing all widgets
void CostWidget::initCostFunctions()
{
	vector<string> AllCost = global_costSpace->getAllCost();
	m_ui->comboBoxCostFunctions->clear();
	
	for (unsigned int i=0; i<AllCost.size(); i++) 
	{
		QString name(AllCost[i].c_str());
		m_ui->comboBoxCostFunctions->addItem(name);
	}
	
	connect(m_ui->comboBoxCostFunctions, SIGNAL(currentIndexChanged(int)),this, SLOT(setCostFunction(int)));
	m_ui->comboBoxCostFunctions->setCurrentIndex( 0 );
	
	if( GroundCostObj != NULL )
	{
		vector<string> costFunc  = global_costSpace->getAllCost();
		vector<string>::iterator it = std::find( costFunc.begin() , costFunc.end() , "costMap2D" );
		m_ui->comboBoxCostFunctions->setCurrentIndex( it - costFunc.begin() );
	}
	else 
	{
		global_costSpace->setCost("NoCost");
	}
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
        if (costFunctionId< AllCost.size() && costFunctionId>= 0)
        {
            global_costSpace->setCost(AllCost[costFunctionId]);
            cout << "Cost Function is now :  " << AllCost[costFunctionId] << endl;
        }
}

void CostWidget::extractBestPath()
{
	//	p3d_ExtractBestTraj(XYZ_GRAPH);
	Graph* myGraph = new Graph(XYZ_GRAPH);
	Robot* myRobot = myGraph->getRobot();
	
	myGraph->extractBestTraj(myRobot->getInitialPosition(),myRobot->getGoTo());
	
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
	
	shared_ptr<Configuration> Init = ptrGraph->getRobot()->getInitialPosition();
	shared_ptr<Configuration> Goal = ptrGraph->getRobot()->getGoTo();
	
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
	
	cout << "Traj cost = " << traj.costDeltaAlongTraj() << endl;
	
	if (global_costSpace) 
  {
    for( double param=0; param<traj.getRangeMax(); param = param + step)
    {
      cost.push_back( traj.configAtParam(param)->cost() );
    }
    myPlot->setTitle("Cost Space along trajectory");
  }
  else if (optimizer)
  {
    optimizer->getTrajectoryCost(cost,step);
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
	if (!ENV.getBool(Env::enableHri) ) 
	{
		cerr << "Hri is not enabled" << endl;
		return;
	}	

  double kDistanceTmp = ENV.getDouble(Env::Kdistance);
	double kVisibiliTmp = ENV.getDouble(Env::Kvisibility);
	double kReachablTmp = ENV.getDouble(Env::Kreachable);

#if defined(USE_QWT)
	cout << "--------------------------------" << endl;
	
	p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	p3d_traj* CurrentTrajPt = robotPt->tcur;
	
	if (!CurrentTrajPt) 
	{
		cerr << "No trajectory" << endl;
		return;
	}
#if defined(HRI_COSTSPACE)
	ENV.setInt(Env::hriCostType,HRICS_Combine);
#endif
	
	MultiPlot* myPlot = new MultiPlot( m_plot );
	myPlot->setGeometry( m_plot->getPlot()->geometry() );
	int nbSample = myPlot->getPlotSize();
	
	Robot* thisRob = new Robot(robotPt);
	API::Trajectory traj(thisRob,CurrentTrajPt);
	
	cout << "Traj cost = " << traj.costDeltaAlongTraj() << endl;
	
	ENV.setDouble(Env::Kreachable,0.0);
	ENV.setDouble(Env::Kvisibility,0.0);
	ENV.setDouble(Env::Kreachable,0.0);
	
	
	// Compute the 3 costs separatly 
	ENV.setDouble(Env::Kdistance,kDistanceTmp);
	cout << "Distance Cost = " << traj.costDeltaAlongTraj() << endl;
	ENV.setDouble(Env::Kdistance,0.0);
	
	ENV.setDouble(Env::Kvisibility,kVisibiliTmp);
	cout << "Visibility Cost = " << traj.costDeltaAlongTraj() << endl;
	ENV.setDouble(Env::Kvisibility,0);
	
	ENV.setDouble(Env::Kreachable,kReachablTmp);
	cout << "Reachalbilty Cost = " << traj.costDeltaAlongTraj() << endl;
	
	ENV.setDouble(Env::Kdistance,		kDistanceTmp);
	ENV.setDouble(Env::Kvisibility,	kVisibiliTmp);
	ENV.setDouble(Env::Kreachable,	kReachablTmp);
	
	double step = traj.getRangeMax() / (double) nbSample;
	
	//    cout << "Traj param max = " << traj.getRangeMax() << endl;
	//    cout << "Traj step = " << step << endl;
	
	std::ostringstream oss;
	oss << "statFiles/CostAlongTraj_" << std::setw(3) << std::setfill('0') << count_hri_traj++ << ".csv";
  
	const char *res = oss.str().c_str();
	
	std::ofstream s;
	s.open(res);
	
	cout << "Opening save file : " << res << endl;
	
	s << "Dista"  << ";";
	s << "Visib"  << ";";
	s << "Reach"  << ";";
	
	s << endl;
	
	vector<double> costDistance;
	vector<double> costVisibili;
	vector<double> costReachabl;
	for( double param=0; param<traj.getRangeMax(); param = param + step)
	{
		shared_ptr<Configuration> q = traj.configAtParam(param);
		
#ifdef HRI_COSTSPACE
		if(ENV.getBool(Env::HRIPlannerCS))
		{
			q->cost();
			
			double dCost = dynamic_cast<HRICS::ConfigSpace*>(HRICS_MotionPLConfig)->getLastDistanceCost();
			double vCost = dynamic_cast<HRICS::ConfigSpace*>(HRICS_MotionPLConfig)->getLastVisibiliCost();
			costDistance.push_back(dCost);
			costVisibili.push_back(vCost);
		}
		if(ENV.getBool(Env::HRIPlannerWS))
		{
			ENV.setDouble(Env::Kreachable,	0.0);
			ENV.setDouble(Env::Kvisibility,	0.0);
			ENV.setDouble(Env::Kreachable,	0.0);
			
			
			// Compute the 3 costs separatly 
			ENV.setDouble(Env::Kdistance,		kDistanceTmp);
			q->setCostAsNotTested();
			double dCost = q->cost();
			
			ENV.setDouble(Env::Kdistance,		0.0);
			ENV.setDouble(Env::Kvisibility,	kVisibiliTmp);
			q->setCostAsNotTested();
			double vCost = q->cost();
			
			ENV.setDouble(Env::Kvisibility,	0.0);
			ENV.setDouble(Env::Kreachable,	kReachablTmp);
			q->setCostAsNotTested();
			double rCost = q->cost();
			
			// Put 3 costs in vector
			costDistance.push_back( dCost );
			costVisibili.push_back( vCost );
			costReachabl.push_back( rCost );
			
			/*cout	<<	"dCost: "			<< dCost << 
								" , vCost: "	<< vCost << 
								" , rCost: " << rCost << endl;*/
			
			// Save to file
			s << dCost << ";";
			s << vCost << ";";
			s << rCost << ";";
			s << endl;
		}
		//        cout << cost.back() << endl;
#endif
	}
	
	vector< vector<double> > curves;
	curves.push_back( costDistance );
	curves.push_back( costVisibili );
	curves.push_back( costReachabl );
	
	vector< string > plotNames;
	plotNames.push_back( "Distance" );
	plotNames.push_back( "Visibility" );
	plotNames.push_back( "Reachability" );
	
	myPlot->setData( plotNames , curves );
	
	delete m_plot->getPlot();
	m_plot->setPlot(myPlot);
	m_plot->show();
	
	s.close();
#endif
	
	cout << "Closing save file" << endl;
	
	ENV.setDouble(Env::Kdistance,		kDistanceTmp);
	ENV.setDouble(Env::Kvisibility,	kVisibiliTmp);
	ENV.setDouble(Env::Kreachable,	kReachablTmp);

	
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
		
		//        N = new Node(ptrGraph,rob->getGoTo());
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


