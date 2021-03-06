/*
 *  qtMotionPlanner.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "qtMotionPlanner.hpp"
#include "ui_qtMotionPlanner.h"

#include <iostream>

#include "qtBase/SpinBoxSliderConnector_p.hpp"

#ifdef USE_QWT_5
#include "qtPlot/multiPlot.hpp"
#endif

#include "planner_handler.hpp"

#include <libmove3d/include/P3d-pkg.h>
#include <libmove3d/include/Planner-pkg.h>
#include <libmove3d/p3d/ParametersEnv.hpp>

#include "planner/planner.hpp"
#include "planner/planEnvironment.hpp"
#include "planner/cost_space.hpp"

#include "API/project.hpp"
#include "API/Roadmap/compco.hpp"
#include "API/Roadmap/node.hpp"
#include "API/Trajectory/trajectory.hpp"

#include "SaveContext.hpp"
#include "MultiRun.hpp"

#include "hri_costspace/HRICS_navigation.hpp"

// using namespace std;
using std::cout;
using std::endl;
using namespace QtShiva;
using namespace Move3D;

MOVE3D_USING_SHARED_PTR_NAMESPACE

MotionPlanner::MotionPlanner(QWidget* parent)
    : QWidget(parent), m_ui(new Ui::MotionPlanner) {
  m_ui->setupUi(this);

  initDiffusion();
  initPRM();
  initOptim();
  initMultiRun();
  initGeneral();
  initShowGraph();
  initAStar();
  initGraphSampling();
}

MotionPlanner::~MotionPlanner() { delete m_ui; }

//---------------------------------------------------------------------
// GENERAL
//---------------------------------------------------------------------
void MotionPlanner::initGeneral() {
  connect(m_ui->pushButtonCheckAllEdges,
          SIGNAL(clicked()),
          this,
          SLOT(checkAllEdges()));
  connect(m_ui->pushButtonAStarInCurrentGraph,
          SIGNAL(clicked()),
          this,
          SLOT(computeAStarInCurrentGraph()));
  connect(m_ui->pushButtonPlanSequence,
          SIGNAL(clicked()),
          this,
          SLOT(planASequenceOfPlans()));

  // Connecting the Dmax Env variable
  m_ui->doubleSpinBoxDMax->setMaximum(p3d_get_env_dmax());
  m_ui->doubleSpinBoxDMax->setValue(p3d_get_env_dmax());
  connect(m_ui->doubleSpinBoxDMax,
          SIGNAL(valueChanged(double)),
          SLOT(envDmaxSpinBoxValueChanged(double)));
  connect(m_ui->doubleSpinBoxDMax,
          SIGNAL(valueChanged(double)),
          ENV.getObject(Env::dmax),
          SLOT(set(double)));
  connect(ENV.getObject(Env::dmax),
          SIGNAL(valueChanged(double)),
          m_ui->doubleSpinBoxDMax,
          SLOT(setValue(double)));
  m_ui->checkBoxSmooth->setChecked(
      PlanEnv->getBool(PlanParam::env_createTrajs));

//	connect(m_ui->doubleSpinBoxTest, SIGNAL(valueChanged( double )),
//PlanEnv->getObject(PlanParam::eleven),SLOT(set(double)));
//	connect(PlanEnv->getObject(PlanParam::eleven),
//SIGNAL(valueChanged(double)),m_ui->doubleSpinBoxTest, SLOT(setValue(double)));
//	connect(PlanEnv->getObject(PlanParam::eleven),
//SIGNAL(valueChanged(double)),this,SLOT(testParam(double)));

#ifdef MULTILOCALPATH
  new connectCheckBoxToEnv(m_ui->checkBoxGnuplot,
                           ENV.getObject(Env::plotSoftMotionCurve));
  // WARNING REVERT XAVIER
  // new
  // connectCheckBoxToEnv(m_ui->checkBoxTrajAsArray,ENV.getObject(Env::exportSoftMotionTrajAsArrayOfConf));
  new connectCheckBoxToEnv(m_ui->checkBoxExportFiles,
                           ENV.getObject(Env::writeSoftMotionFiles));
#endif
}

void MotionPlanner::testParam(double param) {
  // cout << "Value changed : " << PlanEnv->getDouble(PlanParam::eleven) <<
  // endl;
}

void MotionPlanner::checkAllEdges() { emit selectedPlanner("checkAllEdges"); }

void MotionPlanner::computeAStarInCurrentGraph() {
  emit selectedPlanner("ComputeAStarInCurrentGraph");
}

void MotionPlanner::planASequenceOfPlans() {
  emit selectedPlanner("PlanSequence");
}

void MotionPlanner::envDmaxSpinBoxValueChanged(double dmax) {
  p3d_set_env_dmax(dmax);
}

void MotionPlanner::envIsWithGoalValueChanged(bool state) {
  if (!state && m_ui->isBidir->isChecked()) {
    m_ui->isBidir->setChecked(false);
    ENV.setBool(Env::biDir, false);
  }
  ENV.setBool(Env::expandToGoal, state);
  m_ui->isWithGoal->setChecked(state);
}

void MotionPlanner::envBiDirValueChanged(bool state) {
  if (state && !m_ui->isWithGoal->isChecked()) {
    m_ui->isWithGoal->setChecked(true);
    ENV.setBool(Env::expandToGoal, true);
  }
  ENV.setBool(Env::biDir, state);
  m_ui->isBidir->setChecked(state);
}

void MotionPlanner::envRandomConnectionToGoalValueChanged(bool state) {
  ENV.setBool(Env::tryClosest, !state);
  m_ui->checkBoxClosestInCompCo->setChecked(!state);
}

void MotionPlanner::envTryClosestValueChanged(bool state) {
  ENV.setBool(Env::randomConnectionToGoal, !state);
  m_ui->checkBoxRandomInCompCo->setChecked(!state);
}

void MotionPlanner::envMultiRRTValueChanged(bool state) {
  if (state) {
    m_ui->spinBoxNumberOfSeeds->setEnabled(true);
  } else {
    m_ui->spinBoxNumberOfSeeds->setDisabled(true);
  }
}

void MotionPlanner::envPRMTypeChanged(int type) {
  if (type == 2) {
    m_ui->label->setVisible(true);
    m_ui->spinBoxMaxConnect->setVisible(true);
  } else {
    m_ui->label->setVisible(false);
    m_ui->spinBoxMaxConnect->setVisible(false);
  }
}

//---------------------------------------------------------------------
// DIFFUSION
//---------------------------------------------------------------------
void MotionPlanner::initDiffusion() {
  new connectCheckBoxToEnv(m_ui->isWithGoal, ENV.getObject(Env::expandToGoal));
  new connectCheckBoxToEnv(m_ui->isBidir, ENV.getObject(Env::biDir));

  connect(m_ui->isWithGoal,
          SIGNAL(toggled(bool)),
          SLOT(envIsWithGoalValueChanged(bool)));
  connect(
      m_ui->isBidir, SIGNAL(toggled(bool)), SLOT(envBiDirValueChanged(bool)));

  new connectCheckBoxToEnv(m_ui->isManhattan, ENV.getObject(Env::isManhattan));
#ifndef BIO
  m_ui->isManhattan->setDisabled(true);
#endif

  new connectCheckBoxToEnv(m_ui->isEST, ENV.getObject(Env::treePlannerIsEST));
  new connectCheckBoxToEnv(m_ui->isBalanced,
                           ENV.getObject(Env::expandBalanced));
  new connectCheckBoxToEnv(m_ui->isRefinementControl,
                           ENV.getObject(Env::refinementControl));
  new connectCheckBoxToEnv(m_ui->isDiscardingNodes,
                           ENV.getObject(Env::discardNodes));
  new connectCheckBoxToEnv(m_ui->checkBoxIsGoalBias,
                           ENV.getObject(Env::isGoalBiased));
  new connectCheckBoxToEnv(m_ui->checkBoxRandomInCompCo,
                           ENV.getObject(Env::randomConnectionToGoal));
  new connectCheckBoxToEnv(m_ui->checkBoxClosestInCompCo,
                           ENV.getObject(Env::tryClosest));

  connect(m_ui->checkBoxRandomInCompCo,
          SIGNAL(toggled(bool)),
          SLOT(envRandomConnectionToGoalValueChanged(bool)));
  connect(m_ui->checkBoxClosestInCompCo,
          SIGNAL(toggled(bool)),
          SLOT(envTryClosestValueChanged(bool)));

  connect(m_ui->expansionMethod,
          SIGNAL(currentIndexChanged(int)),
          &ENV,
          SLOT(setExpansionMethodSlot(int)),
          Qt::DirectConnection);
  connect(&ENV,
          SIGNAL(expansionMethodChanged(int)),
          m_ui->expansionMethod,
          SLOT(setCurrentIndex(int)));
  m_ui->expansionMethod->setCurrentIndex((int)ENV.getExpansionMethod());

  connect(m_ui->spinBoxMaxNodes,
          SIGNAL(valueChanged(int)),
          ENV.getObject(Env::maxNodeCompco),
          SLOT(set(int)));
  connect(ENV.getObject(Env::maxNodeCompco),
          SIGNAL(valueChanged(int)),
          m_ui->spinBoxMaxNodes,
          SLOT(setValue(int)));
  m_ui->spinBoxMaxNodes->setValue(ENV.getInt(Env::maxNodeCompco));

  connect(m_ui->spinBoxMaxIterations,
          SIGNAL(valueChanged(int)),
          PlanEnv->getObject(PlanParam::plannerMaxIterations),
          SLOT(set(int)));
  connect(PlanEnv->getObject(PlanParam::plannerMaxIterations),
          SIGNAL(valueChanged(int)),
          m_ui->spinBoxMaxIterations,
          SLOT(setValue(int)));
  m_ui->spinBoxMaxIterations->setValue(
      PlanEnv->getInt(PlanParam::plannerMaxIterations));

  connect(m_ui->spinBoxNbTry,
          SIGNAL(valueChanged(int)),
          ENV.getObject(Env::NbTry),
          SLOT(set(int)));
  connect(ENV.getObject(Env::NbTry),
          SIGNAL(valueChanged(int)),
          m_ui->spinBoxNbTry,
          SLOT(setValue(int)));
  m_ui->spinBoxNbTry->setValue(ENV.getInt(Env::NbTry));

  new QtShiva::SpinBoxSliderConnector(this,
                                      m_ui->doubleSpinBoxExtentionStep,
                                      m_ui->horizontalSliderExtentionStep,
                                      ENV.getObject(Env::extensionStep));
  new QtShiva::SpinBoxSliderConnector(this,
                                      m_ui->doubleSpinBoxBias,
                                      m_ui->horizontalSliderBias,
                                      ENV.getObject(Env::Bias));

  initMultiRRT();
}

void MotionPlanner::initMultiRRT() {
  new connectCheckBoxToEnv(m_ui->checkBoxMultiRRT,
                           ENV.getObject(Env::isMultiRRT));
  connect(m_ui->checkBoxMultiRRT,
          SIGNAL(toggled(bool)),
          SLOT(envMultiRRTValueChanged(bool)));
  connect(m_ui->spinBoxNumberOfSeeds,
          SIGNAL(valueChanged(int)),
          ENV.getObject(Env::nbOfSeeds),
          SLOT(set(int)));
  connect(ENV.getObject(Env::nbOfSeeds),
          SIGNAL(valueChanged(int)),
          m_ui->spinBoxNumberOfSeeds,
          SLOT(setValue(int)));
  m_ui->spinBoxNumberOfSeeds->setDisabled(true);
}
// void MainWindow::setLineEditWithNumber(Env::intParameter p,int num)
//{
//    if(p == Env::maxNodeCompco)
//    {
//        m_ui->lineEditMaxNodes->setText(QString::number(num));
//    }
//}

//---------------------------------------------------------------------
// PRM
//---------------------------------------------------------------------
void MotionPlanner::initPRM() {
  new connectCheckBoxToEnv(m_ui->checkBoxUseDistance,
                           ENV.getObject(Env::useDist));
  new connectCheckBoxToEnv(m_ui->checkBox_9,
                           PlanEnv->getObject(PlanParam::orientedGraph));
  // PRMType
  connect(m_ui->comboBoxPRMType,
          SIGNAL(currentIndexChanged(int)),
          ENV.getObject(Env::PRMType),
          SLOT(set(int)));
  connect(ENV.getObject(Env::PRMType),
          SIGNAL(valueChanged(int)),
          m_ui->comboBoxPRMType,
          SLOT(setCurrentIndex(int)));
  connect(m_ui->comboBoxPRMType,
          SIGNAL(currentIndexChanged(int)),
          SLOT(envPRMTypeChanged(int)));
  m_ui->comboBoxPRMType->setCurrentIndex(0 /*INTEGRAL*/);
  // 0 => PRM
  // 1 => Visib
  // 2 => ACR

  //    connect(ENV.getObject(Env::PRMType),
  //    SIGNAL(valueChanged(int)),m_ui->comboBoxPRMType,
  //    SLOT(setCurrentIndex(int)));

  m_ui->spinBoxMaxConnect->setValue(ENV.getInt(Env::maxConnect));
  connect(m_ui->spinBoxMaxConnect,
          SIGNAL(valueChanged(int)),
          ENV.getObject(Env::maxConnect),
          SLOT(set(int)));
  connect(ENV.getObject(Env::maxConnect),
          SIGNAL(valueChanged(int)),
          m_ui->spinBoxMaxConnect,
          SLOT(setValue(int)));
  m_ui->spinBoxMaxConnect->setVisible(false);
  m_ui->label->setVisible(false);
}

//---------------------------------------------------------------------
// OPTIM
//---------------------------------------------------------------------
void MotionPlanner::initOptim() {
  new connectCheckBoxToEnv(m_ui->checkBoxCostSpace2,
                           ENV.getObject(Env::isCostSpace));
  new connectCheckBoxToEnv(m_ui->checkBoxUseCostSmooth,
                           PlanEnv->getObject(PlanParam::trajUseCost));
  new connectCheckBoxToEnv(m_ui->checkBoxDebug2,
                           ENV.getObject(Env::debugCostOptim));

  new connectCheckBoxToEnv(
      m_ui->checkBoxExtractCurrentTraj,
      PlanEnv->getObject(PlanParam::rrtExtractShortestPath));

  new connectCheckBoxToEnv(m_ui->checkBoxSaveTrajCost,
                           PlanEnv->getObject(PlanParam::trajSaveCost));
  new connectCheckBoxToEnv(m_ui->checkBoxPartialShortcut,
                           PlanEnv->getObject(PlanParam::trajPartialShortcut));
  new connectCheckBoxToEnv(m_ui->checkBoxRecomputeCost,
                           PlanEnv->getObject(PlanParam::trajCostRecompute));
  new connectCheckBoxToEnv(m_ui->checkBoxCheckCollision,
                           PlanEnv->getObject(PlanParam::trajComputeCollision));
  new connectCheckBoxToEnv(m_ui->checkBoxNPoints,
                           PlanEnv->getObject(PlanParam::trajNPoints));

  new connectCheckBoxToEnv(m_ui->checkBoxWithTimeLimitSmoothing,
                           PlanEnv->getObject(PlanParam::trajWithTimeLimit));
  new connectCheckBoxToEnv(m_ui->checkBoxWithTimeLimitPlanning,
                           PlanEnv->getObject(PlanParam::planWithTimeLimit));

  new connectCheckBoxToEnv(m_ui->checkBoxWithDescent,
                           PlanEnv->getObject(PlanParam::withDescent));
  new connectCheckBoxToEnv(m_ui->checkBoxWithDeform,
                           PlanEnv->getObject(PlanParam::withDeformation));
  new connectCheckBoxToEnv(m_ui->checkBoxWithShortCut,
                           PlanEnv->getObject(PlanParam::withShortCut));
  new connectCheckBoxToEnv(m_ui->checkBoxWithStomp,
                           PlanEnv->getObject(PlanParam::withStomp));
  new connectCheckBoxToEnv(
      m_ui->checkBoxStompWithTimeLimit,
      PlanEnv->getObject(PlanParam::trajStompWithTimeLimit));
  new connectCheckBoxToEnv(m_ui->checkBoxWithGainLimit,
                           PlanEnv->getObject(PlanParam::withGainLimit));
  new connectCheckBoxToEnv(m_ui->checkBoxWithIterLimit,
                           PlanEnv->getObject(PlanParam::withMaxIteration));
  new connectCheckBoxToEnv(m_ui->checkBoxShowExploration,
                           PlanEnv->getObject(PlanParam::showExploration));

  new connectCheckBoxToEnv(m_ui->checkBoxPartialShortcut,
                           PlanEnv->getObject(PlanParam::trajPartialShortcut));

  // Main functions for shortcut and optimization
  connect(this,
          SIGNAL(selectedPlanner(QString)),
          global_plannerHandler,
          SLOT(startPlanner(QString)));
  connect(m_ui->pushButtonRandomShortCut,
          SIGNAL(clicked()),
          this,
          SLOT(shortCutCost()));
  connect(m_ui->pushButtonTriangleDeformation,
          SIGNAL(clicked()),
          this,
          SLOT(optimizeCost()));

  connect(m_ui->pushButtonRemoveRedundantNodes,
          SIGNAL(clicked()),
          this,
          SLOT(removeRedundant()));
  connect(m_ui->pushButtonEraseDebugTraj,
          SIGNAL(clicked()),
          this,
          SLOT(eraseDebugTraj()));
  connect(m_ui->pushButtonCutTrajInSmallLP,
          SIGNAL(clicked()),
          this,
          SLOT(cutTrajInSmallLP()));
  connect(m_ui->pushButtonCutTrajAndOptimizeSM,
          SIGNAL(clicked()),
          this,
          SLOT(cutTrajAndOptimizeSM()));

  // costCriterium
  connect(ENV.getObject(Env::costDeltaMethod),
          SIGNAL(valueChanged(int)),
          this,
          SLOT(setCostCriterium(int)),
          Qt::DirectConnection);
  //		connect( ENV.getObject(Env::costDeltaMethod),
  //SIGNAL(valueChanged(int)),				m_ui->comboBoxTrajCostExtimation,
  //SLOT(setCurrentIndex(int)));
  connect(m_ui->comboBoxTrajCostExtimation,
          SIGNAL(currentIndexChanged(int)),
          this,
          SLOT(setCostCriterium(int)));
  m_ui->comboBoxTrajCostExtimation->setCurrentIndex(
      /*MECHANICAL_WORK*/ INTEGRAL);
  setCostCriterium(MECHANICAL_WORK);

  new QtShiva::SpinBoxSliderConnector(
      this,
      m_ui->doubleSpinBoxNbRounds,
      m_ui->horizontalSliderNbRounds,
      PlanEnv->getObject(PlanParam::smoothMaxIterations));
  //	new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxNbRounds,
  //m_ui->horizontalSliderNbRounds , Env::nbCostOptimize );

  new QtShiva::SpinBoxSliderConnector(
      this,
      m_ui->doubleSpinBoxTimeLimitSmoothing,
      m_ui->horizontalSliderTimeLimitSmoothing,
      PlanEnv->getObject(PlanParam::timeLimitSmoothing));
  new QtShiva::SpinBoxSliderConnector(
      this,
      m_ui->doubleSpinBoxTimeLimitPlanning,
      m_ui->horizontalSliderTimeLimitPlanning,
      PlanEnv->getObject(PlanParam::timeLimitPlanning));

  connect(m_ui->pushButtonRunMultiSmooth,
          SIGNAL(clicked()),
          this,
          SLOT(runMultiSmooth()));
  connect(m_ui->pushButtonSimpleMultiRRT,
          SIGNAL(clicked()),
          this,
          SLOT(runAllRRT()));

  // ------------------------------------------------------------
  // ------------------------------------------------------------

  new QtShiva::SpinBoxSliderConnector(this,
                                      m_ui->doubleSpinBoxMaxDeformStep,
                                      m_ui->horizontalSliderMaxDeformStep,
                                      PlanEnv->getObject(PlanParam::MaxFactor));

  connect(m_ui->spinBoxIthNodeInTraj,
          SIGNAL(valueChanged(int)),
          this,
          SLOT(getIthNodeInBestTraj()),
          Qt::QueuedConnection);

  QtShiva::SpinBoxConnector(this,
                            m_ui->doubleSpinBoxStompTimeLimit,
                            PlanEnv->getObject(PlanParam::trajStompTimeLimit));
  QtShiva::SpinBoxConnector(
      this, m_ui->spinBoxNbMultiSmooth, ENV.getObject(Env::nbMultiSmooth));

  // connect(connector,SIGNAL(valueChanged(double)),PlanEnv->getObject(PlanParam::timeLimitSmoothing),SLOT(set(double)));
  // connect(PlanEnv->getObject(PlanParam::timeLimitSmoothing),SIGNAL(valueChanged(double)),this,SLOT(test(double)));
  // connect(connector1,SIGNAL(valueChanged(double)),this,SLOT(test(double)));

  //
  //	PlanEnv->getObject(PlanParam::timeLimitSmoothing)->dumpObjectInfo();
  //	PlanEnv->setDouble(PlanParam::timeLimitSmoothing,15.0);
  //	PlanEnv->setDouble(PlanParam::timeLimitSmoothing,30.0);
  // PlanEnv->getObject(PlanParam::timeLimitSmoothing)->metaObject()->connectSlotsByName
  // ();
}

void MotionPlanner::test(double value) {
  // cout << "Value of PlanParam::timeLimitSmoothing : " <<
  // PlanEnv->getDouble(PlanParam::timeLimitSmoothing) << endl;
  cout << "Value of PlanParam::timeLimitSmoothing : "
       << PlanEnv->getDouble(PlanParam::MinStep) << endl;
}

void MotionPlanner::cutTrajInSmallLP() {
  Robot* rob = global_Project->getActiveScene()->getActiveRobot();
  double dmax = global_Project->getActiveScene()->getDMax();

  Move3D::Trajectory traj = rob->getCurrentTraj();

  cout << "Cutting into small LP" << endl;
  traj.cutTrajInSmallLP(floor(traj.getParamMax() / dmax));
  traj.replaceP3dTraj();
}

int __traj_id = 0;

void MotionPlanner::cutTrajAndOptimizeSM() {
  Robot* rob = global_Project->getActiveScene()->getActiveRobot();

#ifdef LIGHT_PLANNER
  MANPIPULATION_TRAJECTORY_CONF_STR confs;
  SM_TRAJ smTraj;
  if (p3d_convert_traj_to_softMotion(rob->getP3dRobotStruct()->tcur,
                                     ENV.getBool(Env::smoothSoftMotionTraj),
                                     true,
                                     false,
                                     confs.first,
                                     confs.second,
                                     smTraj) == 1) {
    printf(
        "p3d_optim_traj_softMotion : cannot compute the softMotion "
        "trajectory\n");
    return;
  } else {
    cout << "Robot tcur is converted to SoftMotion" << endl;
  }
  __traj_id++;
#endif

  //#if defined(USE_QWT_5)
  //	cout << "Plot speed --------------------------------" << endl;
  //
  //	MultiPlot* myPlot = new MultiPlot(this->plot);
  //	myPlot->setGeometry(this->plot->getPlot()->geometry());
  //
  //	vector< vector<double> > curves(7);
  //	curves[0] = confs.second[0];
  //	curves[1] = confs.second[1];
  //	curves[2] = confs.second[2];
  //  curves[3] = confs.second[3];
  //  curves[4] = confs.second[4];
  //  curves[5] = confs.second[5];
  //  curves[6] = confs.second[6];
  //
  //  for (unsigned int i=0; i<curves.size(); i++)
  //  {
  //    for (unsigned int j=1; j<curves[i].size(); j++)
  //    {
  //      curves[i][j] = (curves[i][j]-curves[i][j-1])/0.01;
  //    }
  //  }
  //
  //	vector< string > plotNames;
  //	plotNames.push_back( "V1" );
  //	plotNames.push_back( "V2" );
  //	plotNames.push_back( "V3" );
  //	plotNames.push_back( "V4" );
  //  plotNames.push_back( "V5" );
  //  plotNames.push_back( "V6" );
  //  plotNames.push_back( "V7" );
  //
  //	myPlot->setData( plotNames , curves );
  //
  //	delete this->plot->getPlot();
  //	this->plot->setPlot(myPlot);
  //	this->plot->show();
  //#endif
}

void MotionPlanner::eraseDebugTraj() { global_trajToDraw.clear(); }

void MotionPlanner::setCostCriterium(int choice) {
// cout << "Set Delta Step Choise to " << choice << endl;
#ifdef P3D_PLANNER
  p3d_SetDeltaCostChoice(choice);
#endif

  std::map<int, CostSpaceDeltaStepMethod> methods;

  methods.insert(std::pair<int, CostSpaceDeltaStepMethod>(MECHANICAL_WORK,
                                                          cs_mechanical_work));
  methods.insert(
      std::pair<int, CostSpaceDeltaStepMethod>(INTEGRAL, cs_integral));
  methods.insert(std::pair<int, CostSpaceDeltaStepMethod>(2, cs_visibility));
  methods.insert(std::pair<int, CostSpaceDeltaStepMethod>(3, cs_average));
  methods.insert(
      std::pair<int, CostSpaceDeltaStepMethod>(4, cs_config_cost_and_dist));

  ENV.setInt(Env::costDeltaMethod, choice);
  // m_ui->comboBoxTrajCostExtimation->setCurrentIndex(choice);
}

void MotionPlanner::computeGrid() {
  p3d_rob* robotPt = (p3d_rob*)p3d_get_desc_curid(P3D_ROBOT);
#ifdef P3D_PLANNER
  p3d_CreateDenseRoadmap(robotPt);
#endif
}

void MotionPlanner::runMultiSmooth() { emit selectedPlanner("MultiSmooth"); }

/**
 * @ingroup qtWindow
 * @brief Planner thread class
 */
//-----------------------------------------------
void MotionPlanner::optimizeCost() {
  emit(selectedPlanner(QString("Optimize")));
}

void MotionPlanner::shortCutCost() {
  emit(selectedPlanner(QString("Shortcut")));
}

void MotionPlanner::removeRedundant() {
  ENV.setBool(Env::isRunning, true);
#ifdef WITH_XFORMS
  std::string str = "removeRedunantNodes";
  write(qt_fl_pipe[1], str.c_str(), str.length() + 1);
#else
  cout << "Not implemented" << endl;
#endif
}

void MotionPlanner::extractBestTraj() {
#ifdef P3D_PLANNER
  p3d_ExtractBestTraj(XYZ_GRAPH);
#endif
}

Move3D::Node* MotionPlanner::getIthNodeInBestTraj() {
  if (API_activeGraph == NULL) {
    cout << "Graph is NULL" << endl;
  }

  confPtr_t q_init = global_Move3DPlanner->getInitConf();
  confPtr_t q_goal = global_Move3DPlanner->getGoalConf();

  std::pair<bool, std::vector<Node*> > nodes =
      API_activeGraph->extractBestNodePathSoFar(q_init, q_goal);

  if (nodes.second.empty()) {
    cout << "nodes is empty!!!" << endl;
    return NULL;
  }

  int ith = m_ui->spinBoxIthNodeInTraj->value();

  //		cout << "removing node nb : " << ith << endl;
  //		cout << "graph size nb : " << nodes.size() << endl;
  if ((ith >= 0) && (((int)nodes.second.size()) > ith)) {
    Robot* rob = API_activeGraph->getRobot();
    rob->setAndUpdate(*nodes.second[ith]->getConfiguration());
    m_mainWindow->drawAllWinActive();
    return nodes.second[ith];
  } else {
    cout << "out of bounds" << endl;
    return NULL;
  }
}

//---------------------------------------------------------------------
// Multiple Runs
//---------------------------------------------------------------------
void MotionPlanner::initMultiRun() {
  connect(m_ui->pushButtonSaveContext,
          SIGNAL(clicked()),
          this,
          SLOT(saveContext()));
  connect(m_ui->pushButtonSetSelected,
          SIGNAL(clicked()),
          this,
          SLOT(setToSelected()));
  connect(m_ui->pushButtonPrintSelected,
          SIGNAL(clicked()),
          this,
          SLOT(printContext()));
  connect(m_ui->pushButtonDeleteSelected,
          SIGNAL(clicked()),
          this,
          SLOT(deleteSelected()));
  connect(m_ui->pushButtonPrintAllContext,
          SIGNAL(clicked()),
          this,
          SLOT(printAllContext()));
  connect(m_ui->pushButtonResetContext,
          SIGNAL(clicked()),
          this,
          SLOT(resetContext()));

  contextList = new QListWidget;
  m_ui->multiRunLayout->addWidget(contextList);

  connect(m_ui->horizontalSliderNbMutliRun,
          SIGNAL(valueChanged(int)),
          m_ui->spinBoxNbMutliRun,
          SLOT(setValue(int)));
  connect(m_ui->spinBoxNbMutliRun,
          SIGNAL(valueChanged(int)),
          ENV.getObject(Env::nbMultiRun),
          SLOT(set(int)));
  connect(m_ui->spinBoxNbMutliRun,
          SIGNAL(valueChanged(int)),
          m_ui->horizontalSliderNbMutliRun,
          SLOT(setValue(int)));
  connect(ENV.getObject(Env::nbMultiRun),
          SIGNAL(valueChanged(int)),
          m_ui->spinBoxNbMutliRun,
          SLOT(setValue(int)));
  m_ui->spinBoxNbMutliRun->setValue(ENV.getInt(Env::nbMultiRun));

  connect(
      m_ui->pushButtonRunAllRRT, SIGNAL(clicked()), this, SLOT(runAllRRT()));
  connect(m_ui->pushButtonRunAllGreedy,
          SIGNAL(clicked()),
          this,
          SLOT(runAllGreedy()));
  connect(m_ui->pushButtonShowHisto,
          SIGNAL(clicked()),
          this,
          SLOT(showHistoWindow()));

  new connectCheckBoxToEnv(m_ui->checkBoxStopMultiRun,
                           ENV.getObject(Env::StopMultiRun));
}

void MotionPlanner::saveContext() {
  if (itemList.size() != storedContext.getNumberStored()) {
    storedContext.clear();
  }

  if (m_ui->lineEditContext->text() == "") {
    std::ostringstream oss;
    oss << "Context" << itemList.size();

    m_ui->lineEditContext->setText(oss.str().c_str());
  }
  QListWidgetItem* item = new QListWidgetItem(contextList);
  itemList.push_back(item);

  ENV.setString(Env::nameOfFile, m_ui->lineEditContext->text().toStdString());

  itemList.back()->setText(m_ui->lineEditContext->text());

  storedContext.saveCurrentEnvToStack();
  //	storedPlannerContext->saveCurrentEnvToStack();
}

void MotionPlanner::printAllContext() {
  if (storedContext.getNumberStored() > 0) {
    for (uint i = 0; i < storedContext.getNumberStored(); i++) {
      std::cout << "------------ Context Number " << i << " ------------"
                << std::endl;
      storedContext.printData(i);
      //			storedPlannerContext->printData(i);
    }
    std::cout << "-------------------------------------------" << std::endl;
    std::cout << " Total number of contexts in stack =  "
              << storedContext.getNumberStored() << std::endl;
    std::cout << "-------------------------------------------" << std::endl;
  } else {
    std::cout << "Warning: no context in stack" << std::endl;
  }
}

void MotionPlanner::setToSelected() {
  if (storedContext.getNumberStored() > 0) {
    int i = contextList->currentRow();
    storedContext.switchCurrentEnvTo(i);
    //		storedPlannerContext->switchCurrentEnvTo(i);
  } else {
    std::cout << "Warning: no context in stack" << std::endl;
  }
}

void MotionPlanner::deleteSelected() {
  if (storedContext.getNumberStored() > 0) {
    int i = contextList->currentRow();
    storedContext.deleteEnv(i);
    //		storedPlannerContext->deleteEnv(i);
    std::cout << "Delete context " << i << std::endl;
    contextList->takeItem(contextList->row(contextList->currentItem()));
  } else {
    std::cout << "Warning: no context in stack" << std::endl;
  }
}

void MotionPlanner::printContext() {
  if (storedContext.getNumberStored() > 0) {
    int i = contextList->currentRow();
    std::cout << "------------ Context Number " << i << " ------------"
              << std::endl;
    storedContext.printData(i);
    //		storedPlannerContext->printData(i);
  } else {
    std::cout << "Warning: no context in stack" << std::endl;
  }
}

void MotionPlanner::resetContext() {
  storedContext.clear();
  //	storedPlannerContext->clear();
  //	setContextUserApp(context);
  for (unsigned int i = 0; i < itemList.size(); i++) {
    delete itemList.at(i);
  }
  itemList.clear();
}

/**
 * @ingroup qtWindow
 * @brief Multi Planner thread class
 */
//-----------------------------------------------
MultiThread::MultiThread(bool isRRT, QObject* parent)
    : QThread(parent), m_isRRT(isRRT) {}

void MultiThread::run() {
  if (m_isRRT) {
    MultiRun multiRRTs;
    multiRRTs.runMultiRRT();
  }

  cout << "Ends Multi Thread" << endl;
}
//-----------------------------------------------
void MotionPlanner::runMultiRRT() { emit selectedPlanner("MultiRRT"); }

void MotionPlanner::runAllRRT() {
//	runAllRounds->setDisabled(true);
#ifdef WITH_XFORMS
  std::string str = "MultiRRT";
  write(qt_fl_pipe[1], str.c_str(), str.length() + 1);
#else
  MultiThread* ptrPlan = new MultiThread(true);
  ptrPlan->start();

#endif
}

void MotionPlanner::runAllGreedy() {
  //	runAllRounds->setDisabled(true);
  std::string str = "MultiGreedy";
#ifdef WITH_XFORMS
  write(qt_fl_pipe[1], str.c_str(), str.length() + 1);
#endif
}

void MotionPlanner::showHistoWindow() {
#ifdef USE_QWT_5
  histoWin = new HistoWindow();
  histoWin->startWindow();
#endif
}

//-----------------------------------------------
// Show Graph
//-----------------------------------------------
void MotionPlanner::initShowGraph() {
  // connect(m_ui->spinBoxNodeToShow,
  // SIGNAL(valueChanged(int)),ENV.getObject(Env::cellToShow),SLOT(set(int)),
  // Qt::DirectConnection);
  connect(m_ui->spinBoxNodeToShow,
          SIGNAL(valueChanged(int)),
          this,
          SLOT(nodeToShowChanged()),
          Qt::QueuedConnection);
  connect(m_ui->spinBoxEdgeToShow,
          SIGNAL(valueChanged(int)),
          this,
          SLOT(edgeToShowChanged()),
          Qt::QueuedConnection);
  connect(
      m_ui->pushButtonRemoveNode, SIGNAL(clicked()), this, SLOT(removeNode()));
}

void MotionPlanner::nodeToShowChanged() {
  if (!API_activeGraph) {
    return;
  }

  std::vector<Move3D::Node*> nodes = API_activeGraph->getNodes();

  if (nodes.empty()) {
    cout << "Warning :: nodes is empty!!!" << endl;
  }

  int ith = m_ui->spinBoxNodeToShow->value();

  cout << "Showing node nb : " << ith << endl;
  cout << "graph size nb : " << nodes.size() << endl;

  if ((ith >= 0) && (((int)nodes.size()) > ith)) {
    Move3D::Robot* rob = Move3D::API_activeGraph->getRobot();
    rob->setAndUpdate(*nodes[ith]->getConfiguration());

    cout << "Node number : " << nodes[ith]->getNodeStruct()->num << endl;
    cout << "Connected Component : "
         << nodes[ith]->getConnectedComponent()->getId() << endl;
  } else {
    confPtr_t q_init = API_activeGraph->getRobot()->getInitPos();
    API_activeGraph->getRobot()->setAndUpdate(*q_init);
    cout << "Exede the number of nodes" << endl;
  }
  m_mainWindow->drawAllWinActive();
}

void MotionPlanner::edgeToShowChanged() {
  if (!API_activeGraph) {
    return;
  }

  std::vector<Edge*> edges = API_activeGraph->getEdges();

  if (edges.empty()) {
    cout << "Warning :: edges is empty!!!" << endl;
  }

  int ith = m_ui->spinBoxEdgeToShow->value();

  cout << "Showing edge nb : " << ith << endl;
  cout << "graph size nb : " << edges.size() << endl;

  if ((ith >= 0) && (((int)edges.size()) > ith)) {
    Robot* rob = API_activeGraph->getRobot();
    MOVE3D_PTR_NAMESPACE::shared_ptr<LocalPath> lp = edges[ith]->getLocalPath();
    configPt q = lp->getP3dLocalpathStruct()->config_at_distance(
        rob->getP3dRobotStruct(),
        lp->getP3dLocalpathStruct(),
        lp->length() / 2);
    Configuration* config = new Configuration(rob);
    config->setConfiguration(q);
    rob->setAndUpdate(*config);

    //        cout << "Edge number : " << edges[ith]->getNodeStruct()->num <<
    //        endl ;
    //        cout << "Connected Component : " <<
    //        edges[ith]->getConnectedComponent()->getId() << endl ;
  } else {
    confPtr_t q_init = API_activeGraph->getRobot()->getInitPos();
    API_activeGraph->getRobot()->setAndUpdate(*q_init);
    cout << "Exede the number of edges" << endl;
  }
  m_mainWindow->drawAllWinActive();
}

Node* MotionPlanner::getIthNodeInActiveGraph() {
  if (!API_activeGraph) {
    cout << "Graph is NULL" << endl;
    return NULL;
  }

  std::vector<Node*> nodes = API_activeGraph->getNodes();

  if (nodes.empty()) {
    cout << "nodes is empty!!!" << endl;
  }

  int ith = m_ui->spinBoxNodeToShow->value();

  //		cout << "removing node nb : " << ith << endl;
  //		cout << "graph size nb : " << nodes.size() << endl;
  if ((ith >= 0) && (((int)nodes.size()) > ith)) {
    return nodes[ith];
  } else {
    cout << "out of bounds" << endl;
    return NULL;
  }
}

void MotionPlanner::removeNode() {
  if (API_activeGraph == NULL) {
    return;
  }

  try {
    Node* node = getIthNodeInActiveGraph();

    if (node != NULL) {
      API_activeGraph->removeNode(node);
    } else {
      confPtr_t q_init = API_activeGraph->getRobot()->getInitPos();
      API_activeGraph->getRobot()->setAndUpdate(*q_init);
      cout << "Exede the number of nodes" << endl;
    }
  } catch (std::string str) {
    std::cerr << "Exeption in MotionPlanner::removeNode()" << endl;
    std::cerr << str << endl;
  }

  m_mainWindow->drawAllWinActive();
}

void MotionPlanner::on_pushButtonSetGoal_toggled(bool checked) {
  if (checked) {
    m_mh = new MovingHuman(0, 0, 0);
    m_mh->show();
    m_mh->setMainWindow(m_mainWindow);
    m_ui->pushButtonComputeNavigation->setEnabled(true);
    ENV.setBool(Env::drawTraj, true);
  } else {
    m_mh->close();
    delete m_mh;
    m_ui->pushButtonComputeNavigation->setEnabled(false);
    ENV.setBool(Env::drawTraj, false);
  }
}

void MotionPlanner::on_pushButtonComputeNavigation_clicked() {
  if (m_mh != NULL) {
    Robot* r = new Robot(p3d_get_robot_by_name(global_ActiveRobotName.c_str()));
    cout << r->getName() << endl;
    HRICS::Navigation* n = new HRICS::Navigation(r);
    std::vector<double> goal;
    goal.push_back(m_mh->getX());
    goal.push_back(m_mh->getY());
    goal.push_back(m_mh->getRZ());
    std::vector<std::vector<double> > path;
    n->getSimplePath(goal, path);
    cout << "printing the path :" << endl;
    for (unsigned int i = 0; i < path.size(); i++) {
      cout << "(x,y,rz) = (" << path[i][0] << "," << path[i][1] << ","
           << path[i][2] << ")" << endl;
    }
    m_mainWindow->drawAllWinActive();
  }
}

void MotionPlanner::on_checkBoxSmooth_toggled(bool checked) {
  PlanEnv->setBool(PlanParam::env_createTrajs, checked);
}

//------------------------------------------------------------------------
//------------------------------------------------------------------------
//------------------------------------------------------------------------

void MotionPlanner::initAStar() {
  connect(m_ui->pushButtonPlanAStar,
          SIGNAL(clicked()),
          this,
          SLOT(planAStarPath()));
  new SpinBoxConnector(
      this, m_ui->doubleSpinBoxPace, PlanEnv->getObject(PlanParam::grid_pace));
}

void MotionPlanner::planAStarPath() { emit selectedPlanner("AStarPlanning"); }

//------------------------------------------------------------------------
//------------------------------------------------------------------------
//------------------------------------------------------------------------

void MotionPlanner::initGraphSampling() {
  connect(m_ui->pushButtonSampleGraph,
          SIGNAL(clicked()),
          this,
          SLOT(sampleGraph()));
  connect(m_ui->pushButtonMakeGridGraph,
          SIGNAL(clicked()),
          this,
          SLOT(makeGridGraph()));
  new SpinBoxSliderConnector(
      this,
      m_ui->doubleSpinBoxSampleParamA,
      m_ui->horizontalSliderSampleParamA,
      PlanEnv->getObject(PlanParam::samplegraphVarianceA));
  new SpinBoxSliderConnector(
      this,
      m_ui->doubleSpinBoxSampleParamB,
      m_ui->horizontalSliderSampleParamB,
      PlanEnv->getObject(PlanParam::samplegraphVarianceB));
  new connectCheckBoxToEnv(m_ui->checkBoxSampleGraphMultipLoop,
                           PlanEnv->getObject(PlanParam::samplegraphMultiLoop));
}

void MotionPlanner::sampleGraph() { emit selectedPlanner("SampleGraph"); }

void MotionPlanner::makeGridGraph() { emit selectedPlanner("MakeGridGraph"); }
