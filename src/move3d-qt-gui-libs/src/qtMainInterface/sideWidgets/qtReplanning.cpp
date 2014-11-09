/*
 *  qtReplanning.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "qtReplanning.hpp"
#include "ui_qtReplanning.h"

#include "planner_handler.hpp"

#include "planner/TrajectoryOptim/trajectoryOptim.hpp"
#include "planner/TrajectoryOptim/Stomp/policy_improvement_loop.hpp"
#include "planner/planEnvironment.hpp"
#include "planner/replanningAlgorithms.hpp"
#include "planner/replanningSimulators.hpp"

#include "qtBase/SpinBoxSliderConnector_p.hpp"

#if defined(USE_QWT)
#include "qtPlot/basicPlot.hpp"
#include "qtPlot/multiPlot.hpp"
#include "qtPlot/replottingVectors.hpp"
#endif

using namespace std;
using namespace Eigen;
using namespace QtShiva;
using namespace Move3D;

MOVE3D_USING_SHARED_PTR_NAMESPACE

ReplanningWidget::ReplanningWidget(QWidget *parent) :
    QWidget(parent),
    m_ui(new Ui::ReplanningWidget)
{
    m_ui->setupUi(this);

    init();
}

ReplanningWidget::~ReplanningWidget()
{
    delete m_ui;
}

void ReplanningWidget::init()
{
    // This function enables multi-threading
    connect(this, SIGNAL(selectedPlanner(QString)), global_plannerHandler, SLOT(startPlanner(QString)));

    connect(m_ui->pushButtonRunStomp, SIGNAL(clicked()), this, SLOT(runStomp()));
    connect(m_ui->pushButtonRunChomp, SIGNAL(clicked()), this, SLOT(runChomp()));
    connect(m_ui->pushButtonRunNoReset, SIGNAL(clicked()), this, SLOT(runNoReset()));

    connect(m_ui->pushButtonExecuteSimpleSim, SIGNAL(clicked()), this, SLOT(executeSimpleSimu()));

    // Enable multi-thread graphical mode
    connect(m_ui->checkBoxMultiThreadGraphical, SIGNAL(toggled(bool)), this, SLOT(multiThreadGraphicalMode(bool)));

    // Active Joints
    connect(m_ui->radioButtonNavigation,    SIGNAL(toggled(bool)), this, SLOT(setActiveJoints()));
    connect(m_ui->radioButtonManipulation,  SIGNAL(toggled(bool)), this, SLOT(setActiveJoints()));
    connect(m_ui->radioButtonMobileManip,   SIGNAL(toggled(bool)), this, SLOT(setActiveJoints()));
    connect( ENV.getObject(Env::setOfActiveJoints), SIGNAL(valueChanged(int)), this, SLOT(setActiveJointsRadioButtons(int)), Qt::DirectConnection );

    // Init Method
    connect(m_ui->radioButtonRRT,             SIGNAL(toggled(bool)), this, SLOT(setInitMethod()));
    connect(m_ui->radioButtonStraightLine,    SIGNAL(toggled(bool)), this, SLOT(setInitMethod()));
    connect( PlanEnv->getObject(PlanParam::replanningInitMethod), SIGNAL(valueChanged(int)), this, SLOT(setInitMethodRadioButtons(int)), Qt::DirectConnection );

    // replanner Type
    connect(m_ui->comboBoxReplanner, SIGNAL(currentIndexChanged(int)), PlanEnv->getObject(PlanParam::replanningAlgorithm),SLOT(set(int)));
    connect( PlanEnv->getObject(PlanParam::replanningAlgorithm), SIGNAL(valueChanged(int)) ,m_ui->comboBoxReplanner, SLOT(setCurrentIndex(int)));
    m_ui->comboBoxReplanner->setCurrentIndex( 0 /*Simple*/ );
    // 0 => Simple
    // 1 => SoftMotion
    // 2 => RRT

    // Plot Noisy trajectories
#ifdef USE_QWT
    connect(m_ui->pushButtonPlotNoise, SIGNAL(clicked()), this, SLOT(plotNoisyTrajectories()));
    connect(m_ui->pushButtonPlotTraj, SIGNAL(clicked()), this, SLOT(plotMultiVectors()));
    connect(m_ui->pushButtonPlotConvergence, SIGNAL(clicked()), this, SLOT(plotConvergence()));
    m_plot = new BasicPlotWindow();
#endif

    connect(m_ui->pushButtonComputeSM, SIGNAL(clicked()), this, SLOT(computeSoftMotion()));
    connect(m_ui->pushButtonFixJoints, SIGNAL(clicked()), this, SLOT(setMlpCntrtsAndFixJoints()));

    // Smooth And Obstacle Weigth
    new SpinBoxConnector( this, m_ui->doubleSpinBoxSmoothWeight, PlanEnv->getObject(PlanParam::trajOptimSmoothWeight) );
    new SpinBoxConnector( this, m_ui->doubleSpinBoxObstacWeight, PlanEnv->getObject(PlanParam::trajOptimObstacWeight) );
    new SpinBoxConnector( this, m_ui->doubleSpinBoxGeneraWeight, PlanEnv->getObject(PlanParam::trajOptimGlobalWeight) );
    new SpinBoxConnector( this, m_ui->doubleSpinBoxObstacFactor, PlanEnv->getObject(PlanParam::trajOptimObstacFactor) );
    new SpinBoxConnector( this, m_ui->doubleSpinBoxSmoothFactor, PlanEnv->getObject(PlanParam::trajOptimSmoothFactor) );

    //---------------------------------------
    // Test the multi gaussian
    new connectCheckBoxToEnv( m_ui->checkBoxActiveJointsSetAtStart, ENV.getObject(Env::setActiveJointsGroup) );
    new connectCheckBoxToEnv( m_ui->checkBoxInitStomp,              ENV.getObject(Env::setStompPlanner) );

    new connectCheckBoxToEnv( m_ui->checkBoxTestMultiGauss, PlanEnv->getObject(PlanParam::trajOptimTestMultiGauss) );
    new connectCheckBoxToEnv( m_ui->checkBoxDrawTraj, ENV.getObject(Env::drawTraj) );
    new connectCheckBoxToEnv( m_ui->checkBoxOptimizeCurrentTraj, PlanEnv->getObject(PlanParam::withCurrentTraj) );
    new connectCheckBoxToEnv( m_ui->checkBoxDoReplanning, PlanEnv->getObject(PlanParam::doReplanning) );
    new connectCheckBoxToEnv( m_ui->checkBoxMoveHuman, PlanEnv->getObject(PlanParam::trajMoveHuman) );
    new connectCheckBoxToEnv( m_ui->checkBoxRecomputeOtp, PlanEnv->getObject(PlanParam::trajUseOtp) );
    new connectCheckBoxToEnv( m_ui->checkBoxSelectedDuration, PlanEnv->getObject(PlanParam::useSelectedDuration) );
    new connectCheckBoxToEnv( m_ui->checkBoxTimeLimit, PlanEnv->getObject(PlanParam::trajStompWithTimeLimit) );
    new connectCheckBoxToEnv( m_ui->checkBoxMMatrix, PlanEnv->getObject(PlanParam::trajStompMultiplyM) );
    new connectCheckBoxToEnv( m_ui->checkBoxWithRRT, PlanEnv->getObject(PlanParam::trajStompWithRRT) );
    new connectCheckBoxToEnv( m_ui->checkBoxCovAdaptation, PlanEnv->getObject(PlanParam::trajStompMatrixAdaptation) );
    new connectCheckBoxToEnv( m_ui->checkBoxStompMaxIteration, PlanEnv->getObject(PlanParam::trajStompWithIterLimit) );
    new connectCheckBoxToEnv( m_ui->checkBoxDrawParallel, PlanEnv->getObject(PlanParam::drawParallelTraj) );
//    new connectCheckBoxToEnv( m_ui->checkBoxDrawInCollision, PlanEnv->getObject(PlanParam::drawScaleFactorNodeSphere) );
    new connectCheckBoxToEnv( m_ui->checkBoxStompNoPrint, PlanEnv->getObject(PlanParam::trajStompNoPrint) );
    new connectCheckBoxToEnv( m_ui->checkBoxMultipleStomp, PlanEnv->getObject(PlanParam::trajStompRunMultiple) );
    new connectCheckBoxToEnv( m_ui->checkBoxStompParallel, PlanEnv->getObject(PlanParam::trajStompRunParallel) );
    new connectCheckBoxToEnv( m_ui->checkBoxStompDrawInCollision, PlanEnv->getObject(PlanParam::trajStompDrawImprovement) );
    new connectCheckBoxToEnv( m_ui->checkBoxMoveEndConfig, PlanEnv->getObject(PlanParam::trajStompMoveEndConfig) );
    new connectCheckBoxToEnv( m_ui->checkBoxStopWhenCollisionFree, PlanEnv->getObject(PlanParam::trajOptimStopWhenCollisionFree) );

    // Stomp max iteration
    new SpinBoxConnector(this,m_ui->spinBoxStompMaxIteration,PlanEnv->getObject(PlanParam::stompMaxIteration));

    // Stomp draw iteration
    new SpinBoxConnector(this,m_ui->spinBoxStompDrawIteration,PlanEnv->getObject(PlanParam::stompDrawIteration));

    // Time Limit
    new SpinBoxConnector(this,m_ui->doubleSpinBoxTimeLimit,PlanEnv->getObject(PlanParam::trajStompTimeLimit));

    // Set the number of point to be optimized
    new SpinBoxConnector(this,m_ui->spinBoxNbPoints,PlanEnv->getObject(PlanParam::nb_pointsOnTraj));

    // Set the duration of the optimized trajectory
    new SpinBoxConnector(this,m_ui->doubleSpinBoxDuration,PlanEnv->getObject(PlanParam::trajDuration));

    // Set the standard deviation of the perturbations
    new SpinBoxConnector(this,m_ui->doubleSpinBoxStdDev,PlanEnv->getObject(PlanParam::trajOptimStdDev));

    // The replanning window
    new SpinBoxSliderConnector( this, m_ui->doubleSpinBoxReplanningWindow, m_ui->horizontalSliderReplanningWindow , PlanEnv->getObject(PlanParam::trajReplanningWindow ));
    new SpinBoxSliderConnector( this, m_ui->doubleSpinBoxTotalTrajDuration, m_ui->horizontalSliderTotalTrajDuration , PlanEnv->getObject(PlanParam::trajReplanningTotalTime ));
}

//---------------------------------------------------------
// Trajectory Optimization
//---------------------------------------------------------
void ReplanningWidget::computeHandOver()
{
    emit(selectedPlanner(QString("computeHandover")));
}

void ReplanningWidget::runStomp()
{
    emit(selectedPlanner(QString("runStomp")));
}

void ReplanningWidget::runChomp()
{
    emit(selectedPlanner(QString("runChomp")));
}

void ReplanningWidget::runNoReset()
{
    emit(selectedPlanner(QString("runNoReset")));
}

void ReplanningWidget::plotNoisyTrajectories()
{  
#if defined(USE_QWT)
    ReplottingVectors* myPlot = new ReplottingVectors(m_plot);
    myPlot->setGeometry(m_plot->getPlot()->geometry());

    //  vector<double> cost;

    //  for( int i=0;i<int(global_noiseTrajectory.size()); i++)
    //	{
    //    cout << "global_noiseTrajectory[" << i << "] = " << global_noiseTrajectory[i] << endl;
    //  }

    vector< const vector<double>* > toPlot;
    toPlot.push_back( &global_noiseTrajectory1 );
    toPlot.push_back( &global_noiseTrajectory2 );

    myPlot->addData( toPlot );

    delete m_plot->getPlot();
    m_plot->setPlot(myPlot);
    m_plot->show();
#endif
}

void ReplanningWidget::plotMultiVectors()
{
#if defined(USE_QWT)
    MultiPlot* myPlot = new MultiPlot(m_plot);
    myPlot->setGeometry(m_plot->getPlot()->geometry());

    vector< string > plotNames;
    plotNames.push_back( "X SoftMotion" );
    plotNames.push_back( "Y SoftMotion" );
    plotNames.push_back( "X Initial" );
    plotNames.push_back( "Y Initial" );

    myPlot->setData( plotNames , traj_optim_to_plot );
    myPlot->setTitle("Initial and smoothed trajectory");

    delete m_plot->getPlot();
    m_plot->setPlot(myPlot);
    m_plot->show();
#endif
}

// static bool test_plot = true;
void ReplanningWidget::plotConvergence()
{
#if defined(USE_QWT)
    if (traj_optim_convergence.size() != 0 && traj_optim_convergence[0].size() != 0)
    {
        MultiPlot* myPlot = new MultiPlot(m_plot);
        myPlot->setGeometry(m_plot->getPlot()->geometry());

        vector<string> plotNames;
        plotNames.push_back( "Costs" );

        //    vector< vector<double> > to_plot = traj_optim_convergence;
        //
        //    if ( test_plot )
        //    {
        //      to_plot[0].resize(100);
        //      test_plot = false;
        //    }
        //    else {
        //      test_plot = true;
        //    }

        myPlot->setData( plotNames , traj_optim_convergence );
        myPlot->setTitle("Convergence of the traj optim");
        myPlot->setAxisScale( QwtPlot::yLeft, 0, 8000 );

        delete m_plot->getPlot();
        m_plot->setPlot(myPlot);
        m_plot->show();
    }
#endif
}

void ReplanningWidget::computeSoftMotion()
{
    emit(selectedPlanner(QString("convertToSoftMotion")));
    return;
}
//---------------------------------------------------------
// Replanning
//---------------------------------------------------------
void ReplanningWidget::setActiveJoints()
{
    if( m_ui->radioButtonNavigation->isChecked() )
    {
        cout << "radioButtonNavigation->isChecked()" << endl;
        ENV.setInt(Env::setOfActiveJoints,0);
    }
    if ( m_ui->radioButtonManipulation->isChecked() )
    {
        cout << "radioButtonManipulation->isChecked()" << endl;
        ENV.setInt(Env::setOfActiveJoints,1);
    }
    if ( m_ui->radioButtonMobileManip->isChecked() )
    {
        cout << "radioButtonMobileManip->isChecked()" << endl;
        ENV.setInt(Env::setOfActiveJoints,2);
    }
}

void ReplanningWidget::setActiveJointsRadioButtons(int type)
{
    if( type == 0 && !m_ui->radioButtonNavigation->isChecked() )
    {
        cout << "radioButtonNavigation->toggle()" << endl;
        m_ui->radioButtonNavigation->toggle();
    }
    if( type == 1 && !m_ui->radioButtonManipulation->isChecked())
    {
        cout << "radioButtonManipulation->toggle()" << endl;
        m_ui->radioButtonManipulation->toggle();
    }
    if( type == 2 && !m_ui->radioButtonMobileManip->isChecked())
    {
        cout << "radioButtonMobileManip->toggle()" << endl;
        m_ui->radioButtonMobileManip->toggle();
    }
}

void ReplanningWidget::setInitMethod()
{
    if( m_ui->radioButtonStraightLine->isChecked() )
    {
        PlanEnv->setInt(PlanParam::replanningInitMethod,0);
        cout << "radioButtonStraightLine->isChecked()" << endl;
    }
    if ( m_ui->radioButtonRRT->isChecked() )
    {
        PlanEnv->setInt(PlanParam::replanningInitMethod,1);
        cout << "radioButtonRRT->isChecked()" << endl;
    }
}

void ReplanningWidget::setInitMethodRadioButtons(int type)
{
    if( type == 0 && !m_ui->radioButtonStraightLine->isChecked() )
    {
        cout << "radioButtonNavigation->toggle()" << endl;
        m_ui->radioButtonStraightLine->toggle();
    }
    if( type == 1 && !m_ui->radioButtonRRT->isChecked())
    {
        cout << "radioButtonManipulation->toggle()" << endl;
        m_ui->radioButtonRRT->toggle();
    }
}

void ReplanningWidget::multiThreadGraphicalMode(bool enable)
{
    Move3D::global_rePlanningEnv->set_multithread_graphical( enable );
}

void ReplanningWidget::setMlpCntrtsAndFixJoints()
{
    emit(selectedPlanner(QString("initMlpCntrtsAndFixJoints")));
    return;
}

void ReplanningWidget::executeSimpleSimu()
{
    emit(selectedPlanner(QString("ExecuteSimpleSimu")));
}

