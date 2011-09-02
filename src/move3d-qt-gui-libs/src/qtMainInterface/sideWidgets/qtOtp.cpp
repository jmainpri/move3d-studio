/*
 *  qtCost.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "qtOtp.hpp"
#include "ui_qtOtp.h"


#include "HRI_costspace/HRICS_costspace.hpp"

#include "qtMainInterface/mainwindow.hpp"
#include "qtMainInterface/mainwindowGenerated.hpp"
#include "planner_handler.hpp"
#include "planner/planEnvironment.hpp"

#include <QMessageBox>
#include <QDateTime>

using namespace std;
using namespace tr1;


OtpWidget::OtpWidget(QWidget *parent) :
QWidget(parent),
m_ui(new Ui::OtpWidget)
{
    m_ui->setupUi(this);
    initOTP();
}

OtpWidget::~OtpWidget()
{
	delete m_ui;
}

void OtpWidget::initOTP()
{
        connect(this, SIGNAL(selectedPlanner(QString)),global_plannerHandler, SLOT(startPlanner(QString)));

        m_k_distance     = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxDistance,	m_ui->horizontalSliderDistance,		Env::Kdistance);
        m_k_visbility    = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxVisibility,	m_ui->horizontalSliderVisibility,	Env::Kvisibility );
        m_k_naturality   = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxNatural,	m_ui->horizontalSliderNatural ,		Env::Knatural );
        m_k_reachability = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxReachable,	m_ui->horizontalSliderReachable ,	Env::Kreachable );

        m_k_OptimalDist = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxHumanDistance,	m_ui->horizontalSliderHumanDistance,	Env::optimalDist);
        m_k_RobotDistMax= new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxRobotDistance,	m_ui->horizontalSliderRobotDistance,	Env::robotMaximalDist);
        m_k_GazeAngle   = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxAngle,           m_ui->horizontalSliderAngle,            Env::gazeAngle);
        m_k_LimitRot    = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxLimitRot,           m_ui->horizontalSliderLimitRot,         PlanParam::env_limitRot);

        m_k_OptimalDistFactor  = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxHumanDistFactor,	m_ui->horizontalSliderHumanDistFactor,	Env::optimalDistFactor);
        m_k_RobotDistMaxFactor = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxDistFactor,		m_ui->horizontalSliderDistFactor,		Env::robotMaximalDistFactor);
        m_k_GazeAngleFactor    = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxFieldFactor,		m_ui->horizontalSliderFieldFactor,		Env::gazeAngleFactor);


        m_k_Obj =  new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxObj, m_ui->horizontalSliderObj, PlanParam::env_objectNessecity);
        m_k_Obj->setValue(0.5);

        m_k_MaxIter           = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxMaxIter,		    m_ui->horizontalSliderMaxIter,      PlanParam::env_maxIter);
        m_k_TotMaxIter        = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxTotMaxIter,		m_ui->horizontalSliderTotMaxIter,	PlanParam::env_totMaxIter);
        m_k_NbRandomRotOnly   = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxNbRandomRot,       m_ui->horizontalSliderNbRandomRot,	PlanParam::env_nbRandomRotOnly);
        m_k_NbSittingRotation = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxNbSittingRotation, m_ui->horizontalSliderNbSittingRotation,
                                                                    PlanParam::env_nbSittingRotation);

        m_k_XMin = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxXMin,	m_ui->horizontalSliderXMin,		PlanParam::env_randomXMinLimit);
        m_k_XMax = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxXMax,	m_ui->horizontalSliderXMax,		PlanParam::env_randomXMaxLimit);
        m_k_YMin = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxYMin,	m_ui->horizontalSliderYMin,		PlanParam::env_randomYMinLimit);
        m_k_YMax = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxYMax,	m_ui->horizontalSliderYMax,		PlanParam::env_randomYMaxLimit);

        m_k_RobotSpeed = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxRobotSpeed,m_ui->horizontalSliderRobotSpeed,	PlanParam::env_robotSpeed);
        m_k_HumanSpeed = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxHumanSpeed,m_ui->horizontalSliderHumanSpeed,	PlanParam::env_humanSpeed);
        m_k_TimeStamp = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxTimeStamp,	m_ui->horizontalSliderTimeStamp,	PlanParam::env_timeStamp);

        m_k_Psi = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxPsi,		m_ui->horizontalSliderPsi,		PlanParam::env_psi);
        m_k_delta = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxDelta,	m_ui->horizontalSliderDelta,	PlanParam::env_delta);
        m_k_ksi = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxKsi,		m_ui->horizontalSliderKsi,		PlanParam::env_ksi);
        m_k_rho = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxRho,		m_ui->horizontalSliderRho,		PlanParam::env_rho);

        m_k_sittingOffset = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxSittingOffset,	m_ui->horizontalSliderSittingOffset,PlanParam::env_sittingOffset);
        m_k_nbComputeOTP  = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxMOTP,          m_ui->horizontalSliderMOTP,         PlanParam::env_MOTP);
        m_k_cellSize  =     new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxCellsize,      m_ui->horizontalSliderCellsize,     PlanParam::env_Cellsize);

        m_k_sleep = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxSleep,	m_ui->horizontalSliderSleep,PlanParam::env_timeShow);
        m_k_sleep = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxPow,	m_ui->horizontalSliderPow  ,PlanParam::env_pow);

        m_k_RobotPos = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxPR2Position,		m_ui->horizontalSliderPR2Position);
        m_k_RobotPos->setValue(1.5);
        connect(m_k_RobotPos,SIGNAL(valueChanged(double)),this,SLOT(on_RobotPosValuechanged(double)));


        connect( m_ui->pushButtonComputeTheOtp, SIGNAL( clicked()),this, SLOT(computeTheOtp()));
        connect(m_ui->pushButton_TimerUse, SIGNAL(toggled(bool)),this, SLOT(timerOrNot(bool)) );
        connect(m_ui->pushButtonComputeTheOtp, SIGNAL(toggled(bool)),this, SLOT(launchOrStopTimer(bool)) );

        timer = new QTimer();
        timer->setInterval(300);
        connect(timer,SIGNAL(timeout()),this,SLOT(computeTheOtp()));

        timerTraj = new QTimer(this);
        timerTraj->setInterval(300);
        connect(timerTraj,SIGNAL(timeout()),this,SLOT(moveRobots()));

        m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxReach,Env::drawGrid);
        m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxrandomLimits,PlanParam::drawRandomMap);
        m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawSlice,PlanParam::env_drawSlice);
        m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawPoint,PlanParam::env_drawRandomPoint);
        m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxHumanTraj,PlanParam::env_showHumanTraj);
        m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxRobotBaseCostGrid,Env::DrawRobotBaseGridCosts);

        m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawOnlyBest,PlanParam::env_drawOnlyBest);
        m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxIsStanding,PlanParam::env_isStanding);

        m_ui->groupBoxGridComputing->hide();
        m_ui->spinBoxOTPth->setDisabled(true);

}

void OtpWidget::launchOrStopTimer(bool isLaunched)
{

        if (isLaunched)
        {
                timer->start();
        }
        else
        {
             if (timer->isActive())
            {
                timer->stop();
            }
        }
}

void OtpWidget::initSliders()
{
        m_ui->otp_widget->setDisabled(false);
        m_ui->pushButtonComputeTheOtp->setDisabled(false);
        m_ui->sliders_widget->setDisabled(false);

        connect(m_ui->checkBoxReach,SIGNAL(clicked()),m_mainWindow,SLOT(drawAllWinActive()));
        connect(m_ui->checkBoxRobotBaseCostGrid,SIGNAL(clicked()),m_mainWindow,SLOT(drawAllWinActive()));
        connect(m_ui->pushButton_ComputeCosts,SIGNAL(clicked()),m_mainWindow,SLOT(drawAllWinActive()));


        on_pushButton_RobotBaseGrid_clicked();

}

void OtpWidget::computeTheOtp()
{
  Eigen::Vector3d WSPoint;
        int type = -1;
        // this variable allow to choose between 3 fonctions :
        if (m_ui->radioButtonBest->isChecked())
                // either you don't take into account the environment
        {
                type = 0;
        }
        else if (m_ui->radioButtonBestFeasable->isChecked())
                // or you take into account only obstacles for human posture
        {
                type = 1;
        }
        else if (m_ui->radioButtonChoose->isChecked())
                // or you take into account the whole envirenment for both human and rbot to compute the transfert point
        {
                type = 2;
        }
        if (type > -1 && type < 3)
        {
                dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->ComputeTheObjectTransfertPoint(
                                    m_ui->pushButton_Move->isChecked(),type,WSPoint, m_ui->spinBoxThreshold->value());
                cout << "WSPoint = " << WSPoint << endl;
                m_mainWindow->drawAllWinActive();
        }
}


void OtpWidget::drawAllWinActive()
{
	m_mainWindow->drawAllWinActive();	
}

void OtpWidget::timerOrNot(bool isChecked)
{
        m_ui->pushButtonComputeTheOtp->setCheckable(isChecked);
        m_ui->pushButtonComputeTheOtp->setChecked(false);
        launchOrStopTimer(false);
}

void OtpWidget::on_pushButton_clicked()
{
        Robot* Object = global_Project->getActiveScene()->getRobotByNameContaining("HUMAN");
        if (Object != NULL)
        {
//                        qDebug() << "load initial configuration" ;
                Object->setAndUpdate(*Object->getInitialPosition());
        }
        m_mainWindow->drawAllWinActive();


//        Eigen::Vector3d WSPoint;
//        dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->chooseBestTransferPoint(WSPoint,false, 0);

}

void OtpWidget::on_pushButtonFastComputing_toggled(bool checked)
{
	ENV.setBool(Env::FastComputingRobotBase,checked);
}

void OtpWidget::on_pushButton_RobotBaseGrid_clicked()
{
	HRICS::Natural* reachSpace = HRICS_MotionPL->getReachability();
	vector<double> box(4);

	box[0]  = -2.5; box[1]  = 2.5;
	box[2]  = -2.5; box[3]  = 2.5;
	reachSpace->initRobotBaseGrid(box);

	m_mainWindow->drawAllWinActive();
	m_ui->pushButton_ComputeCosts->setDisabled(false);

//	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
//	{
//		dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->initHumanCenteredGrid(0.1);
//	}
}

void OtpWidget::on_pushButton_ComputeCosts_clicked()
{
    HRICS::Natural* reachSpace = HRICS_MotionPL->getReachability();
    reachSpace->getRobotBaseGrid()->recomputeAllCosts();
}

void OtpWidget::on_pushButton_ShowOptions_toggled(bool checked)
{
	if (checked)
	{
		m_ui->groupBoxGridComputing->show();
		m_ui->pushButton_ShowOptions->setText("Hide Options");
	}
	else
	{
		m_ui->groupBoxGridComputing->hide();
		m_ui->pushButton_ShowOptions->setText("Show Options");
	}
}

void OtpWidget::on_radioButtonAll_toggled(bool checked)
{
    m_ui->groupBoxWeightedSum->setDisabled(!checked);
    if (checked)
    {
        ENV.setInt(Env::typeRobotBaseGrid,0);
    }

}

void OtpWidget::on_radioButtonHumanDist_toggled(bool checked)
{
    if (checked)
    {
        ENV.setInt(Env::typeRobotBaseGrid,1);
    }
}

void OtpWidget::on_radioButtonRobotDist_toggled(bool checked)
{
    if (checked)
    {
        ENV.setInt(Env::typeRobotBaseGrid,2);
    }
}

void OtpWidget::on_radioButtonFieldOfVision_toggled(bool checked)
{
    if (checked)
    {
        ENV.setInt(Env::typeRobotBaseGrid,3);
    }
}

void OtpWidget::on_pushButtonPR2RestPose_clicked()
{
    if (dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getActivRobot()->getName().find("PR2") != string::npos)
    {
        dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->initPR2RepoConf();
        m_mainWindow->drawAllWinActive();
    }
}

void OtpWidget::on_pushButtonInitHumanPR2_clicked()
{
    if (dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getActivRobot()->getName().find("PR2") != string::npos)
    {
        dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->initPR2AndHumanTest();
        m_k_RobotPos->setValue(1.5);
        m_ui->widgetPR2Position->setDisabled(false);
        m_mainWindow->drawAllWinActive();
    }
}

void OtpWidget::on_pushButtonComputeGIK_clicked()
{
        cout << "Boutton don't work anymore!!!" << endl;
       // emit(selectedPlanner(QString("otp")));
}

void OtpWidget::on_pushButtonGiveConf_clicked()
{
    if (dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getActivRobot()->getName().find("PR2") != string::npos)
    {
        dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->initPR2GiveConf();
        m_mainWindow->drawAllWinActive();
    }
}

void OtpWidget::on_RobotPosValuechanged(double value)
{
    if (dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getActivRobot()->getName().find("PR2") != string::npos)
    {
        dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->ChangeRobotPos(value);
        m_mainWindow->drawAllWinActive();
    }
}

void OtpWidget::on_spinBoxThreshold_valueChanged(int value)
{
//    computeTheOtp();
	on_pushButton_2_clicked();
}

void OtpWidget::on_pushButtonCreateGrid_clicked()
{
	HRICS_MotionPLConfig  = new HRICS::OTPMotionPl;
	HRICS_activeDist = HRICS_MotionPL->getDistance();

	ENV.setBool(Env::HRIPlannerCS,true);
	ENV.setBool(Env::enableHri,true);
	ENV.setBool(Env::isCostSpace,true);

	ENV.setBool(Env::useBallDist,false);
	ENV.setBool(Env::useBoxDist,true);

	if(ENV.getBool(Env::HRIPlannerCS))
	{
		API_activeGrid = dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getPlanGrid();

		ENV.setBool(Env::drawGrid,true);
		m_mainWindow->setBoolFloor(false);
		m_mainWindow->drawAllWinActive();
	}

	m_ui->spinBoxXCell->setMaximum(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getPlanGrid()->getNbCellX()-1);
	m_ui->spinBoxYCell->setMaximum(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getPlanGrid()->getNbCellY()-1);

	dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->changeHumanByName("HERAKLES_HUMAN1");
}

void OtpWidget::on_pushButtonComputeAStar_clicked()
{
	cout << "Computing 2D A* for EnvGrid" << endl;
	if(ENV.getBool(Env::HRIPlannerCS))
	{
		dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->computeAStarIn2DGrid();
		ENV.setBool(Env::drawOTPTraj,true);
		m_mainWindow->drawAllWinActive();
	}
}

void OtpWidget::on_pushButtonRecomputeAStarGridCosts_clicked()
{
	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
		dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getPlanGrid()->setCellsToblankCost();
	}
	m_mainWindow->drawAllWinActive();
}

void OtpWidget::on_pushButtonNewComputeOTP_clicked()
{
	if (!dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig) )
	{
		on_pushButtonCreateGrid_clicked();
	}
	if (!dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getReachability())
	{
		dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->setReachability(HRICS_activeNatu);
	}
	dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->computeObjectTransfertPoint();
	ENV.setBool(Env::drawOTPTraj,true);
	m_mainWindow->drawAllWinActive();

}

void OtpWidget::on_pushButtonShowTraj_clicked()
{
	dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->SetPathIndexNull();
	timerTraj->start(400);
}

void OtpWidget::moveRobots()
{
	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->moveToNextPos())
	{
		timerTraj->stop();
	}
	m_mainWindow->drawAllWinActive();
}

void OtpWidget::on_pushButtonMultipleOTP_clicked()
{
	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig) )
	{
		emit(selectedPlanner(QString("MultipleOtp")));
		ENV.setBool(Env::drawOTPTraj,true);
	}
}

void OtpWidget::on_pushButton_2_clicked()
{
	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig) )
	{
		emit(selectedPlanner(QString("otp")));
		ENV.setBool(Env::drawOTPTraj,true);
	}

//	dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
//	QTime timeB = QTime::currentTime();
//	dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->initGrid();
////	dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->simpleComputeBaseAndOTP();
//	dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getPlanGrid()->setCellsToblankCost();
//	dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->newComputeOTP(m_ui->checkBoxIsStanding->isChecked(), m_ui->doubleSpinBoxObj->value());
//	QTime timeE = QTime::currentTime();


//	int msec = timeE.msec() - timeB.msec();
//	int sec = timeB.secsTo(timeE);
//	cout << "time for computing the OTP is : " << sec << "s " << msec << "msec" << endl;

//	ENV.setBool(Env::drawOTPTraj,true);
//	m_ui->spinBoxOTPNav->setMaximum(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getNbConf() - 1);

//	m_mainWindow->drawAllWinActive();
}

void OtpWidget::on_pushButtonOTPth_clicked()
{
	if (!dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
		on_pushButtonCreateGrid_clicked();
	}
	if (!dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getReachability())
	{
		dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->setReachability(HRICS_activeNatu);
		m_ui->spinBoxOTPth->setMaximum(HRICS_activeNatu->getGrid()->getNumberOfCells());
	}
	dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->OTPonly(m_ui->spinBoxOTPth->value());
	m_ui->spinBoxOTPth->setEnabled(true);
	m_ui->checkBoxReach->setChecked(false);
//	API_activeGrid = dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getReachability()->getGrid();
	m_mainWindow->drawAllWinActive();

}

void OtpWidget::on_spinBoxOTPth_valueChanged(int value)
{
	dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->OTPonly(value);
	m_mainWindow->drawAllWinActive();
}

void OtpWidget::on_pushButtonAddOTP_clicked()
{
	dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->addToList();
	m_ui->spinBoxNextOTP->setMaximum(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getOTPList().size() - 1);
}

void OtpWidget::on_pushButtonRemoveLast_clicked()
{
	dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->removeLastFromOTPList();
	m_ui->spinBoxNextOTP->setMaximum(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getOTPList().size() - 1);
}

void OtpWidget::on_pushButtonShowList_clicked()
{
	dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->showOTPList();
}

void OtpWidget::on_pushButtonSaveToFile_clicked()
{
  vector<Eigen::Vector3d> OTPList = dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getOTPList();
    QString fileName = QFileDialog::getSaveFileName(this,
                                tr("Save otp list to file"));
    if (!fileName.isEmpty())
    {
        QFile file(fileName);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
            return;

        for (unsigned int i =0 ; i < OTPList.size(); i++)
        {
            QTextStream out(&file);
            out << OTPList.at(i)[0] << endl;
            out << OTPList.at(i)[1] << endl;
            out << OTPList.at(i)[2] << endl;
            out <<  "-----------------" << endl;
        }
    }



}

void OtpWidget::on_spinBoxNextOTP_valueChanged(int value)
{
    std::vector<Eigen::Vector3d> OTPList = dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getOTPList();
    if (value < (int) OTPList.size())
    {
        dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->setCurrentOTP(OTPList.at(value));
        dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->placeRobot();
    }
    m_mainWindow->drawAllWinActive();
}

void OtpWidget::on_pushButtonLoadFromFile_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(
                                this, tr("Open OTP list"));
    if (!fileName.isEmpty())
    {
        QFile file(fileName);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
             return;

        vector<Eigen::Vector3d> OTPList;
        dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->clearOTPList();

        QTextStream in(&file);
        Eigen::Vector3d WSPoint;
        while (!in.atEnd()) {
            WSPoint[0] = in.readLine().toDouble();
            WSPoint[1] = in.readLine().toDouble();
            WSPoint[2] = in.readLine().toDouble();
            if (!in.readLine().contains("-----"))
            {
                cout << "Error has occured while loading the file.";
                return;
            }
            dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->addToList(WSPoint);
            OTPList.push_back(WSPoint);

        }

        m_ui->spinBoxNextOTP->setMaximum(OTPList.size()-1);
    }
}

void OtpWidget::on_pushButtonPlaceRobot_clicked()
{
	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
		dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->placeRobot();
	}
	m_mainWindow->drawAllWinActive();
}


void OtpWidget::on_pushButtonDrawOTPs_toggled(bool checked)
{
	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
		dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->drawOTPList(checked);
	}
//	m_mainWindow->drawAllWinActive();
}

void OtpWidget::on_pushButtonPlaceHuman_clicked()
{
	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
		dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->placeHuman();
	}
	m_mainWindow->drawAllWinActive();
}

void OtpWidget::on_PushButtonSaveConfs_clicked()
{
	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
		shared_ptr<Configuration> q = dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getActivRobot()->getCurrentPos();
		shared_ptr<Configuration> q_human = dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getReachability()->getRobot()->getCurrentPos();
		q->print();

		if (confFileName.isEmpty())
		{
			confFileName = QFileDialog::getSaveFileName(this,
										tr("Save conf to file"));
		}
		if (!confFileName.isEmpty())
		{
			QFile file(confFileName);
			if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Append))
				return;

			for (int i =0 ; i < q->getRobot()->getRobotStruct()->njoints; i++)
			{
				QTextStream out(&file);
				if (i == 6){ out << q->at(i) - q_human->at(i) << " ";}
				else if (i == 7){ out << q->at(i) - q_human->at(i) << " ";}
				else if (i == 11){ out << q->at(i) << " ";}

				else if (i == 12){ out << q->at(i) << " ";}

				else if (i == 16){ out << q->at(i) << " ";}
				else if (i == 17){ out << q->at(i) << " ";}
				else if (i == 18){ out << q->at(i) << " ";}
				else if (i == 19){ out << q->at(i) << " ";}
				else if (i == 20){ out << q->at(i) << " ";}
				else if (i == 21){ out << q->at(i) << " ";}
				else if (i == 22){ out << q->at(i) << " ";}
			}
			QTextStream out(&file);
			out << "\n";

		}
	}
}

void OtpWidget::on_pushButtonAddToconfList_clicked()
{
	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
		dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->addConfToList();
		m_ui->spinBoxNavigate->setMaximum(m_ui->spinBoxNavigate->maximum() + 1);
	}
}

void OtpWidget::on_pushButtoRemoveLastconf_clicked()
{
	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
		dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->removeLastConf();
		m_ui->spinBoxNavigate->setMaximum(m_ui->spinBoxNavigate->maximum() - 1);
	}
}

void OtpWidget::on_spinBoxNavigate_valueChanged(int value)
{
	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
		dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->setRobotsToConf(value,m_ui->checkBoxIsStanding->isChecked());
		std::vector<HRICS::ConfigHR> v = dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getConfList();
		if ((unsigned int) value < v.size())
		{
			m_ui->lineEditConfCost->setText(QString::number(v.at(value).getCost()));
			m_ui->spinBoxNavigate->setMaximum(v.size() - 1);
		}
	}
	m_mainWindow->drawAllWinActive();
}

void OtpWidget::on_pushButtonSaveConfs_clicked()
{
	QString fileName = QFileDialog::getSaveFileName(this,
								tr("Save configuration to XML file"));
	if (!fileName.isEmpty() && dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
		dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveToXml(fileName.toStdString());
	}


}

void OtpWidget::on_pushButtonLoadConfs_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(
                                this, tr("Load configurations from XML"));

	if (!fileName.isEmpty() && dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
		m_ui->spinBoxNavigate->setMaximum(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadConfsFromXML(fileName.toStdString(),true,false) - 1);
	}
}

void OtpWidget::on_pushButtonComputeConfortCost_clicked()
{
	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
//		double cost = dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getReachability()->getConfigCost();
//		double cost = dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->testComputeConfigCost();

//		QString str("Cost of human configuration is : ");



		QString str("");
		if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->testCol(true,true))
		{
			str += QString("human collision with configuration collision checking : YES\n");
		}
		else
		{
			str += QString("human collision with configuration collision checking : NO\n");
		}
		if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->testCol(true,false))
		{
			str += QString("human collision with robot collision checking             : YES\n");
		}
		else
		{
			str += QString("human collision with robot collision checking             : NO\n");
		}
		if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->testCol(false,true))
		{
			str += QString("robot collision with configuration collision checking   : YES\n");
		}
		else
		{
			str += QString("robot collision with configuration collision checking   : NO\n");
		}
		if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->testCol(false,false))
		{
			str += QString("robot collision with robot collision checking              : YES\n");
		}
		else
		{
			str += QString("robot collision with robot collision checking              : NO\n");
		}

//				("config in collision");
//		str += QString::number(cost);
		QMessageBox::information(this, tr("Human config cost"), str);
		ENV.setBool(Env::drawOTPTraj,true);
	}
}


void OtpWidget::on_pushButtonInit_clicked()
{

	on_pushButtonCreateGrid_clicked();

	if (!dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getReachability())
	{
		dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->setReachability(HRICS_activeNatu);
	}

	QString home( getenv("HOME_MOVE3D") );
	QString fileNameStand = "/statFiles/OtpComputing/conf.xml";
	QString fileNameSit = "/statFiles/OtpComputing/confSit.xml";
	QString fileNameStandSlice = "/statFiles/OtpComputing/confTranche.xml";
	QString fileNameSitSlice = "/statFiles/OtpComputing/confSitTranche.xml";
	if (PlanEnv->getBool(PlanParam::env_useOldDude))
	{
		fileNameStand = "/statFiles/OtpComputing/confOldDude.xml";
		fileNameSit = "/statFiles/OtpComputing/confOldDude.xml";
		fileNameStandSlice = "/statFiles/OtpComputing/confOldDude.xml";
		fileNameSitSlice = "/statFiles/OtpComputing/confOldDude.xml";
	}

	fileNameStand = home + fileNameStand;
	fileNameSit = home + fileNameSit;
	fileNameStandSlice = home + fileNameStandSlice;
	fileNameSitSlice = home + fileNameSitSlice;

	if ( dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
		m_ui->spinBoxNavigate->setMaximum(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadConfsFromXML(fileNameStand.toStdString(),true, false) - 1);
		dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadConfsFromXML(fileNameSit.toStdString(),false, false);

		dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadConfsFromXML(fileNameStandSlice.toStdString(),true, true);
		dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadConfsFromXML(fileNameSitSlice.toStdString(),false, true);
	}
	dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->initGrid();
	m_ui->spinBoxOTPth->setEnabled(true);
	m_ui->checkBoxReach->setChecked(false);
	m_mainWindow->drawAllWinActive();
}

void OtpWidget::on_pushButtonConfCost_clicked()
{
	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
		m_ui->lineEditConfCost->setText(QString::number(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getReachability()->getConfigCost()));
	}
}

void OtpWidget::on_pushButtonSaveConf_clicked()
{
	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
		dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
	}
}

void OtpWidget::on_pushButtonLoadConf_clicked()
{
	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
		dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);
	}
	m_mainWindow->drawAllWinActive();
}

void OtpWidget::on_pushButtonShowOTP_clicked()
{
	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
		dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->showBestConf();
	}
	m_mainWindow->drawAllWinActive();
}

void OtpWidget::on_spinBoxOTPNav_valueChanged(int value)
{
	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
		double cost = dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->showConf(value);
		m_ui->lineEditConfigCost->setText(QString::number(cost));
	}
	m_mainWindow->drawAllWinActive();
}


void OtpWidget::on_radioButtonHumGrid_toggled(bool checked)
{
    PlanEnv->setBool(PlanParam::env_humanGridDraw,checked);
    m_mainWindow->drawAllWinActive();
}

void OtpWidget::on_radioButtonRobGrid_toggled(bool checked)
{
    PlanEnv->setBool(PlanParam::env_robotGridDraw,checked);
    m_mainWindow->drawAllWinActive();
}

void OtpWidget::on_radioButtonDistGrid_toggled(bool checked)
{
    PlanEnv->setBool(PlanParam::env_drawDistGrid,checked);
    m_mainWindow->drawAllWinActive();
}

void OtpWidget::on_pushButtonInitEnvGrid_clicked()
{
	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
		QTime timeB = QTime::currentTime();
		dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getPlanGrid()->initGrid();
		QTime timeE = QTime::currentTime();

		int msec = timeE.msec() - timeB.msec();
		int sec = timeB.secsTo(timeE);
		cout << "time for computing the grid is : " << sec << "s " << msec << "msec" << endl;
	}
	m_mainWindow->drawAllWinActive();
}

void OtpWidget::on_checkBox_clicked(bool checked)
{
	m_ui->widgetShowCell->setEnabled(checked);
	if (!checked)
	{
		PlanEnv->setInt(PlanParam::env_xToDraw,-1);
		PlanEnv->setInt(PlanParam::env_yToDraw,-1);
	}
}

void OtpWidget::on_spinBoxXCell_valueChanged(int value)
{
	PlanEnv->setInt(PlanParam::env_xToDraw,value);
	changeDistCost();
	m_mainWindow->drawAllWinActive();
}

void OtpWidget::on_spinBoxYCell_valueChanged(int value)
{
	PlanEnv->setInt(PlanParam::env_yToDraw,value);
	changeDistCost();
	m_mainWindow->drawAllWinActive();
}

void OtpWidget::changeDistCost()
{
	int x = PlanEnv->getInt(PlanParam::env_xToDraw);
	int y = PlanEnv->getInt(PlanParam::env_yToDraw);
	if (x > -1 && y > -1 && dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
		double dist = -1;
		if (PlanEnv->getBool(PlanParam::env_humanGridDraw) || PlanEnv->getBool(PlanParam::env_drawDistGrid))
		{
			dist = dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getDistFromCell(x,y,true);
		}
		else if (PlanEnv->getBool(PlanParam::env_robotGridDraw))
		{
			dist = dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getDistFromCell(x,y,false);

		}
		if (dist > -1)
		{
			m_ui->lineEditDist->setText(QString::number(dist));
		}
		else
		{
			m_ui->lineEditDist->setText(QString("NAN"));
		}
		double rot = dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getrotFromCell(x,y);
		if (rot < numeric_limits<double>::max())
		{ m_ui->lineEditRot->setText(QString::number(rot*180/M_PI) + " deg"); }
		else
		{ m_ui->lineEditDist->setText(QString("NAN")); }
	}
}


void OtpWidget::on_pushButtonnbConfs_clicked()
{
	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
		m_ui->spinBoxOTPNav->setMaximum(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getNbConf() - 1);
	}
}


void OtpWidget::on_pushButtonEraseTraj_clicked()
{
	ENV.setBool(Env::drawOTPTraj,false);
}

void OtpWidget::on_pushButtonDump_clicked()
{
	cout << "------------ OTP variable -----------" <<endl;
//	cout << " isStanding = " << PlanEnv->getBool(PlanParam::env_isStanding)?"true":"false" << endl;
	cout << " objectNecessity = " <<PlanEnv->getDouble(PlanParam::env_objectNessecity) << endl;
	cout << " maxIter = " << PlanEnv->getInt(PlanParam::env_maxIter)<< endl;
	cout << " totMaxIter = " <<PlanEnv->getInt(PlanParam::env_totMaxIter) << endl;
	cout << " robotSpeed = " << PlanEnv->getDouble(PlanParam::env_robotSpeed)<< endl;
	cout << " humanSpeed = " <<PlanEnv->getDouble(PlanParam::env_humanSpeed) << endl;
	cout << " timeStamp = " <<PlanEnv->getDouble(PlanParam::env_timeStamp) << endl;
	cout << " psi = " <<PlanEnv->getDouble(PlanParam::env_psi) << endl;
	cout << " delta = " <<PlanEnv->getDouble(PlanParam::env_delta) << endl;
	cout << " ksi = " <<PlanEnv->getDouble(PlanParam::env_ksi) << endl;
	cout << " rho = " <<PlanEnv->getDouble(PlanParam::env_rho) << endl;
	cout << " sittingOffset = " <<PlanEnv->getDouble(PlanParam::env_sittingOffset) << endl;

	cout << "------------ OTP variable end--------" <<endl;
}

void OtpWidget::on_pushButtonComputeHumanRobotDist_clicked()
{
	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
		std::pair<double,double> p = dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->computeHumanRobotDist();
		cout << "min dist = " << p.first << " max dist = " << p.second << endl;
	}
}


void OtpWidget::on_radioButtonNormalRandom_toggled(bool checked)
{
    PlanEnv->setBool(PlanParam::env_normalRand,checked);
}

void OtpWidget::on_radioButtonFusedGrid_toggled(bool checked)
{
    PlanEnv->setBool(PlanParam::env_fusedGridRand,checked);
}

void OtpWidget::on_pushButtonClearConfList_clicked()
{
	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
		//dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->clearConfList();
		m_ui->spinBoxNavigate->setMaximum(0);
	}
}

void OtpWidget::on_radioButtonUseSlice_toggled(bool checked)
{
	PlanEnv->setBool(PlanParam::env_useSlice,checked);
}

void OtpWidget::on_radioButtonOrientedSlice_toggled(bool checked)
{
	PlanEnv->setBool(PlanParam::env_useOrientedSlice,checked);
}



void OtpWidget::on_radioButtonAllGrid_toggled(bool checked)
{
	PlanEnv->setBool(PlanParam::env_useAllGrid,checked);
}

void OtpWidget::on_pushButtonDumpCosts_clicked()
{
	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
		dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpCosts();
	}
}

void OtpWidget::on_pushButtonStandUp_clicked()
{
	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
		dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->standUp();
	}
}

void OtpWidget::on_pushButtonSetPreferences_clicked()
{
    ENV.setDouble(Env::Kdistance,10.0);
    ENV.setDouble(Env::Kvisibility,35.0);
    ENV.setDouble(Env::Kreachable,50.0);
}

void OtpWidget::on_radioButtonOldHuman_toggled(bool checked)
{
    PlanEnv->setBool(PlanParam::env_useOldDude,checked);
}

void OtpWidget::on_pushButtonTestCompute_clicked()
{
    if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
    {
        std::vector<pair<double,double> > traj;
        configPt handConf = 0;
        Eigen::Vector3d dockPos;
        dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
        dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->InitMhpObjectTransfert("HERAKLES_HUMAN1");
        if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getOtp("HERAKLES_HUMAN1",dockPos,traj,handConf,
                                                                            PlanEnv->getBool(PlanParam::env_isStanding),
                                                                            PlanEnv->getDouble(PlanParam::env_objectNessecity)))
        {
            cout << dockPos << endl;
            cout << "success" << endl;
            ENV.setBool(Env::drawOTPTraj,true);
        }

    }
}
