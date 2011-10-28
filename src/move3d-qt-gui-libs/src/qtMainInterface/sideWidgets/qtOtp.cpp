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
#include <QDebug>

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

        m_k_sleep = new QtShiva::SpinBoxSliderConnector(this,         m_ui->doubleSpinBoxSleep,         m_ui->horizontalSliderSleep,        PlanParam::env_timeShow);
        m_k_timeLimit = new QtShiva::SpinBoxSliderConnector(this,     m_ui->doubleSpinBoxTimeLimit,     m_ui->horizontalSliderTimeLimit,    PlanParam::env_timeLimitation);
        m_k_timeLimit = new QtShiva::SpinBoxSliderConnector(this,     m_ui->doubleSpinBoxTimeLimitSit,  m_ui->horizontalSliderTimeLimitSit, PlanParam::env_sitTimeLimitation);
        m_k_sleep = new QtShiva::SpinBoxSliderConnector(this,         m_ui->doubleSpinBoxPow,           m_ui->horizontalSliderPow,          PlanParam::env_pow);


        m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxReach,Env::drawGrid);
        m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxrandomLimits,PlanParam::drawRandomMap);
        m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawSlice,PlanParam::env_drawSlice);
        m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawPoint,PlanParam::env_drawRandomPoint);
        m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxHumanTraj,PlanParam::env_showHumanTraj);


        m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawOnlyBest,PlanParam::env_drawOnlyBest);
        m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxIsStanding,PlanParam::env_isStanding);



}

void OtpWidget::initSliders()
{
        connect(m_ui->checkBoxReach,SIGNAL(clicked()),m_mainWindow,SLOT(drawAllWinActive()));
}

void OtpWidget::drawAllWinActive()
{
	m_mainWindow->drawAllWinActive();	
}

void OtpWidget::on_pushButtonPR2RestPose_clicked()
{
    if (dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getActivRobot()->getName().find("PR2") != string::npos)
    {
        dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->initPR2RepoConf();
        m_mainWindow->drawAllWinActive();
    }
}

void OtpWidget::on_pushButtonGiveConf_clicked()
{
    if (dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->getActivRobot()->getName().find("PR2") != string::npos)
    {
        dynamic_cast<HRICS::Workspace*>(HRICS_MotionPL)->initPR2GiveConf();
        m_mainWindow->drawAllWinActive();
    }
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

    if (!dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getReachability())
    {
            dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->setReachability(HRICS_activeNatu);
    }

    QString home( getenv("HOME_MOVE3D") );
    Robot* hum = dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getHuman();


    QString fileNameStand;
    QString fileNameSit;
    QString fileNameStandSlice;
    QString fileNameSitSlice;
    if (hum->getName().find("HERAKLES")!= string::npos)
        {
            fileNameStand = "/statFiles/OtpComputing/confHerakles.xml";
            fileNameSit = "/statFiles/OtpComputing/confHeraklesSit.xml";
            fileNameStandSlice = "/statFiles/OtpComputing/confHerakles.xml";
            fileNameSitSlice = "/statFiles/OtpComputing/confHeraklesSit.xml";
        }
        else if (hum->getName().find("OLDDUDE")!= string::npos)
        {
            fileNameStand = "/statFiles/OtpComputing/confOldDude.xml";
            fileNameSit = "/statFiles/OtpComputing/confOldDudeSit.xml";
            fileNameStandSlice = "/statFiles/OtpComputing/confOldDude.xml";
            fileNameSitSlice = "/statFiles/OtpComputing/confOldDudeSit.xml";
        }
        else if (hum->getName().find("ACHILE")!= string::npos)
        {
            fileNameStand = "/statFiles/OtpComputing/confAchile.xml";
            fileNameSit = "/statFiles/OtpComputing/confAchileSit.xml";
            fileNameStandSlice = "/statFiles/OtpComputing/confAchileTranche.xml";
            fileNameSitSlice = "/statFiles/OtpComputing/confAchileTrancheSit.xml";
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
                dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->initGrid();
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
    m_mainWindow->drawAllWinActive();
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

void OtpWidget::on_pushButton_toggled(bool checked)
{
    PlanEnv->setBool(PlanParam::env_realTime,checked);
    Eigen::Vector3d pos = dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getHumanActualPos();

    if (checked)
    {
        m_mh = new MovingHuman(pos[0],pos[1],pos[2]);
        m_mh->show();
        m_mh->setMainWindow(m_mainWindow);

        emit(selectedPlanner(QString("realTimeOtp")));
    }
    else
    {
        cout << "killing the window" << endl;
        if (m_mh)
        {
            m_mh->close();
            delete m_mh;
        }
    }
}

void OtpWidget::on_pushButtonTestTraj_clicked()
{
    if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
    {
            if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->isTheRealNearThePredicted(0.2))
            {
                cout << "the trajectories are almost the same" << endl;
            }
            else
            {
                cout << "the trajectories are too different" << endl;
            }
    }
}
