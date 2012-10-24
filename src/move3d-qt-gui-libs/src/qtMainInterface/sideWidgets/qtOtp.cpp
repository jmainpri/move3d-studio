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
#include <QProcess>

using namespace std;
using namespace tr1;


OtpWidget::OtpWidget(QWidget *parent) :
QWidget(parent),
m_ui(new Ui::OtpWidget)
{
    m_ui->setupUi(this);
    initOTP();
    reloadGuiParams();
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
//        m_k_Obj->setValue(0.9);

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
        m_k_delta = new QtShiva::SpinBoxSliderConnector(this,m_ui->doubleSpinBoxDelta,	        m_ui->horizontalSliderDelta,            PlanParam::env_delta);
        m_k_ksi = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxKsi,		m_ui->horizontalSliderKsi,		PlanParam::env_ksi);
        m_k_rho = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxRho,		m_ui->horizontalSliderRho,		PlanParam::env_rho);

        m_k_sittingOffset = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxSittingOffset,	m_ui->horizontalSliderSittingOffset,PlanParam::env_sittingOffset);
        m_k_nbComputeOTP  = new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxMOTP,          m_ui->horizontalSliderMOTP,         PlanParam::env_MOTP);
        m_k_cellSize  =     new QtShiva::SpinBoxSliderConnector(this, m_ui->doubleSpinBoxCellsize,      m_ui->horizontalSliderCellsize,     PlanParam::env_Cellsize);

        m_k_sleep = new QtShiva::SpinBoxSliderConnector(this,         m_ui->doubleSpinBoxSleep,         m_ui->horizontalSliderSleep,        PlanParam::env_timeShow);
        m_k_timeLimit = new QtShiva::SpinBoxSliderConnector(this,     m_ui->doubleSpinBoxTimeLimit,     m_ui->horizontalSliderTimeLimit,    PlanParam::env_timeLimitation);
        m_k_timeLimitSit = new QtShiva::SpinBoxSliderConnector(this,  m_ui->doubleSpinBoxTimeLimitSit,  m_ui->horizontalSliderTimeLimitSit, PlanParam::env_sitTimeLimitation);
        m_k_pow = new QtShiva::SpinBoxSliderConnector(this,           m_ui->doubleSpinBoxPow,           m_ui->horizontalSliderPow,          PlanParam::env_pow);
        m_k_pow_rot = new QtShiva::SpinBoxSliderConnector(this,       m_ui->doubleSpinBoxPowRot,        m_ui->horizontalSliderPowRot,       PlanParam::env_anglePow);
        m_k_time_dump = new QtShiva::SpinBoxSliderConnector(this,     m_ui->doubleSpinBoxTimeToDump,    m_ui->horizontalSliderTimeToDump,   PlanParam::env_timeToDump);


        m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxReach,Env::drawGrid);
        m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxCreateTrajs,PlanParam::env_createTrajs);
        m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxrandomLimits,PlanParam::drawRandomMap);
        m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawSlice,PlanParam::env_drawSlice);
        m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawPoint,PlanParam::env_drawRandomPoint);
        m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxHumanTraj,PlanParam::env_showHumanTraj);


        m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxDrawOnlyBest,PlanParam::env_drawOnlyBest);
        m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxIsStanding,PlanParam::env_isStanding);
        m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxOldCriteria,PlanParam::env_oldCriteria);
        m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxNoRepetition,PlanParam::env_noRepetition);
        m_mainWindow->connectCheckBoxToEnv(m_ui->checkBoxCreateHumanTraj,PlanParam::env_computeTrajForHuman);

//        m_ui->groupBoxTraj->hide();



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
    if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig) )
    {
        vector<vector<double> > path;
        dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getSimplePath(4.5,8,0,path);
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
    std::vector<Eigen::Vector3d> OTPList = dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getConfGenerator()->getOTPList();
    if (value < (int) OTPList.size())
    {
        dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getConfGenerator()->setCurOTP(OTPList.at(value));
        dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getConfGenerator()->placeRobot();
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
            dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getConfGenerator()->addToList(WSPoint);
            OTPList.push_back(WSPoint);

        }

        m_ui->spinBoxNextOTP->setMaximum(OTPList.size()-1);
    }
}

void OtpWidget::on_pushButtonPlaceRobot_clicked()
{
	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
                dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getConfGenerator()->placeRobot();
	}
	m_mainWindow->drawAllWinActive();
}

void OtpWidget::on_pushButtonDrawOTPs_toggled(bool checked)
{
	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
                dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getConfGenerator()->drawOTPList(checked);
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
                dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getConfGenerator()->addConfToList();
		m_ui->spinBoxNavigate->setMaximum(m_ui->spinBoxNavigate->maximum() + 1);
	}
}

void OtpWidget::on_pushButtoRemoveLastconf_clicked()
{
	if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
	{
                dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getConfGenerator()->removeLastConf();
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
                dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getConfGenerator()->saveToXml(fileName.toStdString());
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

    reloadGuiParams();
    clock_t first = clock();
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
//            m_mainWindow->setBoolFloor(false);
            m_mainWindow->drawAllWinActive();
    }

    m_ui->spinBoxXCell->setMaximum(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getPlanGrid()->getNbCellX()-1);
    m_ui->spinBoxYCell->setMaximum(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getPlanGrid()->getNbCellY()-1);

    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->changeHumanByName("HERAKLES_HUMAN1");
    cout << "WARNING: in order to make the soft work, the NAME HERAKLES_HUMAN1 has been hard coded, you can change it by using changeHumanByName(str)" << endl;

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
    clock_t second = clock();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->initGrid();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->setInitTime(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getInitTime() +
                                                                         ((double)second - first) / CLOCKS_PER_SEC );
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
    ENV.setDouble(Env::Kdistance,40.0);
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

//        dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
        dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->InitMhpObjectTransfert("HERAKLES_HUMAN1");
        dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
//        if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getOtp("HERAKLES_HUMAN1",dockPos,traj,handConf,
//                                                                            PlanEnv->getBool(PlanParam::env_isStanding),
//                                                                            PlanEnv->getDouble(PlanParam::env_objectNessecity)))
        if (PlanEnv->getBool(PlanParam::env_trajRos))
        {
            std::vector<std::vector<double> > traj;
            configPt handConf = 0;
            if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getOtp("HERAKLES_HUMAN1",traj,handConf,
                                                                                PlanEnv->getBool(PlanParam::env_isStanding),
                                                                                PlanEnv->getDouble(PlanParam::env_objectNessecity)))
            {
                cout << "success" << endl;
                for (unsigned int i = 0; i < traj.size(); i++)
                {
                    cout << "pos nb " << i << " is : x = " << traj.at(i).at(0) << ", y = " << traj.at(i).at(1) << ", theta = " << traj.at(i).at(2) << endl;
                }
                ENV.setBool(Env::drawOTPTraj,true);
            }
        }
        else
        {
            std::vector<pair<double,double> > traj;
            std::vector<SM_TRAJ> smTraj;
            configPt handConf = 0;
            Eigen::Vector3d dockPos;
            if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getOtp("HERAKLES_HUMAN1",dockPos,smTraj,handConf,
                                                                                PlanEnv->getBool(PlanParam::env_isStanding),
                                                                                PlanEnv->getDouble(PlanParam::env_objectNessecity)))
            {
                cout << dockPos << endl;
                cout << "success" << endl;
                ENV.setBool(Env::drawOTPTraj,true);
            }
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

void OtpWidget::on_pushButtonSoftMotion_clicked()
{
    PlanEnv->setBool(PlanParam::env_softMotionTraj,!PlanEnv->getBool(PlanParam::env_softMotionTraj));
}



void OtpWidget::on_pushButtonSmooth_clicked()
{
    if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
    {
            dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->setRobotTraj(
                    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->smoothTrajectory(
                            dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getRobotTraj()));
    }
}

void OtpWidget::on_pushButtonInitTraj_clicked()
{
    if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
    {
            dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->initTrajTest();
    }
}

void OtpWidget::on_pushButtonNextStep_clicked()
{
    if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
    {
            dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->goToNextStep();
    }
    drawAllWinActive();
}

void OtpWidget::on_pushButtonTestCol_clicked()
{
    Robot* robCyl = NULL;
    for (int i=0; i<XYZ_ENV->nr; i++)
    {
        string name(XYZ_ENV->robot[i]->name);
        if(name.find("PR_2CYLINDER") != string::npos )
        {
            robCyl = new Robot(XYZ_ENV->robot[i]);
        }

    }
    if (robCyl->isInCollisionWithOthersAndEnv())
    {
        cout << "The cylinder is in colision with the environment" << endl;
    }
    else
    {
        cout << "No colision detected between the cylinder and the environment" << endl;
    }
}

void OtpWidget::on_pushButtonTestTraj_2_clicked()
{
    if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
    {
        if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->testCurrentTraj())
        {
            cout << "this trajectory can be done" << endl;
        }
        else
        {
            cout << "this trajectory is unfeasable" << endl;
        }
    }
    drawAllWinActive();
}

void OtpWidget::on_pushButtonPlan_clicked()
{
    emit(selectedPlanner(QString("simpleNav")));
}

void OtpWidget::on_pushButtonGridVar_clicked()
{
    if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
    {
        dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getPlanGrid()->dumpVar();
    }
}

void OtpWidget::on_radioButtonBiasAndRot_toggled(bool checked)
{
    PlanEnv->setBool(PlanParam::env_fusedGridAndRotRand,checked);
}

void OtpWidget::on_pushButtonTestTrajs_clicked()
{
    if (dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig))
    {
        dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->testTrajectories(true);
    }
}

void OtpWidget::on_pushButtonScript_clicked()
{

    //mobility:
//    m_k_Obj->setValue(0.9);

    // variant
    //    on_radioButtonAllGrid_toggled(true);
    //    on_radioButtonFusedGrid_toggled(true);
    //    on_radioButtonBiasAndRot_toggled(true);



    //cell size:
//    m_k_cellSize->setValue(0.2);

    //init:
//    on_pushButtonInit_clicked();
    //OTP:
    //on_pushButtonMultipleOTP_clicked();

    QDir dir(QString("/home/magharbi/openrobots/data/BioMove3D/statFiles/OtpComputing"));
    std::vector<std::vector<double> > results;
    std::vector<QString> names;
    names.push_back("AllGridHM0.2.lst");
    names.push_back("AllGridMM0.2.lst");
    names.push_back("AllGridLM0.2.lst");
    names.push_back("FusedGridOnlyHM0.2.lst");
    names.push_back("FusedGridOnlyMM0.2.lst");
    names.push_back("FusedGridOnlyLM0.2.lst");
    names.push_back("FusedGridHM0.2.lst");
    names.push_back("FusedGridMM0.2.lst");
    names.push_back("FusedGridLM0.2.lst");
    names.push_back("FusedGridBiasHM0.2.lst");
    names.push_back("FusedGridBiasMM0.2.lst");
    names.push_back("FusedGridBiasLM0.2.lst");

    names.push_back("AllGridHM0.15.lst");
    names.push_back("AllGridMM0.15.lst");
    names.push_back("AllGridLM0.15.lst");
    names.push_back("FusedGridOnlyHM0.15.lst");
    names.push_back("FusedGridOnlyLM0.15.lst");
    names.push_back("FusedGridOnlyLM0.15.lst");
    names.push_back("FusedGridHM0.15.lst");
    names.push_back("FusedGridMM0.15.lst");
    names.push_back("FusedGridLM0.15.lst");
    names.push_back("FusedGridBiasHM0.15.lst");
    names.push_back("FusedGridBiasMM0.15.lst");
    names.push_back("FusedGridBiasLM0.15.lst");

    names.push_back("AllGridHM0.1.lst");
    names.push_back("AllGridMM0.1.lst");
    names.push_back("AllGridLM0.1.lst");
    names.push_back("FusedGridOnlyHM0.1.lst");
    names.push_back("FusedGridOnlyLM0.1.lst");
    names.push_back("FusedGridOnlyLM0.1.lst");
    names.push_back("FusedGridHM0.1.lst");
    names.push_back("FusedGridMM0.1.lst");
    names.push_back("FusedGridLM0.1.lst");
    names.push_back("FusedGridBiasHM0.1.lst");
    names.push_back("FusedGridBiasMM0.1.lst");
    names.push_back("FusedGridBiasLM0.1.lst");

    std::vector<std::string> numbersId;
    numbersId.push_back("Average time = ");
    numbersId.push_back("Average init Grid time = ");
    numbersId.push_back("Average First conf computing time = ");
    numbersId.push_back("Average loop time = ");
    numbersId.push_back("Average cost = ");
    numbersId.push_back("Average nb Iteration = ");
    numbersId.push_back("Average nb solutions = ");
    numbersId.push_back("Average rate = ");
    numbersId.push_back("Nb of OTP computed = ");

    PlanEnv->setBool(PlanParam::env_createTrajs,false);
    PlanEnv->setDouble(PlanParam::env_timeToDump,0.01);


    int i = 0;
    qDebug() << "\n\n###################################################\n cells size = 0.2" << endl;
    m_k_cellSize->setValue(0.2);

    qDebug() << "\n\n################ all grid \n " ;

    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(1.0);
    on_radioButtonAllGrid_toggled(true);
    on_radioButtonOldHuman_toggled(false);
    on_radioButtonFusedGrid_toggled(false);
    on_radioButtonBiasAndRot_toggled(false);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;

    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(0.35);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;


    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(0.0);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;

    qDebug() << "\n\n################ fused grid \n " ;

    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(1.0);
    on_radioButtonAllGrid_toggled(false);
    on_radioButtonOldHuman_toggled(true);
    on_radioButtonFusedGrid_toggled(false);
    on_radioButtonBiasAndRot_toggled(false);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;

    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(0.35);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;


    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(0.0);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;

    qDebug() << "\n\n################ fused grid  with bias\n " ;

    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(1.0);
    on_radioButtonAllGrid_toggled(false);
    on_radioButtonOldHuman_toggled(false);
    on_radioButtonFusedGrid_toggled(true);
    on_radioButtonBiasAndRot_toggled(false);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;

    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(0.35);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;


    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(0.0);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;


    qDebug() << "\n\n################ fused grid  with bias and rot\n " ;

    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(1.0);
    on_radioButtonAllGrid_toggled(false);
    on_radioButtonOldHuman_toggled(false);
    on_radioButtonFusedGrid_toggled(false);
    on_radioButtonBiasAndRot_toggled(true);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;

    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(0.35);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;


    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(0.0);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;


    qDebug() << "\n\n###################################################\n cells size = 0.15" << endl;
    m_k_cellSize->setValue(0.15);

    qDebug() << "\n\n################ all grid \n " ;

    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(1.0);
    on_radioButtonAllGrid_toggled(true);
    on_radioButtonOldHuman_toggled(false);
    on_radioButtonFusedGrid_toggled(false);
    on_radioButtonBiasAndRot_toggled(false);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;

    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(0.35);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;


    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(0.0);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;

    qDebug() << "\n\n################ fused grid \n " ;

    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(1.0);
    on_radioButtonAllGrid_toggled(false);
    on_radioButtonOldHuman_toggled(true);
    on_radioButtonFusedGrid_toggled(false);
    on_radioButtonBiasAndRot_toggled(false);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;

    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(0.35);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;


    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(0.0);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;

    qDebug() << "\n\n################ fused grid  with bias\n " ;

    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(1.0);
    on_radioButtonAllGrid_toggled(false);
    on_radioButtonOldHuman_toggled(false);
    on_radioButtonFusedGrid_toggled(true);
    on_radioButtonBiasAndRot_toggled(false);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;

    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(0.35);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;


    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(0.0);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;


    qDebug() << "\n\n################ fused grid  with bias and rot\n " ;

    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(1.0);
    on_radioButtonAllGrid_toggled(false);
    on_radioButtonOldHuman_toggled(false);
    on_radioButtonFusedGrid_toggled(false);
    on_radioButtonBiasAndRot_toggled(true);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;

    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(0.35);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;


    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(0.0);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;




    qDebug() << "\n\n###################################################\n cells size = 0.1" << endl;
    m_k_cellSize->setValue(0.1);

    qDebug() << "\n\n################ all grid \n " ;

    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(1.0);
    on_radioButtonAllGrid_toggled(true);
    on_radioButtonOldHuman_toggled(false);
    on_radioButtonFusedGrid_toggled(false);
    on_radioButtonBiasAndRot_toggled(false);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;

    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(0.35);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;


    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(0.0);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;

    qDebug() << "\n\n################ fused grid \n " ;

    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(1.0);
    on_radioButtonAllGrid_toggled(false);
    on_radioButtonOldHuman_toggled(true);
    on_radioButtonFusedGrid_toggled(false);
    on_radioButtonBiasAndRot_toggled(false);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;

    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(0.35);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;


    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(0.0);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;

    qDebug() << "\n\n################ fused grid  with bias\n " ;

    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(1.0);
    on_radioButtonAllGrid_toggled(false);
    on_radioButtonOldHuman_toggled(false);
    on_radioButtonFusedGrid_toggled(true);
    on_radioButtonBiasAndRot_toggled(false);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;

    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(0.35);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;


    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(0.0);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;


    qDebug() << "\n\n################ fused grid  with bias and rot\n " ;

    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(1.0);
    on_radioButtonAllGrid_toggled(false);
    on_radioButtonOldHuman_toggled(false);
    on_radioButtonFusedGrid_toggled(false);
    on_radioButtonBiasAndRot_toggled(true);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;

    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(0.35);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;


    qDebug() << "\n\n################\n" << names.at(i);
    m_k_Obj->setValue(0.0);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(i));
    dir.rename("configCosts.lst", names.at(i));
    dir.remove(names.at(i)+QString("costsMax.lst"));
    dir.rename("costsMax.lst", names.at(i)+QString("costsMax.lst"));
    dir.remove(names.at(i)+QString("costsMin.lst"));
    dir.rename("costsMin.lst", names.at(i)+QString("costsMin.lst"));
    dir.remove(names.at(i)+QString("costsVariance.lst"));
    dir.rename("costsVariance.lst", names.at(i)+QString("costsVariance.lst"));
    i++;



    for (unsigned int i = 0; i < results.size(); i++)
    {

        qDebug() << "\n\n################\n" << names.at(i);
        if (numbersId.size() == results.at(i).size())
        {
            for (unsigned int j = 0; j < numbersId.size(); j++)
            {
                cout << numbersId.at(j) << results.at(i).at(j) <<endl;
            }
            for (unsigned int j = 0; j < numbersId.size(); j++)
            {
                cout << results.at(i).at(j) <<endl;
            }
        }
        else
        {
            cout << "ERROR: bad numbers of results" << endl;
        }
    }

    /*

    qDebug() << "\n\n###################################################\n cells size = 0.2" << endl;
    m_k_cellSize->setValue(0.2);

    qDebug() << "\n\n################\n" << names.at(0);
    m_k_Obj->setValue(0.9);
    on_radioButtonAllGrid_toggled(true);
    on_radioButtonOldHuman_toggled(false);
    on_radioButtonFusedGrid_toggled(false);
    on_radioButtonBiasAndRot_toggled(false);

    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(0));
    dir.rename("configCosts.lst", names.at(0));


    qDebug() << "\n\n################\n" << names.at(1);
    m_k_Obj->setValue(0.1);
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(1));
    dir.rename("configCosts.lst", names.at(1));

    qDebug() << "\n\n################\n" << names.at(2);
    m_k_Obj->setValue(0.9);
    on_radioButtonAllGrid_toggled(false);
    on_radioButtonOldHuman_toggled(true);
    on_radioButtonFusedGrid_toggled(false);
    on_radioButtonBiasAndRot_toggled(false);
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(2));
    dir.rename("configCosts.lst", names.at(2));


    qDebug() << "\n\n################\n" << names.at(3);
    m_k_Obj->setValue(0.1);
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(3));
    dir.rename("configCosts.lst", names.at(3));

    qDebug() << "\n\n################\n" << names.at(4);
    m_k_Obj->setValue(0.9);
    on_radioButtonAllGrid_toggled(false);
    on_radioButtonOldHuman_toggled(false);
    on_radioButtonFusedGrid_toggled(true);
    on_radioButtonBiasAndRot_toggled(false);
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(4));
    dir.rename("configCosts.lst", names.at(4));


    qDebug() << "\n\n################\n" << names.at(5);
    m_k_Obj->setValue(0.1);
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(5));
    dir.rename("configCosts.lst", names.at(5));


    qDebug() << "\n\n################\n" << names.at(6);
    m_k_Obj->setValue(0.9);
    on_radioButtonAllGrid_toggled(false);
    on_radioButtonOldHuman_toggled(false);
    on_radioButtonFusedGrid_toggled(false);
    on_radioButtonBiasAndRot_toggled(true);
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(6));
    dir.rename("configCosts.lst", names.at(6));


    qDebug() << "\n\n################\n" << names.at(7);
    m_k_Obj->setValue(0.1);
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(7));
    dir.rename("configCosts.lst", names.at(7));


    
    
    qDebug() << "\n\n###################################################\n cells size = 0.15" << endl;
    m_k_cellSize->setValue(0.15);

    qDebug() << "\n\n################\n" << names.at(8);
    m_k_Obj->setValue(0.9);
    on_radioButtonAllGrid_toggled(true);
    on_radioButtonOldHuman_toggled(false);
    on_radioButtonFusedGrid_toggled(false);
    on_radioButtonBiasAndRot_toggled(false);
    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(8));
    dir.rename("configCosts.lst", names.at(8));


    qDebug() << "\n\n################\n" << names.at(9);
    m_k_Obj->setValue(0.1);
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(9));
    dir.rename("configCosts.lst", names.at(9));


    qDebug() << "\n\n################\n" << names.at(10);
    m_k_Obj->setValue(0.9);
    on_radioButtonAllGrid_toggled(false);
    on_radioButtonOldHuman_toggled(true);
    on_radioButtonFusedGrid_toggled(false);
    on_radioButtonBiasAndRot_toggled(false);
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(10));
    dir.rename("configCosts.lst", names.at(10));


    qDebug() << "\n\n################\n" << names.at(11);
    m_k_Obj->setValue(0.1);
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(11));
    dir.rename("configCosts.lst", names.at(11));

    qDebug() << "\n\n################\n" << names.at(12);
    m_k_Obj->setValue(0.9);
    on_radioButtonAllGrid_toggled(false);
    on_radioButtonOldHuman_toggled(false);
    on_radioButtonFusedGrid_toggled(true);
    on_radioButtonBiasAndRot_toggled(false);
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(12));
    dir.rename("configCosts.lst", names.at(12));


    qDebug() << "\n\n################\n" << names.at(13);
    m_k_Obj->setValue(0.1);
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(13));
    dir.rename("configCosts.lst", names.at(13));


    qDebug() << "\n\n################\n" << names.at(14);
    m_k_Obj->setValue(0.9);
    on_radioButtonAllGrid_toggled(false);
    on_radioButtonOldHuman_toggled(false);
    on_radioButtonFusedGrid_toggled(false);
    on_radioButtonBiasAndRot_toggled(true);
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(14));
    dir.rename("configCosts.lst", names.at(14));


    qDebug() << "\n\n################\n" << names.at(15);
    m_k_Obj->setValue(0.1);
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(15));
    dir.rename("configCosts.lst", names.at(15));

    
    qDebug() << "\n\n###################################################\n cells size = 0.1" << endl;
    m_k_cellSize->setValue(0.1);

    qDebug() << "\n\n################\n" << names.at(16);
    m_k_Obj->setValue(0.9);
    on_radioButtonAllGrid_toggled(true);
    on_radioButtonOldHuman_toggled(false);
    on_radioButtonFusedGrid_toggled(false);
    on_radioButtonBiasAndRot_toggled(false);
    on_pushButtonInit_clicked();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(16));
    dir.rename("configCosts.lst", names.at(16));


    qDebug() << "\n\n################\n" << names.at(17);
    m_k_Obj->setValue(0.1);
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(17));
    dir.rename("configCosts.lst", names.at(17));


    qDebug() << "\n\n################\n" << names.at(18);
    m_k_Obj->setValue(0.9);
    on_radioButtonAllGrid_toggled(false);
    on_radioButtonOldHuman_toggled(true);
    on_radioButtonFusedGrid_toggled(false);
    on_radioButtonBiasAndRot_toggled(false);
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(18));
    dir.rename("configCosts.lst", names.at(18));


    qDebug() << "\n\n################\n" << names.at(19);
    m_k_Obj->setValue(0.1);
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(19));
    dir.rename("configCosts.lst", names.at(19));

    qDebug() << "\n\n################\n" << names.at(20);
    m_k_Obj->setValue(0.9);
    on_radioButtonAllGrid_toggled(false);
    on_radioButtonOldHuman_toggled(false);
    on_radioButtonFusedGrid_toggled(true);
    on_radioButtonBiasAndRot_toggled(false);
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(20));
    dir.rename("configCosts.lst", names.at(20));


    qDebug() << "\n\n################\n" << names.at(21);
    m_k_Obj->setValue(0.1);
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(21));
    dir.rename("configCosts.lst", names.at(21));


    qDebug() << "\n\n################\n" << names.at(22);
    m_k_Obj->setValue(0.9);
    on_radioButtonAllGrid_toggled(false);
    on_radioButtonOldHuman_toggled(false);
    on_radioButtonFusedGrid_toggled(false);
    on_radioButtonBiasAndRot_toggled(true);
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(22));
    dir.rename("configCosts.lst", names.at(22));


    qDebug() << "\n\n################\n" << names.at(23);
    m_k_Obj->setValue(0.1);
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->dumpVar();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->saveInitConf();
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->multipliComputeOtp(PlanEnv->getInt(PlanParam::env_MOTP));
    dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->loadInitConf(true,true);

    results.push_back(dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getMultipleData());
    dir.remove(names.at(23));
    dir.rename("configCosts.lst", names.at(23));
    
    
    
    
    
    for (unsigned int i = 0; i < results.size(); i++)
    {

        qDebug() << "\n\n################\n" << names.at(i);
        if (numbersId.size() == results.at(i).size())
        {
            for (unsigned int j = 0; j < numbersId.size(); j++)
            {
                cout << numbersId.at(j) << results.at(i).at(j) <<endl;
            }
            for (unsigned int j = 0; j < numbersId.size(); j++)
            {
                cout << results.at(i).at(j) <<endl;
            }
        }
        else
        {
            cout << "ERROR: bad numbers of results" << endl;
        }
    }
    */

}

void OtpWidget::on_pushButtonPreInit_clicked()
{
    double avT = 0;
    for (int i =0; i < PlanEnv->getInt(PlanParam::env_MOTP); i++)
    {
        on_pushButtonInit_clicked();
        avT += dynamic_cast<HRICS::OTPMotionPl*>(HRICS_MotionPLConfig)->getInitTime();
    }
    avT = avT / PlanEnv->getInt(PlanParam::env_MOTP);
    cout << "average init Time = " << avT << endl;




}

void OtpWidget::on_radioButtonNormal_toggled(bool checked)
{
    PlanEnv->setBool(PlanParam::env_trajNormal,checked);
}

void OtpWidget::on_radioButtonSoftMotion_toggled(bool checked)
{
    PlanEnv->setBool(PlanParam::env_trajSoftMotion,checked);
}

void OtpWidget::on_radioButtonRos_toggled(bool checked)
{
    PlanEnv->setBool(PlanParam::env_trajRos,checked);
}


void OtpWidget::reloadGuiParams()
{
    m_ui->checkBoxIsStanding->setChecked(PlanEnv->getBool(PlanParam::env_isStanding));
    m_k_Obj->setValue(PlanEnv->getDouble(PlanParam::env_objectNessecity));
    m_k_cellSize->setValue(PlanEnv->getDouble(PlanParam::env_Cellsize));

    m_ui->radioButtonNormalRandom->setChecked(PlanEnv->getBool(PlanParam::env_normalRand));
    m_ui->radioButtonAllGrid->setChecked(PlanEnv->getBool(PlanParam::env_useAllGrid));
    m_ui->radioButtonFusedGrid->setChecked(PlanEnv->getBool(PlanParam::env_fusedGridRand));
    m_ui->radioButtonBiasAndRot->setChecked(PlanEnv->getBool(PlanParam::env_fusedGridAndRotRand));
    m_ui->radioButtonUseSlice->setChecked(PlanEnv->getBool(PlanParam::env_useSlice));
    m_ui->radioButtonOrientedSlice->setChecked(PlanEnv->getBool(PlanParam::env_useOrientedSlice));
    m_ui->radioButtonOldHuman->setChecked(PlanEnv->getBool(PlanParam::env_useOldDude));

    m_ui->checkBoxOldCriteria->setChecked(PlanEnv->getBool(PlanParam::env_oldCriteria));
    m_ui->checkBoxNoRepetition->setChecked(PlanEnv->getBool(PlanParam::env_noRepetition));

    m_k_sleep->setValue(PlanEnv->getInt(PlanParam::env_timeShow));
    m_k_MaxIter->setValue(PlanEnv->getInt(PlanParam::env_maxIter));
    m_k_TotMaxIter->setValue(PlanEnv->getInt(PlanParam::env_totMaxIter));
    m_k_timeLimit->setValue(PlanEnv->getDouble(PlanParam::env_timeLimitation));
    m_k_timeLimitSit->setValue(PlanEnv->getDouble(PlanParam::env_sitTimeLimitation));
    m_k_time_dump->setValue(PlanEnv->getDouble(PlanParam::env_timeToDump));

    m_ui->checkBoxHumanTraj->setChecked(PlanEnv->getBool(PlanParam::env_showHumanTraj));
    m_k_nbComputeOTP->setValue(PlanEnv->getInt(PlanParam::env_MOTP));

    m_ui->checkBoxDrawSlice->setChecked(PlanEnv->getBool(PlanParam::env_drawSlice));
    m_ui->checkBoxCreateTrajs->setChecked(PlanEnv->getBool(PlanParam::env_createTrajs));
    m_ui->radioButtonNormal->setChecked(PlanEnv->getBool(PlanParam::env_trajNormal));
    m_ui->radioButtonSoftMotion->setChecked(PlanEnv->getBool(PlanParam::env_trajSoftMotion));
    m_ui->radioButtonRos->setChecked(PlanEnv->getBool(PlanParam::env_trajRos));
    m_ui->checkBoxCreateHumanTraj->setChecked(PlanEnv->getBool(PlanParam::env_computeTrajForHuman));

    m_ui->checkBoxReach->setChecked(ENV.getBool(Env::drawGrid));
    m_ui->checkBoxDrawPoint->setChecked(PlanEnv->getBool(PlanParam::env_drawRandomPoint));
    m_ui->radioButtonHumGrid->setChecked(PlanEnv->getBool(PlanParam::env_humanGridDraw));
    m_ui->radioButtonRobGrid->setChecked(PlanEnv->getBool(PlanParam::env_robotGridDraw));
    m_ui->radioButtonDistGrid->setChecked(PlanEnv->getBool(PlanParam::env_drawDistGrid));

    m_k_distance->setValue(ENV.getDouble(Env::Kdistance));
    m_k_visbility->setValue(ENV.getDouble(Env::Kvisibility));
    m_k_naturality->setValue(ENV.getDouble(Env::Knatural));
    m_k_reachability->setValue(ENV.getDouble(Env::Kreachable));

    m_k_pow->setValue(PlanEnv->getInt(PlanParam::env_pow));
    m_k_pow_rot->setValue(PlanEnv->getInt(PlanParam::env_anglePow));

    m_k_NbRandomRotOnly->setValue(PlanEnv->getInt(PlanParam::env_nbRandomRotOnly));
    m_k_NbSittingRotation->setValue(PlanEnv->getInt(PlanParam::env_nbSittingRotation));

    m_k_XMin->setValue(PlanEnv->getDouble(PlanParam::env_randomXMinLimit));
    m_k_XMax->setValue(PlanEnv->getDouble(PlanParam::env_randomXMaxLimit));
    m_k_YMin->setValue(PlanEnv->getDouble(PlanParam::env_randomYMinLimit));
    m_k_YMax->setValue(PlanEnv->getDouble(PlanParam::env_randomYMaxLimit));
    m_k_RobotSpeed->setValue(PlanEnv->getDouble(PlanParam::env_robotSpeed));
    m_k_HumanSpeed->setValue(PlanEnv->getDouble(PlanParam::env_humanSpeed));
    m_k_TimeStamp->setValue(PlanEnv->getDouble(PlanParam::env_timeStamp));
    m_k_Psi->setValue(PlanEnv->getDouble(PlanParam::env_psi));
    m_k_delta->setValue(PlanEnv->getDouble(PlanParam::env_delta));
    m_k_ksi->setValue(PlanEnv->getDouble(PlanParam::env_ksi));
    m_k_rho->setValue(PlanEnv->getDouble(PlanParam::env_rho));
    m_k_sittingOffset->setValue(PlanEnv->getDouble(PlanParam::env_sittingOffset));

}
