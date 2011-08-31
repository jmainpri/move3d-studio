/*
 *  qtCost.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef QT_OTP_H
#define QT_OTP_H

#include "qtLibrary.hpp"
#include "qtMainInterface/mainwindow.hpp"
#include "qtBase/SpinBoxSliderConnector_p.hpp"


namespace Ui
{
	class OtpWidget;
};



/**
 * @ingroup qtMainWindow
 * @brief Qt Motion Planner
 */
class OtpWidget : public QWidget
{
	Q_OBJECT
	
public:
	OtpWidget(QWidget *parent = 0);
	~OtpWidget();
	
	void initOTP();
        void setMainWindow(MainWindow *ptrMW) { m_mainWindow = ptrMW; }
	Ui::OtpWidget* Ui() { return m_ui; }
        void initSliders();
        void changeDistCost();


private slots:

        void on_pushButtonTestCompute_clicked();
        void on_radioButtonOldHuman_toggled(bool checked);
        void on_pushButtonSetPreferences_clicked();
        void on_pushButtonStandUp_clicked();
        void on_pushButtonDumpCosts_clicked();
        void on_radioButtonAllGrid_toggled(bool checked);
        void on_pushButtonMultipleOTP_clicked();
        void on_radioButtonOrientedSlice_toggled(bool checked);
        void on_radioButtonUseSlice_toggled(bool checked);
        void on_pushButtonClearConfList_clicked();
        void on_radioButtonFusedGrid_toggled(bool checked);
        void on_radioButtonNormalRandom_toggled(bool checked);
        void on_radioButtonDistGrid_toggled(bool checked);
        void on_pushButtonComputeHumanRobotDist_clicked();
        void on_pushButtonDump_clicked();
        void on_pushButtonEraseTraj_clicked();
        void on_pushButtonnbConfs_clicked();
        void on_spinBoxXCell_valueChanged(int value);
        void on_spinBoxYCell_valueChanged(int value);
        void on_checkBox_clicked(bool checked);
        void on_pushButtonInitEnvGrid_clicked();
        void on_radioButtonRobGrid_toggled(bool checked);
        void on_radioButtonHumGrid_toggled(bool checked);
        void on_spinBoxOTPNav_valueChanged(int value);
        void on_pushButtonShowOTP_clicked();
        void on_pushButtonLoadConf_clicked();
        void on_pushButtonSaveConf_clicked();
        void on_pushButtonConfCost_clicked();
        void on_pushButtonInit_clicked();
        void on_pushButtonComputeConfortCost_clicked();
        void on_pushButtonLoadConfs_clicked();
        void on_pushButtonSaveConfs_clicked();
        void on_spinBoxNavigate_valueChanged(int value);
        void on_pushButtoRemoveLastconf_clicked();
        void on_pushButtonAddToconfList_clicked();
        void on_PushButtonSaveConfs_clicked();
        void on_pushButtonPlaceHuman_clicked();
        void on_pushButtonDrawOTPs_toggled(bool checked);
        void on_pushButtonPlaceRobot_clicked();
        void on_pushButtonLoadFromFile_clicked();
        void on_spinBoxNextOTP_valueChanged(int value);
        void on_pushButtonSaveToFile_clicked();
        void on_pushButtonShowList_clicked();
        void on_pushButtonRemoveLast_clicked();
        void on_pushButtonAddOTP_clicked();
        void on_spinBoxOTPth_valueChanged(int value);
        void on_pushButtonOTPth_clicked();
        void on_pushButton_2_clicked();
        void on_pushButtonShowTraj_clicked();
        void on_pushButtonNewComputeOTP_clicked();
        void on_pushButtonRecomputeAStarGridCosts_clicked();
        void on_pushButtonComputeAStar_clicked();
        void on_pushButtonCreateGrid_clicked();
        void on_spinBoxThreshold_valueChanged(int value);
        void on_RobotPosValuechanged(double value);
        void on_pushButtonGiveConf_clicked();
        void on_pushButtonComputeGIK_clicked();
        void on_pushButtonInitHumanPR2_clicked();
        void on_pushButtonPR2RestPose_clicked();
        void on_radioButtonFieldOfVision_toggled(bool checked);
        void on_radioButtonRobotDist_toggled(bool checked);
        void on_radioButtonHumanDist_toggled(bool checked);
        void on_radioButtonAll_toggled(bool checked);
        void on_pushButton_ShowOptions_toggled(bool checked);
        void on_pushButton_ComputeCosts_clicked();
        void on_pushButton_RobotBaseGrid_clicked();
        void on_pushButtonFastComputing_toggled(bool checked);
        void on_pushButton_clicked();
        void computeTheOtp();
        void drawAllWinActive();
        void launchOrStopTimer(bool isLaunched);
        void timerOrNot(bool isChecked);
        void moveRobots();


    signals:
      void selectedPlanner(QString);
private:
	Ui::OtpWidget*					m_ui;
	MainWindow*							m_mainWindow;
        QtShiva::SpinBoxSliderConnector*	m_k_distance;
        QtShiva::SpinBoxSliderConnector*	m_k_visbility;
        QtShiva::SpinBoxSliderConnector*	m_k_reachability;
        QtShiva::SpinBoxSliderConnector*	m_k_naturality;

        QtShiva::SpinBoxSliderConnector*	m_k_OptimalDist;
        QtShiva::SpinBoxSliderConnector*	m_k_RobotDistMax;
        QtShiva::SpinBoxSliderConnector*	m_k_GazeAngle;
        QtShiva::SpinBoxSliderConnector*	m_k_LimitRot;

        QtShiva::SpinBoxSliderConnector*	m_k_OptimalDistFactor;
        QtShiva::SpinBoxSliderConnector*	m_k_RobotDistMaxFactor;
        QtShiva::SpinBoxSliderConnector*	m_k_GazeAngleFactor;

        QtShiva::SpinBoxSliderConnector*	m_k_RobotPos;
        QtShiva::SpinBoxSliderConnector*	m_k_Obj;

        QtShiva::SpinBoxSliderConnector*	m_k_MaxIter;
        QtShiva::SpinBoxSliderConnector*	m_k_TotMaxIter;
        QtShiva::SpinBoxSliderConnector*	m_k_NbRandomRotOnly;
        QtShiva::SpinBoxSliderConnector*	m_k_NbSittingRotation;

        QtShiva::SpinBoxSliderConnector*	m_k_XMin;
        QtShiva::SpinBoxSliderConnector*	m_k_XMax;
        QtShiva::SpinBoxSliderConnector*	m_k_YMin;
        QtShiva::SpinBoxSliderConnector*	m_k_YMax;

        QtShiva::SpinBoxSliderConnector*	m_k_RobotSpeed;
        QtShiva::SpinBoxSliderConnector*	m_k_HumanSpeed;
        QtShiva::SpinBoxSliderConnector*	m_k_TimeStamp;

        QtShiva::SpinBoxSliderConnector*	m_k_Psi;
        QtShiva::SpinBoxSliderConnector*	m_k_delta;
        QtShiva::SpinBoxSliderConnector*	m_k_ksi;
        QtShiva::SpinBoxSliderConnector*	m_k_rho;

        QtShiva::SpinBoxSliderConnector*	m_k_sittingOffset;
        QtShiva::SpinBoxSliderConnector*	m_k_sleep;
        QtShiva::SpinBoxSliderConnector*	m_k_pow;

        QtShiva::SpinBoxSliderConnector*	m_k_nbComputeOTP;
        QtShiva::SpinBoxSliderConnector*	m_k_cellSize;

        QTimer *timer;
        QTimer *timerTraj;

        QString confFileName;


};

#endif