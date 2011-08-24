/*
 *  qtRobot.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef QT_ROBOT_H
#define QT_ROBOT_H

#include "qtLibrary.hpp"
#include "qtFormRobot/moverobot.hpp"
#include "qtMainInterface/mainwindow.hpp"

namespace Ui
{
	class RobotWidget;
}

namespace Manip
{
  void runManipulation();
  void runCurrentTest();
}

/**
 * @ingroup qtMainWindow
 * @brief Qt Motion Planner
 */
class RobotWidget : public QWidget
{
	Q_OBJECT
	
public:
	RobotWidget(QWidget *parent = 0);
	~RobotWidget();
	
	void setMainWindow(MainWindow *ptrMW) { m_mainWindow = ptrMW; }
	
	void initRobot();
	
	MoveRobot* getMoveRobot();
	
private slots:
  
	// Test Model -------------------------
 void on_pushButtonrefrech_clicked();
 void on_spinBoxNavigate_valueChanged(int value);
 void costTest();
	void collisionsTest();
	void localpathsTest();
	void allTests();
	void setAttMatrix();
	
	// Hri Planner ------------------------
	// HRI GIK functions
	void computeHriGik(bool leftArm);
	
#if defined (HRI_PLANNER)
	void computeHriGikLARM() { this->computeHriGik(true); }
	void computeHriGikRARM() { this->computeHriGik(false); }
#endif
	
	// Grab Object ------------------------
	void GrabObject();
	void ReleaseObject();
	void currentObjectChange(int i);
	void SetObjectToCarry();
	
	// MISC -------------------------------
	void printCurrentPos();
	void printAbsPos();
	
#ifdef LIGHT_PLANNER
	void switchFKIK();
#endif
	
	void printPQPColPair();
	
	// Manipulation -----------------------
  void isDebugManip(bool value);
  void isCartesianMode(bool on);
  void resetManipulationData();
  void runManipTest();
  void optimizeRedundantCost();
	void armFree();
	void armPickGoto();
	void armPickTakeToFree();
  void armPickTakeToFreePoint();
	void armPickGotoAndTakeToFree();
  void armReplanTask();
  void loadWorkspace();
	
	/*void initVoxelCollisionChecker();
	 void createVoxelCC();
	 void deleteVoxelCC();
	 
	 void voxelCCTest();*/

  void launch();
  void saveConfig();
  void clearConfigs();
  void deleteConfig();
  
signals:
  void selectedPlanner(QString);
	
private:
	Ui::RobotWidget *m_ui;
	
	MainWindow *m_mainWindow;
	
	std::vector<QString> mFreeFlyers;
	
	void initModel();
	void initManipulation();
};

#endif
