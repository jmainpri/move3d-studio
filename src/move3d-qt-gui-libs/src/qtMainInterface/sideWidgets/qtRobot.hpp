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

#ifdef MULTILOCALPATH
namespace Manip
{
    void runManipulation();
    void runNavigation();
    void runCurrentTest();
}
#endif

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
    void initTrajectoryFromConfig();

    MoveRobot* getMoveRobot();

private slots:

    void on_pushButtonLoadConf_clicked();
    void on_pushButtonSaveCurConf_clicked();
    void on_pushButtonPlanNavigation_clicked();
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
#ifdef MULTILOCALPATH
    void setRobotAtInitConfig();
    void setRobotAtGoalConfig();
    void setRobotAtOpenConfig();

    void setInitConfigAtCurrent();
    void setGoalConfigAtCurrent();

    void objectNameChanged(int id);
    void placementNameChanged(int id);
    void supportNameChanged(int id);

    void isDebugManip(bool value);
    void isCartesianMode(bool on);
    void isToFreePoint(bool value);
    void isUsingMobileBase(bool value);
  
    void resetManipulationData();
    void runManipTest();
    void optimizeRedundantCost();

    void armFree();
    void armPickGoto();
    void armTakeToFree();
    void armTakeToPlace();
    void armPlaceFromFree();
    void armExtract();
    void armReplanTask();
    void armEscapeObject();
    void loadWorkspace();
#endif
    /*void initVoxelCollisionChecker();
	 void createVoxelCC();
	 void deleteVoxelCC();
	 
	 void voxelCCTest();*/

    void makeSoftmotionTraj();
    void makeNormalTraj();
    void saveConfig();
    void clearConfigs();
    void deleteConfig();

signals:
    void selectedPlanner(QString);

private:
    void initModel();
    void initManipulation();
    void initObjectSupportAndPlacementCombo();
    void callToManipulationPlanner();
    std::string getNameOfFreeFlyerFromIndex(int id);

    Ui::RobotWidget *m_ui;
    MainWindow *m_mainWindow;
    std::vector<QString> m_FreeFlyers;
    QString m_ObjectName;
    QString m_SupportName;
    QString m_PlacementName;
    int m_configNum;
};

#endif
