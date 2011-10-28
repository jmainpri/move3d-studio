/*
 *  qtCost.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef QT_HRICS_H
#define QT_HRICS_H

#include "qtLibrary.hpp"
#include "qtMainInterface/mainwindow.hpp"
#include "qtMainInterface/sideWidgets/qtMotionPlanner.hpp"
#include "API/Grids/gridsAPI.hpp"
#include "qtFormRobot/qtmovinghuman.hpp"

namespace Ui
{
	class HricsWidget;
};

/**
 * @ingroup qtMainWindow
 * @brief Qt Motion Planner
 */
class HricsWidget : public QWidget
{
	Q_OBJECT
	
public:
	HricsWidget(QWidget *parent = 0);
	~HricsWidget();
	
	void initHRI();
	//void initCost();
	void initVectorField();
	void initDrawOneLineInGrid();
	void initObjectTransferPoint();
	void initHumanLike();
  void initGrids();
  
  void drawAllWinActive();
	
	void setMainWindow(MainWindow *ptrMW) { m_mainWindow = ptrMW; }
	void setMotionWidget(MotionPlanner* ptrMLPW) { m_motionWidget = ptrMLPW; }
	
	Ui::HricsWidget* Ui() { return m_ui; }
	

private slots:
        void on_checkBoxPosOr_toggled(bool checked);
        void set_X_line(bool);
        void set_Y_line(bool);
        void set_Z_line(bool);
        void on_pushButton_initColPos_clicked();

	// Vector Field
	void computeVectorField();
	
	// OTP
	void computeObjectTransferPoint();
	
	// Workspace
	void make3DHriGrid();
	void delete3DHriGrid();
	void computeGridCost();
	void resetGridCost();
  void initAgentGrids();
  
	void AStarIn3DGrid();
	void HRICSRRT();
	void zoneSizeChanged();
	void resetRandomPoints();
  void setWhichTestSlot(int);
	
private:
        QtShiva::SpinBoxSliderConnector*	m_k_distance;
        QtShiva::SpinBoxSliderConnector*        m_k_visbility;
        QtShiva::SpinBoxSliderConnector*        m_k_reachability;
        QtShiva::SpinBoxSliderConnector*        m_k_naturality;
	
        MovingHuman*                            m_mh;

        Ui::HricsWidget*                        m_ui;
	
        MotionPlanner*                          m_motionWidget;

        MainWindow*                             m_mainWindow;
};

#endif
