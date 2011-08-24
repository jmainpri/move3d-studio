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
	
#ifdef HRI_COSTSPACE
	void initHRI();
#endif
	//void initCost();
	void initVectorField();
	void initDrawOneLineInGrid();
	void initNaturalSpace();
	void initConfigSpace();
	void initTaskSpace();
	void initObjectTransferPoint();
	void initHumanLike();
  void initGrids();
	
	void setMainWindow(MainWindow *ptrMW) { m_mainWindow = ptrMW; }
	void setMotionWidget(MotionPlanner* ptrMLPW) { m_motionWidget = ptrMLPW; }
	
	Ui::HricsWidget* Ui() { return m_ui; }
	
	private slots:
#ifdef HRI_COSTSPACE
	
	// Vector Field
	void computeVectorField();
	
	// OTP
	void computeObjectTransferPoint();
	
	void set_X_line(bool enable);
	void set_Y_line(bool enable);
	void set_Z_line(bool enable);
	
	void saveActiveGridToFile();
	void loadActiveGridFromFile();
	// HRI ----------------------------------------
	// Natural
	void newNaturalCostSpace();
	void deleteNaturalCostSpace();
	void recomputeReachability();
	void computeAllCellNaturalCost();
	void computeReachability();
	void getSortedReachableWSPoint();
	
	void on_pushButton_NaturalParam_toggled(bool checked);
	void on_pushButton_gridProperties_toggled(bool checked);
	void on_pushButtonInitBaseGrid_clicked();
	void drawAllWinActive();
	void on_pushButton_initColPos_clicked();

	void on_horizontalSlider_zmax_sliderMoved(int position);
	void on_horizontalSlider_ymax_sliderMoved(int position);
	void on_horizontalSlider_xmax_sliderMoved(int position);
	void on_horizontalSlider_zmin_sliderMoved(int position);
	void on_horizontalSlider_ymin_sliderMoved(int position);
	void on_horizontalSlider_xmin_sliderMoved(int position);



	// CSpace
	void newHRIConfigSpace();
	void deleteHRIConfigSpace();
	void makeGridHRIConfigSpace();
	void makePlanHRIConfigSpace();
	void AStarInPlanHRIConfigSpace();
	void writeToOBPlane();
	void hriPlanRRT();
	
	// Workspace
	void make3DHriGrid();
	void delete3DHriGrid();
	void computeGridCost();
	void resetGridCost();
	void AStarIn3DGrid();
	void HRICSRRT();
	void zoneSizeChanged();
	void resetRandomPoints();
	
	// Taskspace
	void computeWorkspacePath();
	void computeHoleMotion();
	void KDistance(double value);
	void KVisibility(double value);
	void make2DGrid();
	
	void enableHriSpace();
	void setWhichTestSlot(int test);
	void setActiveGrid(int grid);
	void mergeGrids();
	void cellToShowChanged();
#endif
	

	
private:
	QtShiva::SpinBoxSliderConnector*	m_k_distance;
	QtShiva::SpinBoxSliderConnector*	m_k_visbility;
	QtShiva::SpinBoxSliderConnector*	m_k_reachability;
	QtShiva::SpinBoxSliderConnector*	m_k_naturality;
	
	Ui::HricsWidget*					m_ui;
	
	MotionPlanner*						m_motionWidget;

	MainWindow*               m_mainWindow;
};

#endif
