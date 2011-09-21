//
//  qtNatural.h
//  move3d-studio
//
//  Created by Jim Mainprice on 21/09/11.
//  Copyright 2011 LAAS/CNRS. All rights reserved.
//

#ifndef QT_NATURAL_H
#define QT_NATURAL_H

#include "qtLibrary.hpp"
#include "qtMainInterface/mainwindow.hpp"
#include "API/Grids/gridsAPI.hpp"

namespace Ui
{
	class NaturalWidget;
};

/**
 * @ingroup qtMainWindow
 * @brief Qt Motion Planner
 */
class NaturalWidget : public QWidget
{
	Q_OBJECT
	
public:
	NaturalWidget(QWidget *parent = 0);
	~NaturalWidget();

	//void initCost();
  void initNatural();
  void initNaturalSpace();
	void initHumanLike();
  void initGrids();

	void setMainWindow(MainWindow *ptrMW) { m_mainWindow = ptrMW; }
	//void setMotionWidget(MotionPlanner* ptrMLPW) { m_motionWidget = ptrMLPW; }
	
	Ui::NaturalWidget* Ui() { return m_ui; }
	
  
private slots:
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

	void enableHriSpace();
	void setWhichTestSlot(int test);
	void setActiveGrid(int grid);
	void mergeGrids();
	void cellToShowChanged();
	
private:
	Ui::NaturalWidget*					m_ui;
	//MotionPlanner*						m_motionWidget;
	MainWindow*               m_mainWindow;
};

#endif