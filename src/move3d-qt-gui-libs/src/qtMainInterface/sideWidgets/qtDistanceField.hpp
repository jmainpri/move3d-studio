/*
 *  qtReplanning.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef QT_DISTFIELD_H
#define QT_DISTFIELD_H

#include "qtLibrary.hpp"
#include "qtMainInterface/mainwindow.hpp"
#include "qtMainInterface/sideWidgets/qtMotionPlanner.hpp"

#include <string>

#ifdef USE_QWT
#include "qtPlot/basicPlotWindow.hpp"
#endif

namespace Ui
{
	class DistFieldWidget;
};

/**
 * @ingroup qtMainWindow
 * @brief Qt Motion Planner
 */
class DistFieldWidget : public QWidget
{
	Q_OBJECT
	
public:
	DistFieldWidget(QWidget *parent = 0);
	~DistFieldWidget();
	
	void initDistField();
	void setMainWindow(MainWindow *ptrMW) 
  { 
    m_mainWindow = ptrMW; 
  }
	
public slots:

	
private slots:
  void createDistanceField();
  void addAllPointsToField();
  void deleteDistanceField();
  void generateRobotBoundingVolumes();
	
signals:
  void selectedPlanner(QString);
  
private:
	Ui::DistFieldWidget*		m_ui;
	
	MainWindow*			m_mainWindow;
	
};

#endif
