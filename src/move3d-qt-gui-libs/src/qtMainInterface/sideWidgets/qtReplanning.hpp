/*
 *  qtReplanning.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef QT_REPLANNING_H
#define QT_REPLANNING_H

#include "qtLibrary.hpp"
#include "qtMainInterface/mainwindow.hpp"
#include "qtMainInterface/sideWidgets/qtMotionPlanner.hpp"

#include <string>

#ifdef USE_QWT
#include "qtPlot/basicPlotWindow.hpp"
#endif

namespace Ui
{
	class ReplanningWidget;
};

/**
 * @ingroup qtMainWindow
 * @brief Qt Motion Planner
 */
class ReplanningWidget : public QWidget
{
	Q_OBJECT
	
public:
	ReplanningWidget(QWidget *parent = 0);
	~ReplanningWidget();
	
  void init();
	void setMainWindow(MainWindow *ptrMW) { m_mainWindow = ptrMW; }
	
public slots:
	
private slots:
  void computeHandOver();
  void mainReplanFunction();
  void runStomp();
  void runChomp();
  void setLocalpath();
  
  void initReplanning();
  void executeReplanTraj();
	
signals:
  void selectedPlanner(QString);
  
private:
	Ui::ReplanningWidget*		m_ui;
	
	MainWindow*			m_mainWindow;
	
};

#endif
