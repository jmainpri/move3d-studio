/*
 *  qtUtil.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 30/07/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef QT_UTIL_H
#define QT_UTIL_H

#if defined( MOVE3D_CORE )

#include "qtLibrary.hpp"
#include "qtMainInterface/mainwindow.hpp"

#endif

namespace Ui
{
	class UtilWidget;
};

/**
 * @ingroup qtMainWindow
 * @brief Qt Util Widget to be changed by user
 */
class UtilWidget : public QWidget
{
	Q_OBJECT
	
public:
	UtilWidget(QWidget *parent = 0);
	~UtilWidget();
	
	void setMainWindow(MainWindow *ptrMW) { m_mainWindow = ptrMW; }
	
	void initGreedy();
	void initRRTthreshold();
	void initDistanceAndLocalpath();
  void initMultiRRT();
	
public slots:
	
	void biasPos();
	void greedyPlan();
	void drawAllWinActive();
	void runThresholdPlanner();
	void computeRandomLP();
	
  void startMultiTRRT();
  void clearConfig();
  void saveCurrentConfigToVector();
  void setCurrentConfig(int index);
  
signals:
  void selectedPlanner(QString);
  
private:
  
	Ui::UtilWidget*					m_ui;
	MainWindow*							m_mainWindow;
	
	QPushButton* greedy;
};

/**
 * @ingroup qtWindow
 * @brief Planner thread class 
 */
class TestPlannerthread: public QThread
{
	Q_OBJECT
	
public:
	TestPlannerthread(QObject* parent = 0);
	
protected:
	void run();
	
};


#endif