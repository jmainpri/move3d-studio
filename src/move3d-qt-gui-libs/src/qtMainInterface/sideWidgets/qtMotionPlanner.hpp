/*
 *  qtMotionPlanner.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef QT_MOTIONPLANNER_H
#define QT_MOTIONPLANNER_H

#include "../p3d/env.hpp"

#include "qtLibrary.hpp"
#include "qtMainInterface/mainwindow.hpp"

#include "API/Roadmap/node.hpp"


#ifdef USE_QWT
#include "qtPlot/basicPlotWindow.hpp"
#endif

namespace Ui
{
	class MotionPlanner;
}

/**
 * @ingroup qtMainWindow
 * @brief Qt Motion Planner
 */
class MotionPlanner : public QWidget
{
	Q_OBJECT
	
public:
	MotionPlanner(QWidget *parent = 0);
	~MotionPlanner();
	
	void setMainWindow(MainWindow *ptrMW) { m_mainWindow = ptrMW; }
  
  Node* getIthNodeInActiveGraph();
	
private slots:
	// Optim -----------------------------
	void test(double value);
	void computeGrid();
	void runMultiSmooth();
	void optimizeCost();
	void shortCutCost();
	void removeRedundant();
	void extractBestTraj();
	void setCostCriterium(int choise);
	void eraseDebugTraj();
	void cutTrajInSmallLP();
  void cutTrajAndOptimizeSM();
	
	// Multi-Run -------------------------
	void saveContext();
	void printContext();
	void deleteSelected();
	void printAllContext();
	void resetContext();
	void setToSelected();
	void runAllRRT();
	void runAllGreedy();
	void showHistoWindow();
	
	// General ---------------------------
	void checkAllEdges();
	void envDmaxSpinBoxValueChanged( double dmax );
	void testParam(double param);
	
	// Show ------------------------------
	void nodeToShowChanged();
	void removeNode();
  
signals:
  void selectedPlanner(QString);
	
private:
	Ui::MotionPlanner *m_ui;
	
	MainWindow *m_mainWindow;
	
	QListWidget* contextList;
	std::vector<QListWidgetItem*> itemList;
	
#ifdef USE_QWT
	BasicPlotWindow *plot;
	HistoWindow* histoWin;
#endif
	
	void initDiffusion();
	void initMultiRRT();
	void initPRM();
	void initMultiRun();
	void initOptim();
	void initGeneral();
	void initShowGraph();
};

/**
 * @ingroup qtWindow
 * @brief Multi Planner thread class 
 */
class MultiThread: public QThread
{
	Q_OBJECT
	
public:
	MultiThread(bool isRRT,QObject* parent = 0);
	
protected:
	void run();
	bool m_isRRT;
};

#endif
