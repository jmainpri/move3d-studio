/*
 *  mainwindowTestFunctions.hpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 04/08/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */
#include "qtMainInterface/mainwindow.hpp"

#ifndef MAIN_WINDOW_TEST_FUNCTIONS
#define MAIN_WINDOW_TEST_FUNCTIONS

#include <QtCore/QObject>

class MainWindowTestFunctions : public QObject
{
	
Q_OBJECT
	
public:
	MainWindowTestFunctions(MainWindow* MainWinPt);

public slots:
	void test1();
	void test2();
	void test3();
	
signals:
  void selectedPlanner(QString);

private:
	MainWindow* m_mainWindow;
};
#endif