/*
 *  mainwindowTestFunctions.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 04/08/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "mainwindowTestFunctions.hpp"
#include "mainwindowGenerated.hpp"

#include "planner_handler.hpp"

using namespace std;

MainWindowTestFunctions::MainWindowTestFunctions(MainWindow* MainWinPt) : m_mainWindow(MainWinPt)
{
  cout << "Init test Functions" << endl;
  connect(this, SIGNAL(selectedPlanner(QString)),
          global_plannerHandler, SLOT(startPlanner(QString)));
  
	connect(m_mainWindow->Ui()->pushButtonTest1,SIGNAL(clicked()),this,SLOT(test1()));
	connect(m_mainWindow->Ui()->pushButtonTest2,SIGNAL(clicked()),this,SLOT(test2()));
	connect(m_mainWindow->Ui()->pushButtonTest3,SIGNAL(clicked()),this,SLOT(test3()));	
}

void MainWindowTestFunctions::test1()
{
  cout << "------------------- test1 -------------------" << endl;
	cout << "---------------------------------------------" << endl;
  emit(selectedPlanner(QString("test1")));
}

void MainWindowTestFunctions::test2()
{
	using namespace std;
	
	cout << "------------------- test2 -------------------" << endl;
	cout << "---------------------------------------------" << endl;
  emit(selectedPlanner(QString("test2")));
}

void MainWindowTestFunctions::test3()
{
  cout << "------------------- test3 -------------------" << endl;
	cout << "---------------------------------------------" << endl;
  emit(selectedPlanner(QString("test3")));
}
