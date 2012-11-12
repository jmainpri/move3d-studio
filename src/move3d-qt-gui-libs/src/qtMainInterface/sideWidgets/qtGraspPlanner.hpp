//
//  qtGraspPlanner.hpp
//  move3d-studio
//
//  Created by Jim Mainprice on 12/11/12.
//  Copyright (c) 2012 LAAS-CNRS. All rights reserved.
//

#ifndef move3d_studio_qtGraspPlanner_hpp
#define move3d_studio_qtGraspPlanner_hpp

#include "qtLibrary.hpp"
#include "qtMainInterface/mainwindow.hpp"
#include "qtMainInterface/sideWidgets/qtMotionPlanner.hpp"

#include <string>

namespace Ui
{
	class GraspPlannerWidget;
};

/**
 * @ingroup qtMainWindow
 * @brief Qt Grasp Planner
 */
class GraspPlannerWidget : public QWidget
{
	Q_OBJECT
	
public:
	GraspPlannerWidget(QWidget *parent = 0);
	~GraspPlannerWidget();
  
  void initGraspPlanner();
  
	void setMainWindow(MainWindow *ptrMW) 
  { 
    m_mainWindow = ptrMW; 
  }
	
private slots:
  void showGrasp();
  
signals:
  void selectedPlanner(QString);
  
private:
	Ui::GraspPlannerWidget*		m_ui;
	
	MainWindow*			m_mainWindow;
	
};

#endif
