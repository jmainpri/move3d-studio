//
//  qtRRTStar.hpp
//  move3d-studio
//
//  Created by Jim Mainprice on 28/07/12.
//  Copyright (c) 2012 LAAS/CNRS. All rights reserved.
//

#ifndef QT_RRTSTAR_HPP
#define QT_RRTSTAR_HPP

#include "qtLibrary.hpp"
#include "qtMainInterface/mainwindow.hpp"
#include "qtMainInterface/sideWidgets/qtMotionPlanner.hpp"

namespace Ui
{
	class RRTStarWidget;
};

/**
 * @ingroup qtMainWindow
 * @brief Qt Motion Planner
 */
class RRTStarWidget : public QWidget
{
	Q_OBJECT
	
public:
  
	RRTStarWidget(QWidget *parent = 0);
	~RRTStarWidget();
  
  void init();
  
  void setMainWindow(MainWindow *ptrMW) { m_mainWindow = ptrMW; }
  
private:
  
  MainWindow*                             m_mainWindow;
  
  Ui::RRTStarWidget*                        m_ui;
};

#endif
