/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
#ifndef MAIN_HPP
#define MAIN_HPP

#if defined( QT_LIBRARY ) && defined( MOVE3D_CORE )
#include "qtLibrary.hpp"
#endif

#if defined( QT_GL ) && defined( MOVE3D_CORE )
#include "qtOpenGL/qtGLWindow.hpp"
#endif

#include <QtCore/QString>

#ifdef QT_GL
/**
 * @ingroup qtWindow
 * @brief Main application with the QT_WidgetMain double thread class (X-Forms Thread)
 */
class Main_threads: public QObject
{
  Q_OBJECT
  
  qtGLWindow* 	g3dWin;
  QApplication*       app;
  QCoreApplication* 	coreApp;
  
public:
  Main_threads();
  ~Main_threads();
  
  int run(int argc, char** argv);
  
public slots:
  void selectPlanner();
  void initInterface();
  void loadSettings();
  
signals:
  void selectedPlanner(QString);
  
  private slots :
  void exit();  
};

/**
 * @ingroup qtWindow
 * @brief Simple OpenGl display with Qt
 */
class Simple_threads : public QObject
{
	Q_OBJECT

	GLWidget* 	m_simpleOpenGlWidget;

	QApplication* 	app;
	
public:
	Simple_threads();
	~Simple_threads();
	
  int run(int argc, char** argv);

	
	private slots :
};

#endif

#endif // MAIN_HPP
