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
#include "mainwindow-remote.hpp"
#include "ui_mainwindow-remote.h"
//#include "qtOpenGL/glwidget.hpp"

#include "posterreader.hpp"

#include "qtMainInterface/settings.hpp"
#include "qtBase/SpinBoxSliderConnector_p.hpp"
#include <iostream>
#include <vector>
#include <QPainter>
#include <QSettings>
#include <QMessageBox>
#include "sparkwidget.hpp"
#include "softmotionwidget.hpp"

#include "planner_handler.hpp"
#include "move3d-gui.h"
#include "API/scene.hpp"
#include "API/project.hpp"
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"

#ifdef ATTENTIONAL_REMOTE
#include "attentionalwidget.hpp"
#endif


using namespace std;

extern sparkWidget* global_w;


MainWindowRemote::MainWindowRemote(PosterReader *pr, QWidget *parent) :
  QMainWindow(parent),
  m_pr(pr),
  m_ui(new Ui::MainWindowRemote)
{
  //    QTimer *timer = new QTimer(this);
  //    connect(timer, SIGNAL(timeout()), this, SLOT(looptest()));
  //    timer->start(100);

  m_ui->setupUi(this);


  m_ui->dockParamWidget->setAndConnectPosterReader(pr);
  m_ui->spark->init(pr, m_ui->dockParamWidget->m_ui);
  m_ui->dockSoftMotion->init(pr, m_ui->dockParamWidget->m_ui);
  m_ui->dockCameras->init(pr, m_ui->dockParamWidget->m_ui);
  m_ui->dockNiut->init(pr, m_ui->dockParamWidget->m_ui);

#ifdef ATTENTIONAL_REMOTE
  dockWidgetAttentional = new QDockWidget(this);
  dockWidgetAttentional->setObjectName(QString::fromUtf8("dockWidgetAttentional"));
  dockAttentional = new AttentionalWidget();
  dockAttentional->setObjectName(QString::fromUtf8("dockAttentional"));
  dockWidgetAttentional->setWidget(dockAttentional);
  this->addDockWidget(static_cast<Qt::DockWidgetArea>(2), dockWidgetAttentional);
  dockAttentional->init(pr, m_ui->dockParamWidget->m_ui);
#endif

  tabifyDockWidget(m_ui->dockparamWidget,m_ui->dockSoftMotionWidget);
  tabifyDockWidget(m_ui->dockSoftMotionWidget, m_ui->dockCameraWidget);

  /* menu robots */
  initRobotsMenu();
  this->menuBar()->setVisible(true);

  on_pushButtonLoadSettings_clicked();

  //timer for recording
  timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(saveVideoTimer()));
  
  connect(m_ui->actionLoadFromFile,SIGNAL(triggered()),this,SLOT(loadInterfaceParameters()));
  connect(m_ui->actionSaveToFile,SIGNAL(triggered()),this,SLOT(saveInterfaceParameters()));
  connect(m_ui->actionLoadQuick,SIGNAL(triggered()),this,SLOT(loadParametersQuick()));
  connect(m_ui->actionSaveQuick,SIGNAL(triggered()),this,SLOT(saveParametersQuick()));

  //connect(m_ui->pushButtonSaveSettings,SIGNAL(toggled(bool)), this, SLOT(on_pushButtonSaveVideo_toggled(bool)));
}


MainWindowRemote::~MainWindowRemote()
{
  delete m_pr;
}

void MainWindowRemote::initRobotsMenu()
{
  QActionGroup* robotActionGroup = new QActionGroup(this);

  for (int i=0; i<XYZ_ENV->nr; i++)
  {
    QString name(XYZ_ENV->robot[i]->name);
    QAction* robotAction = new QAction(name,this);
    robotAction->setCheckable(true);
    robotAction->setActionGroup(robotActionGroup);
    m_RobotsInMenu.push_back( robotAction );
    connect(robotAction,SIGNAL(triggered()),this,SLOT(setRobotAsCurrent()));
    m_ui->menuRobot->addAction(robotAction);
  }

  for (int i=0; i<XYZ_ENV->nr; i++)
  {
    if( ((p3d_rob *) p3d_get_desc_curid(P3D_ROBOT)) == XYZ_ENV->robot[i] )
    {
      m_RobotsInMenu[i]->setChecked ( true );
      break;
    }
  }
}

void MainWindowRemote::setRobotAsCurrent()
{
  Scene* sc = global_Project->getActiveScene();

  for (int i=0; i<XYZ_ENV->nr; i++)
  {
    if( m_RobotsInMenu[i]->isChecked() )
    {
      sc->setActiveRobot( XYZ_ENV->robot[i]->name );
      global_ActiveRobotName = XYZ_ENV->robot[i]->name;
    }
  }
}


//void MainWindowRemote::keyPressEvent(QKeyEvent *event)
//{
//    //    cout << "Key pressed" << endl;
//    switch(event->key())
//    {
//    case Qt::Key_X:
//        mouse_mode = 1;
//        //cout << "Switch to second" << endl;
//        break;
//
//    case Qt::Key_C:
//        mouse_mode = 2;
//        //cout << "Switch to third" << endl;
//        break;
//    }
//}

//void MainWindowRemote::keyReleaseEvent(QKeyEvent *e)
//{
//
//
//}



void MainWindowRemote::initCamera()
{

  // WebBrowser example
  // view = new QWebView(parent);
  // view->load(QUrl("http://www.google.fr";));
  // view->show();
}

void MainWindowRemote::closeEvent(QCloseEvent *event)
{
  if (userReallyWantsToQuit()) {
    on_pushButtonSaveSettings_clicked();
    event->accept();
  } else {
    event->ignore();
  }
}

bool MainWindowRemote::userReallyWantsToQuit()
{

  switch( QMessageBox::information( this, "move3d-remote",
                                    "Do you want to save settings it before exiting?",
                                    "&Save", "&Don't Save", "&Cancel",
                                    0,      // Enter == button Save 0
                                    2 ) ) { // Escape == button Cancel
  case 0: // Save clicked, Alt-S or Enter pressed.
    on_pushButtonSaveSettings_clicked();
    return true;
    break;
  case 1: // Don't Save clicked or Alt-D pressed
    return true;
    break;
  case 2: // Cancel clicked, Alt-C or Escape pressed
    return false;
    break;
  }
  
  return true;
}

bool MainWindowRemote::userWantsToLoadSettings()
{

  switch( QMessageBox::information( this, "move3d-remote",
                                    "Do you want to load your settings ?",
                                    "&Yes", "&Maybe", "&No",
                                    0,      // Enter == button Save 0
                                    2 ) ) { // Escape == button Cancel
  case 0: // Save clicked, Alt-S or Enter pressed.
    on_pushButtonLoadSettings_clicked();
    return true;
    break;
  case 1: // Don't Save clicked or Alt-D pressed
    userWantsToLoadSettings();
    return true;
    break;
  case 2: // Cancel clicked, Alt-C or Escape pressed
    return false;
    break;
  }
  
  return true;
}



void MainWindowRemote::on_pushButtonLoadSettings_clicked()
{ 
  string file, home;
  char * homec;
  homec = getenv ("HOME");
  home = homec;
  file = home + "/.move3d-remote";
  QSettings settings(QString(file.c_str()), QSettings::IniFormat, this);
  //    loadDockSettings(settings, QString("dockSpark"), m_ui->dockSpark);
  //    loadDockSettings(settings, QString("dockViewer"), m_ui->dockViewer);
  //    loadDockSettings(settings, QString("dockCamera"), m_ui->dockCamera);
  //    loadDockSettings(settings, QString("dockNiut"), m_ui->dockNiut);
  //
  //    m_ui->dockSpark->showFullScreen();
}



//void MainWindowRemote::loadDockSettings(QSettings & settings, QString dockName, QDockWidget * dockWidget)
//{
//    settings.beginGroup(dockName);
//    dockWidget->setFloating(settings.value(dockName + QString("/isFloating"), false).toBool());
//    dockWidget->move(settings.value(dockName + QString("/pos"), QPoint(1,1)).toPoint());
//    dockWidget->resize(settings.value(dockName + QString("/size"), QSize(640, 480)).toSize());
//    //dockWidget->restoreGeometry(settings.value(dockName + QString("/geometry")).toByteArray());
//    addDockWidget((Qt::DockWidgetArea)settings.value(dockName + QString("/dockarea"), Qt::RightDockWidgetArea).toInt(), dockWidget);
//    settings.endGroup();
//}

void MainWindowRemote::on_pushButtonSaveSettings_clicked()
{
  //    string file, home;
  //    char * homec;
  //    homec = getenv ("HOME");
  //    home = homec;
  //    file = home + "/.move3d-remote";
  //    QSettings settings(QString(file.c_str()), QSettings::IniFormat, this);
  //    saveDockSettings(settings, QString("dockSpark"), m_ui->dockSpark);
  //    saveDockSettings(settings, QString("dockViewer"), m_ui->dockViewer);
  //    saveDockSettings(settings, QString("dockCamera"), m_ui->dockCamera);
  //    saveDockSettings(settings, QString("dockNiut"), m_ui->dockNiut);
}

//void MainWindowRemote::saveDockSettings(QSettings & settings, QString dockName, QDockWidget * dockWidget)
//{
//    settings.beginGroup(dockName);
//    settings.setValue(dockName + QString("/dockarea"), dockWidgetArea(dockWidget));
//    settings.setValue(dockName +  QString("/isFloating"), dockWidget->isFloating());
//   // settings.setValue(dockName + QString("/geometry"), dockWidget->saveGeometry());
//    settings.setValue(dockName + QString("/size"), dockWidget->size());
//    settings.setValue(dockName + QString("/pos"), dockWidget->pos());
//    settings.endGroup();
//}

//void MainWindowRemote::on_switchSparkView_clicked(bool checked)
//{
//
//}

void MainWindowRemote::loadInterfaceParameters()
{
	QString fileName = QFileDialog::getOpenFileName(this);
	
	if (!fileName.isEmpty())
	{
		qt_loadInterfaceParameters( true, fileName.toStdString() );
		cout << "Loading parameters at : " << fileName.toStdString() << endl;
		global_w->getOpenGL()->updateGL();
	}
}

void MainWindowRemote::saveInterfaceParameters()
{
  QString fileName = QFileDialog::getSaveFileName(this);
	
	if (!fileName.isEmpty())
	{
    //    if( remove( (home+fileName).c_str() ) != 0 )
    //    {
    //      cout << "Error deleting file" << endl;
    //      return;
    //    }
    
		qt_saveInterfaceParameters( true, fileName.toStdString() );
		cout << "Saving parameters at : " << fileName.toStdString() << endl;
		global_w->getOpenGL()->updateGL();
	}
}

void MainWindowRemote::loadParametersQuick()
{
  char* home_path = getenv("HOME_MOVE3D");
  
  if( home_path == NULL)
  {
    cout << "HOME_MOVE3D is not defined" << endl;
    return;
  }
  
  string home(home_path);
  string fileName("/.save_interface_params");
  
  if (!home.empty())
  {
    qt_loadInterfaceParameters( false, home+fileName );
    cout << "Loading parameters at : " << home+fileName << endl;
    cout << "quick load succeded" << endl;
    global_w->getOpenGL()->updateGL();
  }
  else
  {
    cout << "Error : HOME_MOVE3D is not defined" << endl;
  }
}

void MainWindowRemote::saveParametersQuick()
{
  char* home_path = getenv("HOME_MOVE3D");
  
  if( home_path == NULL)
  {
    cout << "HOME_MOVE3D is not defined" << endl;
    return;
  }
  
  string home(home_path);
  string fileName("/.save_interface_params");
  
  if (!home.empty())
  {
    if( remove( (home+fileName).c_str() ) != 0 )
    {
      cout << "Not deleting file!!!" << endl;
    }
    
    qt_saveInterfaceParameters( true, home+fileName );
    cout << "Saving parameters at : " << home+fileName << endl;
    global_w->getOpenGL()->updateGL();
  }
  else
  {
    cout << "Error : HOME_MOVE3D is not defined" << endl;
  }
}

void MainWindowRemote::on_pushButtonSaveVideo_toggled(bool checked)
{
  if (checked)
  {
    global_w->getOpenGL()->resetImageVector();
    timer->start(40);
    cout << "begin recording" << endl;
  }
  else
  {
    timer->stop();
    global_w->getOpenGL()->saveImagesToDisk();
    cout << "end recording" << endl;
  }
}

void MainWindowRemote::saveVideoTimer()
{
  global_w->getOpenGL()->addCurrentImage();
}
