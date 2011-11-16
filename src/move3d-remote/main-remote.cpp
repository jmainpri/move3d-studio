#include "tcpserver.hpp"
#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>

#include "main-remote.hpp"

#include "mainwindow-remote.hpp"

#include "sparkwidget.hpp"
#include "camerawidget.h"
#include "niutwidget.h"
#include "dockwidget.hpp"
#include "softmotionwidget.hpp"

#include "planner_handler.hpp"

#include "API/scene.hpp"
#include "API/project.hpp"
#include "API/Graphic/drawModule.hpp"

#include <iostream>
#include <QDesktopWidget>
#include <QMessageBox>
#include <QtNetwork/QNetworkProxy>

#include <libmove3d/hri/hri.h>

using namespace std;

QSemaphore* sem;

extern int mainMhp(int argc, char** argv);

// Worker thread
PlannerHandler* global_plannerHandler(NULL);
QThread* global_PlanningThread(NULL);

// Drawing widget
GLWidget* openGlWidget;
sparkWidget* global_w(NULL);
vector<sparkWidget*> openGlWidgets;

void draw_opengl()
{
  cout << "draw_opengl" << endl;
  if(global_w != NULL)
  {
    for(unsigned int i=0;i<openGlWidgets.size();i++)
    {
      cout << "Draw window : " << i << endl;
      QMetaObject::invokeMethod(openGlWidgets[i]->getOpenGL(),
                                "myPaintGL",
                                Qt::BlockingQueuedConnection);
    }
  }
}

/**
 * @ingroup qtWindow
 */
Simple_threads::Simple_threads()
{
  sem = new QSemaphore(0);
}

//! Destructor
Simple_threads::~Simple_threads()
{
  
}

//! function that initialize the remote viewer with  
//! seperated windows
void Simple_threads::initSeperatedWidgets(MainWindowRemote& w)
{
  cameraWidget* wCam = new cameraWidget(m_posterHandler, w.m_ui);
  niutWidget* wNiut = new niutWidget(m_posterHandler, w.m_ui);
  //
  sparkWidget* wSpark1 = new sparkWidget(m_posterHandler, w.m_ui);
  //    sparkWidget wSpark2(m_posterHandler, w.m_ui);
  //    sparkWidget wSpark3(m_posterHandler, w.m_ui);

  softmotionWidget* wSoftmotion = new softmotionWidget(m_posterHandler, w.m_ui);
  
  global_w = wSpark1;
  
  openGlWidgets.push_back(wSpark1);
  //    openGlWidgets.push_back(&wSpark2);
  //    openGlWidgets.push_back(&wSpark3);
  
  //    QRect g = QApplication::desktop()->screenGeometry();
  //    cout << " x = " << g.x() << " y = " << g.y() << endl;
  //    cout << " width = " << g.width() << " height = " << g.height() << endl;
  //
  //    QRect g_window = w.geometry();
  //    g_window.setWidth( g.width() );
  //    g_window.setHeight( 0.707*g.height() ); // sqrt(2) / 2
  //    g_window.moveTo( 0, 0 );
  //
  //    w.setGeometry( g_window );
  
  wCam->show();
  wNiut->show();
  wSpark1->show();
  wSoftmotion->show();
  //    wSpark2.show();
  //    wSpark3.show();
  
  w.show();
}

//! function that initialize the remote viewer with  
//! docked windows
void Simple_threads::initDockWidget(MainWindowRemote& w)
{
  DockWindow* severalWidgets = new DockWindow(m_posterHandler, w.m_ui);
  severalWidgets->showMaximized();
  w.show();
}

//! Main application function of the  
//! remote viewer
int Simple_threads::run(int argc, char** argv)
{
  app = new QApplication(argc, argv);
  app->setWindowIcon(QIcon(QPixmap(molecule_xpm)));
  
  mainMhp(argc, argv);
  
  // Init agents for display functions
  GLOBAL_AGENTS = hri_create_agents();
  
  // Creates the wrapper to the project
  // Be carefull to initialize in the right thread
  global_Project = new Project(new Scene(XYZ_ENV));
  
  TcpServer server;
  
  m_posterHandler = new PosterReader();
  m_posterHandler->init();
  
  MainWindowRemote w(m_posterHandler);
  
  const bool several_widgets = true;
  
  if(several_widgets)
  {
    cout << "Init Seperated" << endl;
    initSeperatedWidgets(w);
  }
  else
  {
    cout << "Init Dock" << endl;
    initDockWidget(w);
  }
  
  return app->exec();
}

/**
 * @ingroup qtWindow
 * @brief Main function of Move3D
 */
int main(int argc, char *argv[])
{    
  Simple_threads main;
  return main.run(argc, argv);
}
