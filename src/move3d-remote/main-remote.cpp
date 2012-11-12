#include "tcpserver.hpp"
#include <QtNetwork/QTcpServer>
#include <QtNetwork/QTcpSocket>

#include "main-remote.hpp"

#include "mainwindow-remote.hpp"
#include "ui_mainwindow-remote.h"


#include "sparkwidget.hpp"
#include "camerawidget.h"
#include "niutwidget.h"
#include "softmotionwidget.hpp"
#include "posterreader.hpp"

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
extern bool move3d_studio_load_settings;

extern bool drawTrajOnRemote;
extern bool drawMonitoringSpheresOnRemote;

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

void draw_remote_main()
{
    if (drawTrajOnRemote)
        draw_smtraj_tace();

    if (drawMonitoringSpheresOnRemote)
        draw_monitoring_spheres();

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
void Simple_threads::initWidgets(MainWindowRemote& w)
{

  global_w = w.m_ui->spark;
  ext_g3d_draw_remote = draw_remote_main;
  
  //openGlWidgets.push_back(wSpark1);
  openGlWidgets.push_back(w.m_ui->spark);
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
  w.showMaximized();
//  w.maximize();
  
  if( move3d_studio_load_settings )
  {
    w.loadParametersQuick();
  }
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
  
  initWidgets(w);

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
