#include "main-remote.hpp"

#include "mainwindow-remote.hpp"
#include "ui_mainwindow-remote.h"

#include "planner_handler.hpp"

#include "API/scene.hpp"
#include "API/project.hpp"

#include <iostream>
#include <QDesktopWidget>


QSemaphore* sem;
GLWidget* openGlWidget;


#include "API/Graphic/drawModule.hpp"





extern int mainMhp(int argc, char** argv);

using namespace std;


PlannerHandler* global_plannerHandler(NULL);
MainWindowRemote* global_w(NULL);
QThread* global_PlanningThread(NULL);

void draw_opengl()
{

  if(global_w != NULL)
  {

    QMetaObject::invokeMethod(global_w->getOpenGL(),
                              "myPaintGL",
                              Qt::BlockingQueuedConnection);
  }
}


/**
 * @ingroup qtWindow
 */
Simple_threads::Simple_threads()
{
        sem = new QSemaphore(0);
}

Simple_threads::~Simple_threads()
{

}

int Simple_threads::run(int argc, char** argv)
{
    app = new QApplication(argc, argv);
    app->setWindowIcon(QIcon(QPixmap(molecule_xpm)));


    mainMhp(argc, argv);

    // Creates the wrapper to the project
    // Be carefull to initialize in the right thread
    global_Project = new Project(new Scene(XYZ_ENV));

    MainWindowRemote w;
    global_w = &w;

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

    w.show();
   // w.showMaximized();
    w.raise();

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
