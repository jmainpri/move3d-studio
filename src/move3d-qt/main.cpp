#ifdef QT_UI_XML_FILES
#include "qtMainInterface/mainwindow.hpp"
#include "qtMainInterface/mainwindowGenerated.hpp"
#include "qtMainInterface/settings.hpp"
#endif

#include "main.hpp"
#include "planner_handler.hpp"

#include "API/scene.hpp"
#include "API/project.hpp"

#include <iostream>
#include <QDesktopWidget>
#include <QFileDialog>

#ifdef QT_GL
QSemaphore* sem;
GLWidget* openGlWidget;
#endif

#include "API/Graphic/drawModule.hpp"

//#ifdef USE_GLUT
//#include "glutWindow.hpp"
//#include <glut.h>
//#endif

extern int mainMhp(int argc, char** argv);
extern bool move3d_studio_load_settings;

using namespace std;

#ifdef QT_GL
/**
 * @ingroup qtWindow
 * @brief Main application with the QT_WidgetMain double thread class (X-Forms Thread)
 */
Main_threads::Main_threads()
{

}

Main_threads::~Main_threads()
{
	
}

// Temporary mechanism to redraw the opengl scene.
// Not elegant, but it works.
MainWindow* global_w(NULL);
QThread* global_PlanningThread(NULL);
void draw_opengl()
{
  if (global_PlanningThread != QThread::currentThread() ) 
  {
    cout << "Warning Draw Outside of Planning thread" << endl;
  }
  
  if(global_w != NULL)
  {
    QMetaObject::invokeMethod(global_w->getOpenGL(),
			      "myPaintGL",
			      Qt::BlockingQueuedConnection);
  }
}

PlannerHandler* global_plannerHandler(NULL);

/*
if(UI)
{
  app = new QApplication(argc, argv);
  coreApp = app;
  app->setWindowIcon(QIcon(QPixmap(molecule_xpm)));
}
else
{
  coreApp = new QCoreApplication(argc, argv);
}

// transform the std::vector<char*> into the suitable type : char**
char** prunedArgvPtr = new char*[prunedArgv.size()];
for(unsigned i(0); i < prunedArgv.size(); i++)
{
  prunedArgvPtr[i] = prunedArgv[i];
}

mPlannerThread = new PlannerThread(prunedArgc, prunedArgvPtr);
mPlannerThread->setScript(script);
// no UI
 if(!UI)
  {
  connect(mPlannerThread, SIGNAL(done()), this, SLOT(exit()));
  mPlannerThread->start();
  std::cout << "Running script " << script.toStdString() << std::endl;
  QMetaObject::invokeMethod(mPlannerThread,"script",Qt::QueuedConnection,Q_ARG(QString, script));
  }
  // UI
  else {
    connect(mPlannerThread, SIGNAL(initialized()),this, SLOT(initInterface()));
    mPlannerThread->start();
  }
  
  return coreApp->exec();
  }
*/

void Main_threads::loadSettings()
{
  char* home_path = getenv("HOME_MOVE3D");
  if( home_path == NULL) {
    cout << "HOME_MOVE3D is not defined" << endl;
    return;
  }
  
  string home(home_path);
  string fileName("/.save_interface_params");
  if (!home.empty()) {
    qt_loadInterfaceParameters( false, home+fileName, false ); // OpenGL -> false
    cout << "Loading parameters at : " << home+fileName << endl;
    cout << "quick load succeded" << endl;
  }
  else {
    cout << "Error : HOME_MOVE3D is not defined" << endl;
  }
}

void Main_threads::initInterface()
{
#ifdef QT_UI_XML_FILES
  MainWindow* w = new MainWindow();
	global_w = w;
  
	// Start
	connect( w, SIGNAL(runClicked()), this, SLOT(selectPlanner()));
	connect( w, SIGNAL(runClicked()), w, SLOT(enableStopButton()));
	connect( this, SIGNAL(selectedPlanner(QString)),global_plannerHandler, SLOT(startPlanner(QString)));
	// Stop
	connect( w, SIGNAL(stopClicked()), global_plannerHandler, SLOT(stopPlanner()), Qt::DirectConnection);
	connect( global_plannerHandler, SIGNAL(plannerIsStopped()), w, SLOT(enableRunAndResetButtons()));
	connect( global_plannerHandler, SIGNAL(plannerIsStopped()), w, SLOT(drawAllWinActive()));
	// Reset
	connect( w, SIGNAL(resetClicked()), global_plannerHandler, SLOT(resetPlanner()));
	connect( global_plannerHandler, SIGNAL(plannerIsReset()), w, SLOT(enableRunButton()));
	connect( global_plannerHandler, SIGNAL(plannerIsReset()), w, SLOT(drawAllWinActive()));
	//  w.showMaximized();
  
 	QRect g = QApplication::desktop()->screenGeometry();
 	cout << " x = " << g.x() << " y = " << g.y() << endl;
 	cout << " width = " << g.width() << " height = " << g.height() << endl;
 	
 	QRect g_window = w->geometry();
 	g_window.setWidth( g.width() );
 	g_window.setHeight( 0.707*g.height() ); // sqrt(2) / 2
 	g_window.moveTo( 0, 0 );
  
  if( move3d_studio_load_settings )
  {
    w->loadParametersQuick();
    
    if( ENV.getBool(Env::isCostSpace) ) {
      w->Ui()->tabCost->initCostSpace();
    }
  }
	
  qt_init_after_params();
  
  w->refreshConstraintedDoFs();
 	w->setGeometry( g_window );
	w->show();
	w->raise();
#endif
}

int Main_threads::run(int argc, char** argv)
{
  int argc_tmp;
  char** argv_tmp=NULL;
  int ith_arg=0;
  bool openFileDialog=true;
  bool noGui=false;
  string dirName;
  
  // Find if a file is passed as argument and set openFileDialog mode
  // Also check for nogui mode
  while (ith_arg < argc)
  {
    if (string(argv[ith_arg]) == "-nogui") 
    {
      noGui = true;
    }
    if (string(argv[ith_arg]) == "-d") 
    {
      openFileDialog = true;
      if ((ith_arg+1) < argc) {
        dirName = argv[ith_arg+1];
      } 
      else {
        return 0;
      }
    }
    if (string(argv[ith_arg]) == "-f") 
    {
      openFileDialog = false;
      break;
    }
    
    ith_arg++;
  }
  
  // The no gui mode can start the application on a distant
  // machine whith out recompiling
  if(noGui) {
    coreApp = new QCoreApplication(argc, argv);
  }
  else {
    app = new QApplication(argc, argv);
    app->setWindowIcon(QIcon(QPixmap(molecule_xpm)));
    coreApp = app;
    //    app->setStyle(new QCleanlooksStyle());
    //    app->setStyle(new QWindowsStyle());
    //    app->setStyle(new QMacStyle());
  }
  
  // No argument (load a file from disc)
  if( ( argc == 1 || openFileDialog ) && (!noGui) )
  {
    QString fileName = QFileDialog::getOpenFileName( NULL, tr("Open P3D File"), dirName.c_str(),tr("P3D (*.p3d)"));
    
    if (!fileName.isEmpty())
    {
      // Copy arguments after the fileName
      argc_tmp = argc+2;
      argv_tmp = new char*[argc+2];
      
      argv_tmp[0] = argv[0];
      argv_tmp[1] = new char[string("-f").length()+1];
      argv_tmp[2] = new char[fileName.toStdString().length()+1];
      
      strcpy( argv_tmp[1] , QString("-f").toAscii().data() );
      strcpy( argv_tmp[2] , fileName.toAscii().data() );
      
      if (argc>1) 
      {
        for(int i=1;i<argc;i++)
        {
          string str(argv[i]);
          argv_tmp[i+2] = new char[str.length()+1];
          strcpy( argv_tmp[i+2] , str.c_str() );
        }
      }
      cout << "Openning file : " << fileName.toStdString() << endl;
    }
    else {
      return 0;
    }
  }
  else {
    argc_tmp = argc;
    argv_tmp = argv;
  }
  
  // Create planner thread then initialize environment
	QThread plannerThread;
  global_PlanningThread = &plannerThread;
	global_plannerHandler = new PlannerHandler(argc_tmp, argv_tmp);
	global_plannerHandler->moveToThread(&plannerThread);

  if(noGui) {
    connect(global_plannerHandler, SIGNAL(plannerIsStopped()), this, SLOT(exit()));
    global_PlanningThread->start();
    
    QMetaObject::invokeMethod(global_plannerHandler,"init",Qt::BlockingQueuedConnection);
    if( move3d_studio_load_settings ) loadSettings();
    
    // Creates the wrapper to the project, be carefull to initialize in the right thread
    global_Project = new Project(new Scene(XYZ_ENV));
    QString script("Diffusion");
    ENV.setBool(Env::drawDisabled,true);
    QMetaObject::invokeMethod(global_plannerHandler,"startPlanner",Qt::QueuedConnection,Q_ARG(QString, script));
  }
  else {
    global_PlanningThread->start();
    QMetaObject::invokeMethod(global_plannerHandler,"init",Qt::BlockingQueuedConnection);
    
    // Creates the wrapper to the project, be carefull to initialize in the right thread
    global_Project = new Project(new Scene(XYZ_ENV));
    initInterface();
  }
  
  //	while (true) {
  //		app->processEvents();
  //	}
  return coreApp->exec();
}

void Main_threads::selectPlanner()
{
  if(ENV.getBool(Env::isPRMvsDiffusion))
  {
    emit(selectedPlanner(QString("PRM")));
  }
  else
  {
    emit(selectedPlanner(QString("Diffusion")));
  }
}

void Main_threads::exit()
{
	cout << "Ends all threads" << endl;
	app->quit();
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
	
	//----------------------------------------------------------------------
	// OpenGl Widget
	//----------------------------------------------------------------------
	m_simpleOpenGlWidget = new GLWidget(NULL);
	m_simpleOpenGlWidget->setObjectName(QString::fromUtf8("OpenGL"));
	
// 	QRect g = QApplication::desktop()->screenGeometry();
// 	cout << " x = " << g.x() << " y = " << g.y() << endl;
// 	cout << " width = " << g.width() << " height = " << g.height() << endl;
 	
// 	QRect g_window = w.geometry();
// 	g_window.setWidth( g.width() );
// 	g_window.setHeight( 0.707*g.height() ); // sqrt(2) / 2
// 	g_window.moveTo( 0, 0 );
	
	m_simpleOpenGlWidget->showMaximized();
	m_simpleOpenGlWidget->raise();
	
	return app->exec();
	
//	while (true) {
//		app->processEvents();
//	}
}
#else

// Doesn't draw the opengl display
void draw_opengl()
{
  
}

#endif
/**
 * @ingroup qtWindow
 * @brief Main function of Move3D
 */
int main(int argc, char *argv[])
{
	
	enum DisplayMode 
	{
		MainMHP,
		qtWindow,
		Glut,
		simpleGlWidget,
	} 
  mode;
	
	mode = qtWindow;
	
	switch (mode) 
	{
    case Glut:
		{
//#ifdef USE_GLUT
//			GlutWindowDisplay win(argc,argv);
//			glutMainLoop ();
//#else
//			cout << "Error : Glut is not linked" << endl;
//#endif
		}
#ifdef QT_GL
    case qtWindow:
		{
			Main_threads main;
			//cout << "main.run(argc, argv)"  << endl;
			return main.run(argc, argv);
		}
			
		case simpleGlWidget:
		{
			Simple_threads main;
			return main.run(argc, argv);
		}
#endif
		case MainMHP:
		{
 			return mainMhp(argc, argv);
		}
		default:
      cout << "No main define in : int main(int argc, char *argv[])"<< endl ;
			break;
	}
}
