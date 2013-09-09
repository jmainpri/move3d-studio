#ifdef QT_UI_XML_FILES
#include "qtMainInterface/mainwindow.hpp"
#include "qtMainInterface/mainwindowGenerated.hpp"
#include "qtMainInterface/settings.hpp"
#include "qtMainInterface/guiparams.hpp"
#endif

#include "main.hpp"
#include "planner_handler.hpp"

#include "API/scene.hpp"
#include "API/project.hpp"

#include <iostream>
#include <string>
#include <QDesktopWidget>
#include <QFileDialog>

#include <mcheck.h>

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
extern std::string move3d_studio_settings_file;

using namespace std;

/**
 * @ingroup qtWindow
 * @brief Logo
 */
static char* molecule_xpm[] = {
    (char*) "32 32 147 2",
    (char*) "  	g #FFFFFF",
    (char*) ". 	g #F7F7F7",
    (char*) "+ 	g #EBEBEB",
    (char*) "@ 	g #E6E6E6",
    (char*) "# 	g #E5E5E5",
    (char*) "$ 	g #E9E9E9",
    (char*) "% 	g #F4F4F4",
    (char*) "& 	g #F3F3F3",
    (char*) "* 	g #E4E4E4",
    (char*) "= 	g #D3D3D3",
    (char*) "- 	g #8C8C8C",
    (char*) "; 	g #868686",
    (char*) "> 	g #C9C9C9",
    (char*) ", 	g #EEEEEE",
    (char*) "' 	g #FBFBFB",
    (char*) ") 	g #BBBBBB",
    (char*) "! 	g #101010",
    (char*) "~ 	g #000000",
    (char*) "{ 	g #060606",
    (char*) "] 	g #A1A1A1",
    (char*) "^ 	g #F6F6F6",
    (char*) "/ 	g #F2F2F2",
    (char*) "( 	g #4A4A4A",
    (char*) "_ 	g #232323",
    (char*) ": 	g #3F3F3F",
    (char*) "< 	g #161616",
    (char*) "[ 	g #EDEDED",
    (char*) "} 	g #F9F9F9",
    (char*) "| 	g #A2A2A2",
    (char*) "1 	g #030303",
    (char*) "2 	g #7E7E7E",
    (char*) "3 	g #FEFEFE",
    (char*) "4 	g #B2B2B2",
    (char*) "5 	g #2A2A2A",
    (char*) "6 	g #484848",
    (char*) "7 	g #9F9F9F",
    (char*) "8 	g #E7E7E7",
    (char*) "9 	g #F1F1F1",
    (char*) "0 	g #454545",
    (char*) "a 	g #CECECE",
    (char*) "b 	g #EFEFEF",
    (char*) "c 	g #FDFDFD",
    (char*) "d 	g #D6D6D6",
    (char*) "e 	g #898989",
    (char*) "f 	g #797979",
    (char*) "g 	g #B5B5B5",
    (char*) "h 	g #EAEAEA",
    (char*) "i 	g #2D2D2D",
    (char*) "j 	g #FCFCFC",
    (char*) "k 	g #171717",
    (char*) "l 	g #808080",
    (char*) "m 	g #E3E3E3",
    (char*) "n 	g #303030",
    (char*) "o 	g #F5F5F5",
    (char*) "p 	g #646464",
    (char*) "q 	g #090909",
    (char*) "r 	g #DFDFDF",
    (char*) "s 	g #D9D9D9",
    (char*) "t 	g #C3C3C3",
    (char*) "u 	g #575757",
    (char*) "v 	g #5E5E5E",
    (char*) "w 	g #050505",
    (char*) "x 	g #DDDDDD",
    (char*) "y 	g #ABABAB",
    (char*) "z 	g #787878",
    (char*) "A 	g #BEBEBE",
    (char*) "B 	g #686868",
    (char*) "C 	g #F0F0F0",
    (char*) "D 	g #8F8F8F",
    (char*) "E 	g #6D6D6D",
    (char*) "F 	g #5D5D5D",
    (char*) "G 	g #9B9B9B",
    (char*) "H 	g #E8E8E8",
    (char*) "I 	g #6E6E6E",
    (char*) "J 	g #A6A6A6",
    (char*) "K 	g #E2E2E2",
    (char*) "L 	g #717171",
    (char*) "M 	g #979797",
    (char*) "N 	g #E0E0E0",
    (char*) "O 	g #656565",
    (char*) "P 	g #D5D5D5",
    (char*) "Q 	g #757575",
    (char*) "R 	g #FAFAFA",
    (char*) "S 	g #555555",
    (char*) "T 	g #323232",
    (char*) "U 	g #D8D8D8",
    (char*) "V 	g #4D4D4D",
    (char*) "W 	g #DCDCDC",
    (char*) "X 	g #6C6C6C",
    (char*) "Y 	g #BCBCBC",
    (char*) "Z 	g #2F2F2F",
    (char*) "` 	g #595959",
    (char*) " .	g #C8C8C8",
    (char*) "..	g #DEDEDE",
    (char*) "+.	g #D1D1D1",
    (char*) "@.	g #9A9A9A",
    (char*) "#.	g #696969",
    (char*) "$.	g #E1E1E1",
    (char*) "%.	g #DBDBDB",
    (char*) "&.	g #7B7B7B",
    (char*) "*.	g #3C3C3C",
    (char*) "=.	g #1E1E1E",
    (char*) "-.	g #666666",
    (char*) ";.	g #5F5F5F",
    (char*) ">.	g #939393",
    (char*) ",.	g #C4C4C4",
    (char*) "'.	g #A5A5A5",
    (char*) ").	g #191919",
    (char*) "!.	g #838383",
    (char*) "~.	g #313131",
    (char*) "{.	g #878787",
    (char*) "].	g #5A5A5A",
    (char*) "^.	g #D4D4D4",
    (char*) "/.	g #BFBFBF",
    (char*) "(.	g #8D8D8D",
    (char*) "_.	g #6B6B6B",
    (char*) ":.	g #272727",
    (char*) "<.	g #1B1B1B",
    (char*) "[.	g #1C1C1C",
    (char*) "}.	g #DADADA",
    (char*) "|.	g #8A8A8A",
    (char*) "1.	g #0C0C0C",
    (char*) "2.	g #010101",
    (char*) "3.	g #0B0B0B",
    (char*) "4.	g #292929",
    (char*) "5.	g #3B3B3B",
    (char*) "6.	g #424242",
    (char*) "7.	g #545454",
    (char*) "8.	g #4F4F4F",
    (char*) "9.	g #343434",
    (char*) "0.	g #070707",
    (char*) "a.	g #A8A8A8",
    (char*) "b.	g #D0D0D0",
    (char*) "c.	g #414141",
    (char*) "d.	g #B0B0B0",
    (char*) "e.	g #777777",
    (char*) "f.	g #585858",
    (char*) "g.	g #747474",
    (char*) "h.	g #959595",
    (char*) "i.	g #B7B7B7",
    (char*) "j.	g #F8F8F8",
    (char*) "k.	g #949494",
    (char*) "l.	g #262626",
    (char*) "m.	g #ACACAC",
    (char*) "n.	g #0A0A0A",
    (char*) "o.	g #B3B3B3",
    (char*) "p.	g #8B8B8B",
    (char*) "                                . + @ # $ %                     ",
    (char*) "                              & * = - ; > # ,                   ",
    (char*) "                            ' # ) ! ~ ~ { ] # ^                 ",
    (char*) "                            / # ( ~ ~ ~ ~ _ @ ,                 ",
    (char*) "                            / # : ~ ~ ~ ~ < @ [                 ",
    (char*) "                            } # | 1 ~ ~ ~ 2 # &     3 3         ",
    (char*) "                              , # 4 5 6 7 # $ 3 . + # * 8 9 3   ",
    (char*) "                                [ # 0 a # b c % # d e f g # h 3 ",
    (char*) "                                b @ i # +   j # > k ~ ~ ~ l * / ",
    (char*) "                              c + m n # b   o # p ~ ~ ~ ~ q r h ",
    (char*) "                      c ^ 9 , , s t u m &   o # v ~ ~ ~ ~ w x h ",
    (char*) "                    o , , , b b d y z d b   } # A w ~ ~ ~ B # C ",
    (char*) "        3         & , , % '   ' # D e d @ % $ # B p E F G # H c ",
    (char*) "  c C 8 * @ [ ' ^ , , }   3 . [ x I J r 8 s K p L # # # # , j   ",
    (char*) "j 8 * J z M N # $ , .   ' b , m d O t # K P B Q # h R R 3       ",
    (char*) ", # S ~ ~ ~ T U s , 3 c b , ^ + r V = W * X ; * h c   3         ",
    (char*) "@ Y ~ ~ ~ ~ ~ e d * R . , ^ ^ H # Z s P ` 2 d 8 } [ # * @ b j   ",
    (char*) "# g ~ ~ ~ ~ ~ v  .s * ..W H # +.@.Z d #.- $.%.9 @ N M &.J # 8 j ",
    (char*) "+ * *.~ ~ ~ =.- -.u ;.>.,...'._ 5 w ).!.x N ..* ..~.~ ~ ~ S # [ ",
    (char*) "R @ r {.].z ^.K d d /.(._.u Z :.<.~ [.E K r }.K |.~ ~ ~ ~ ~ Y @ ",
    (char*) "  R + # # # $ . h r %.K # K [.1.2.3.4.5.6.7.8.6 9.~ ~ ~ ~ ~ g # ",
    (char*) "      c R j     j [ h h # $.B 0.< 4.4.a...%.# # d =.~ ~ ~ *.m + ",
    (char*) "                  c C h H d b.c.F -.d.# $ ^ . % # ^.e.f.; r @ R ",
    (char*) "                    3 % # %.g.h.N * # C 3       . $ * # * + R   ",
    (char*) "                3 b # # # i.c.* h j.c               j } c       ",
    (char*) "              3 + # '.` ].n > * R                               ",
    (char*) "              o # - 2.~ ~ 2.k.# ^                               ",
    (char*) "              b # l.~ ~ ~ ~ n # C                               ",
    (char*) "              b # T ~ ~ ~ ~ *.# C                               ",
    (char*) "              j.* m.n.~ ~ 1.o.# }                               ",
    (char*) "                b # a e p.b.# C                                 ",
    (char*) "                  o h # # h ^                                   "};

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

    std::string home(home_path);
    std::string fileName = home + "/" + move3d_studio_settings_file;
    if (!home.empty()) {
        qt_loadInterfaceParameters( false, fileName, false ); // OpenGL -> false
        cout << "Loading parameters at : " << fileName << endl;
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

    // Sets up gui specific parameter structure
    initGuiParameters();

    if( move3d_studio_load_settings )
    {
        cout << "Load Saved Parameters!!! " << endl;
        w->loadParametersQuick();
    }

    qt_init_after_params();

    if( ENV.getBool(Env::isCostSpace) ) {
        w->Ui()->tabCost->initCostSpace();
    }

    QRect g = QApplication::desktop()->screenGeometry();
    cout << "QApplication::desktop()->screenGeometry() : ";
    cout << " x = " << g.x() << " y = " << g.y() << ", width = " << g.width() << " height = " << g.height() << endl;

    QRect g_window = w->geometry();
    // g_window.setWidth( g.width() );
    // g_window.setHeight( 0.707*g.height() ); // sqrt(2) / 2
    // g_window.moveTo( 0, 0 );

    // g_window.setWidth( 1800 );
    // g_window.setHeight( 1024 );

    // g_window.setWidth( 1500 );
    // g_window.setHeight( 700 );

    // g_window.setWidth( 1024 );
    // g_window.setHeight( 720 );

    cout << "set to : width = " << GuiEnv->getInt( GuiParam::mainwin_w ) << " height = " << GuiEnv->getInt( GuiParam::mainwin_h ) << endl;

    g_window.setWidth( GuiEnv->getInt( GuiParam::mainwin_w ) );
    g_window.setHeight( GuiEnv->getInt( GuiParam::mainwin_h ) );

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

    bool noGui=false;

    bool openFileDialog=true;
    string dirName;

    bool launch_script=false;
    string script_id;

    //mtrace();

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
        if (string(argv[ith_arg]) == "-launch")
        {
            launch_script = true;
            if ((ith_arg+1) < argc) {
                script_id = argv[ith_arg+1];
            }
            else {
                return 0;
            }
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

    if(noGui)
    {
        connect(global_plannerHandler, SIGNAL(plannerIsStopped()), this, SLOT(exit()));
        global_PlanningThread->start();

        QMetaObject::invokeMethod( global_plannerHandler, "init", Qt::BlockingQueuedConnection );

        // Creates the wrapper to the project, be carefull to initialize in the right thread
        global_Project = new Project(new Scene(XYZ_ENV));

        if( move3d_studio_load_settings )
            loadSettings();

        qt_init_after_params();

        // QString script("Diffusion");
        // QString script("MultiRRT");
        QString script( script_id.c_str() );
        ENV.setBool(Env::drawDisabled,true);
        QMetaObject::invokeMethod( global_plannerHandler, "startPlanner", Qt::QueuedConnection, Q_ARG(QString, script) );
    }
    else {
        global_PlanningThread->start();
        QMetaObject::invokeMethod(global_plannerHandler,"init",Qt::BlockingQueuedConnection);

        // Creates the wrapper to the project, be carefull to initialize in the right thread
        global_Project = new Project(new Scene(XYZ_ENV));

        cout << endl;
        cout << "  ------------------ Init Interface ------------------" << endl;
        cout << "  ----------------------------------------------------" << endl;
        initInterface();

        if( launch_script )
        {
            QString script( script_id.c_str() );
            QMetaObject::invokeMethod(global_plannerHandler,"startPlanner",Qt::QueuedConnection,Q_ARG(QString, script));
        }
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
