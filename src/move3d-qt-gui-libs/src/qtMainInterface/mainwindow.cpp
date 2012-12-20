#include "mainwindow.hpp"
#include "mainwindowGenerated.hpp"
#include "settings.hpp"

#include "qtBase/SpinBoxSliderConnector_p.hpp"
#include "qtOpenGL/glwidget.hpp"
#include "qtOpenGL/qtMobileCamera.h"

#include "mainwindowTestFunctions.hpp"
#include "planner_handler.hpp" 

#include "P3d-pkg.h"
#include "Util-pkg.h"

#include "move3d-headless.h"
#include "move3d-gui.h"

#include "Planner-pkg.h"

#include <iostream>
#include <tr1/memory>
#include <vector>


#include "planner/planEnvironment.hpp"
#include "planner/cost_space.hpp"

#include "API/planningAPI.hpp"
#include "API/Trajectory/trajectory.hpp"
#include "planner/TrajectoryOptim/Classic/smoothing.hpp"
// Warning contains boost function that conlicts with Qt
//#include "planner_cxx/API/Trajectory/RoboptimTrajectory.h"
#include "planner/TrajectoryOptim/Classic/costOptimization.hpp"
#include "API/Grids/GridToGraph/gridtograph.hpp"
#include "API/Search/GraphState.hpp"
#include "API/Grids/PointCloud.hpp"
#include "API/Grids/BaseGrid.hpp"
#include "API/Grids/TwoDGrid.hpp"
//#include "API/Roadmap2/BoostGraphTest.h"
#include "API/Roadmap/graphConverter.hpp"

#include "utils/testModel.hpp"
#include "utils/SaveContext.hpp"
#include "ui_qtMotionPlanner.h"

using namespace std;
using namespace tr1;

extern string global_ActiveRobotName;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), m_ui(new Ui::MainWindow)
{
    m_ui->setupUi(this);

    m_ui->tabMotionPlanner->setMainWindow(this);

    m_ui->tabCost->setMainWindow(this);
    m_ui->tabCost->setMotionWidget(this->m_ui->tabMotionPlanner);
    m_ui->tabCost->initCost();

    m_ui->tabUtil->setMainWindow(this);

#ifdef HRI_COSTSPACE
    m_ui->tabCost->getHriWidget()->setMainWindow(this);
    m_ui->tabCost->getHriWidget()->setMotionWidget(this->m_ui->tabMotionPlanner);
    m_ui->tabCost->getHriWidget()->initHRI();
    m_ui->tabCost->getOtpWidget()->setMainWindow(this);
#endif

    m_ui->tabCost->getDistFieldWidget()->setMainWindow(this);
#if defined(LIGHT_PLANNER) && defined(MULTILOCALPATH)
    m_ui->tabCost->getReplanningWidget()->setMainWindow(this);
#endif

    m_ui->tabRobot->setMainWindow(this);
    m_ui->tabRobot->initRobot();

    // m_ui->OpenGL->setWinSize(G3D_WIN->size);
    m_ui->OpenGL->setWinSize(600);
    m_ui->OpenGL->setMainWindow(this);

    mKCDpropertiesWindow = new KCDpropertiesWindow();

    m_testFunctions = new MainWindowTestFunctions(this);

    // Connect Menu slots
    connect(m_ui->actionOpenScenario,SIGNAL(triggered()),this,SLOT(openScenario()));
    connect(m_ui->actionSaveScenario,SIGNAL(triggered()),this,SLOT(saveScenario()));
    connect(m_ui->actionRobotForm,SIGNAL(triggered()),m_ui->formRobot,SLOT(show()));

    connect(m_ui->pushButtonChangeCamera,SIGNAL(clicked()),this,SLOT(changeCamera()));
    //#ifdef MULTILOCALPATH
    //	connect(m_ui->actionLocalPathGroups,SIGNAL(triggered()),m_ui->localPathGroups,SLOT(show()));
    //#endif
    connect(m_ui->actionKCDPropietes,SIGNAL(triggered()),mKCDpropertiesWindow,SLOT(show()));
    connect(m_ui->actionDrawColl,SIGNAL(triggered()),this,SLOT(setDrawColl()));

    connect(m_ui->actionLoadGraph,SIGNAL(triggered()),this,SLOT(loadGraph()));
    connect(m_ui->actionSaveGraph_2,SIGNAL(triggered()),this,SLOT(saveGraph()));
    connect(m_ui->actionSaveToDot,SIGNAL(triggered()),this,SLOT(saveXYZGraphToDot()));

    connect(m_ui->actionLoadTrajectory,SIGNAL(triggered()),this,SLOT(loadTraj()));
    connect(m_ui->actionSaveTrajectory,SIGNAL(triggered()),this,SLOT(saveTraj()));

    connect(m_ui->actionLoadInterfaceParameters,SIGNAL(triggered()),this,SLOT(loadInterfaceParameters()));
    connect(m_ui->actionSaveInterfaceParameters,SIGNAL(triggered()),this,SLOT(saveInterfaceParameters()));
    connect(m_ui->actionLoadParametersQuick,SIGNAL(triggered()),this,SLOT(loadParametersQuick()));
    connect(m_ui->actionSaveParametersQuick,SIGNAL(triggered()),this,SLOT(saveParametersQuick()));

    // connect(m_ui->pagesOfStakedWidget, SIGNAL(activated(int)),m_ui->stackedWidget, SLOT(setCurrentIndex(int)));

    connectCheckBoxes();

    // MainWindow init functions
    initRunButtons();
    initViewerButtons();
    initLightSource();
    initRobotsMenu();


    //timer for recording
    timer = new QTimer(this);
    connect( timer, SIGNAL(timeout()), this, SLOT(saveVideoTimer()), Qt::DirectConnection );
    isRecording = false;
}

MainWindow::~MainWindow()
{
	delete mKCDpropertiesWindow;
	delete m_ui;
}

//! Return the OpenGl display
GLWidget* MainWindow::getOpenGL()
{ 
	return m_ui->OpenGL; 
}

//! Refresh sliders
void MainWindow::refreshConstraintedDoFs()
{
  m_ui->tabRobot->getMoveRobot()->refreshConstraintedDoFs();
}

//! Return the MoveRobot widget
MoveRobot* MainWindow::getMoveRobot()
{
	return m_ui->tabRobot->getMoveRobot();
}

void MainWindow::initRobotsMenu()
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

void MainWindow::setRobotAsCurrent()
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

void MainWindow::openScenario()
{
	QString fileName = QFileDialog::getOpenFileName(this);
	
	if (!fileName.isEmpty())
	{
        qt_fileName = fileName.toAscii().data();
		qt_readScenario();
		m_ui->tabRobot->getMoveRobot()->updateAllRobotInitPos();
		this->drawAllWinActive();
	}
}

void MainWindow::saveScenario()
{
	QString fileName = QFileDialog::getSaveFileName(this);
	
	if (!fileName.isEmpty())
	{
        qt_fileName = fileName.toStdString().c_str();
		
#ifdef WITH_XFORMS
		std::string str = "saveScenario";
		write(qt_fl_pipe[1],str.c_str(),str.length()+1);
		cout << "Open scenario " << fileName.toStdString() << endl;
#else
		qt_saveScenario();
		//m_ui->formRobot->updateAllRobotInitPos();
		this->drawAllWinActive();
#endif
	}
}

p3d_matrix4 cam_tmp;

void MainWindow::changeCamera()
{
  G3D_Window *win = qt_get_cur_g3d_win();
  p3d_rob* r = (p3d_rob *) p3d_get_robot_by_name("PR2_ROBOT");
  p3d_jnt* j = r->joints[4];
  p3d_matrix4* m_transf = &(j->abs_pos);
  p3d_matrix4 T1,T2,T3,offset;
  
  p3d_mat4Copy(*m_transf,offset);
  p3d_mat4Pos(T1, 0, 0, 0, 0, 0.0, 0.8);
  p3d_mat4Pos(T2, -1.5, 0, 0, 0, 0, 0);
  p3d_mat4Mult(T1,T2,T3);
  p3d_mat4Mult(*m_transf,T3,offset);
  
  if( true )
  {
    cout << "Change camera" << endl;
    
    g3d_set_camera_parameters_from_frame(offset, win->vs);
    g3d_set_projection_matrix(win->vs.projection_mode);
    qt_change_mob_frame(win,m_transf);
  }
  else
  {
    qt_reset_mob_frame(win);
  }
}

void MainWindow::setDrawColl()
{
  static int n=0;
  
  g3d_set_draw_coll(n);
  cout << "Set draw collision to :  " << n << endl;
  
  if( n == 0 )
    n = 1;
  else
    n = 0;
}

void MainWindow::loadGraph()
{
	QString fileName = QFileDialog::getOpenFileName(this);
	
	if (!fileName.isEmpty())
	{
		qt_fileName = fileName.toStdString().c_str();
		char file[256];
		sprintf(file,"%s",qt_fileName);
		cout << "Loading graph at : " << file << endl;
        p3d_readGraph(file, DEFAULTGRAPH);
		this->drawAllWinActive();
	}
}


void MainWindow::saveGraph()
{
	QString fileName = QFileDialog::getSaveFileName(this);
	
	if (!fileName.isEmpty())
	{
		qt_fileName = fileName.toStdString().c_str();
		char file[256];
		sprintf(file,"%s",qt_fileName);
        cout <<"Saving Graph at " << file << endl;
        GraphConverter gc;
        p3d_graph* G= gc.convert(*API_activeGraph,true);
        p3d_writeGraph(G, file, DEFAULTGRAPH);//Mokhtar Using XML Format
        this->drawAllWinActive();
	}
}

void MainWindow::saveXYZGraphToDot()
{
    QString fileName = QFileDialog::getSaveFileName(this);

    if (!fileName.isEmpty())
    {
        qt_fileName = fileName.toStdString().c_str();
        char file[256];
        sprintf(file,"%s",qt_fileName);
        cout <<"Saving Graph at " << file << endl;
        API_activeGraph->saveBGLGraphToDotFile(file);
    }
}

void MainWindow::loadTraj()
{	
	QString fileName = QFileDialog::getOpenFileName(this);
	
	if (!fileName.isEmpty())
	{
		qt_fileName = fileName.toStdString().c_str();
		char file[256];
		sprintf(file,"%s",qt_fileName);
		qt_readTraj();
		cout << "Loading traj at : " << file << endl;
		this->drawAllWinActive();
	}
}

void MainWindow::saveTraj()
{
	QString fileName = QFileDialog::getSaveFileName(this);
	
	if (!fileName.isEmpty())
	{
		qt_fileName = fileName.toStdString().c_str();
		char file[256];
		sprintf(file,"%s",qt_fileName);
		cout <<"Saving traj at " << file << endl;
		p3d_save_traj(file,(p3d_traj *) p3d_get_desc_curid(P3D_TRAJ));
        //Robot* rob = global_Project->getActiveScene()->getRobotByName( global_ActiveRobotName );
        //p3d_writeXmlTraj(file, rob->getTrajStruct() );
		this->drawAllWinActive();
	}
}

void MainWindow::loadInterfaceParameters()
{
	QString fileName = QFileDialog::getOpenFileName(this);
	
	if (!fileName.isEmpty())
	{
		qt_loadInterfaceParameters( true, fileName.toStdString() );
		cout << "Loading parameters at : " << fileName.toStdString() << endl;
		this->drawAllWinActive();
	}
}

void MainWindow::saveInterfaceParameters()
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
		this->drawAllWinActive();
	}
}

void MainWindow::loadParametersQuick()
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
        this->drawAllWinActive();
    }
    else
    {
        cout << "Error : HOME_MOVE3D is not defined" << endl;
    }
}

void MainWindow::saveParametersQuick()
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
        this->drawAllWinActive();
    }
    else
    {
        cout << "Error : HOME_MOVE3D is not defined" << endl;
    }
}

void MainWindow::connectCheckBoxToEnv(QCheckBox* box, Env::boolParameter p)
{
	connect(ENV.getObject(p), SIGNAL(valueChanged(bool)), box, SLOT(setChecked(bool)), Qt::DirectConnection);
	connect(box, SIGNAL(toggled(bool)), ENV.getObject(p), SLOT(set(bool)), Qt::DirectConnection);
    box->setChecked(ENV.getBool(p));
}

void MainWindow::connectCheckBoxToEnv(QCheckBox* box, PlanParam::boolParameter p)
{
    connect(PlanEnv->getObject(p), SIGNAL(valueChanged(bool)), box, SLOT(setChecked(bool)), Qt::DirectConnection);
	connect(box, SIGNAL(toggled(bool)), PlanEnv->getObject(p), SLOT(set(bool)), Qt::DirectConnection);
	box->setChecked(PlanEnv->getBool(p));
}

// Light sliders ------------------------------------------------
// --------------------------------------------------------------
void MainWindow::initLightSource()
{
	connectCheckBoxToEnv(m_ui->checkBoxDrawLightSource,      Env::drawLightSource);
	
	vector<double>  envSize(6);
	envSize[0] = XYZ_ENV->box.x1; envSize[1] = XYZ_ENV->box.x2;
	envSize[2] = XYZ_ENV->box.y1; envSize[3] = XYZ_ENV->box.y2;
	envSize[4] = XYZ_ENV->box.z1; envSize[5] = XYZ_ENV->box.z2;
	
	m_ui->doubleSpinBoxLightX->setMinimum(2*envSize[0]);
	m_ui->doubleSpinBoxLightX->setMaximum(2*envSize[1]);
	m_ui->doubleSpinBoxLightY->setMinimum(2*envSize[2]);
	m_ui->doubleSpinBoxLightY->setMaximum(2*envSize[3]);
	m_ui->doubleSpinBoxLightZ->setMinimum(2*envSize[4]);
	m_ui->doubleSpinBoxLightZ->setMaximum(2*envSize[5]);
	
	QtShiva::SpinBoxSliderConnector *connectorLightX = new QtShiva::SpinBoxSliderConnector(
                                                                                           this, m_ui->doubleSpinBoxLightX, m_ui->horizontalSliderLightX);
	QtShiva::SpinBoxSliderConnector *connectorLightY = new QtShiva::SpinBoxSliderConnector(
                                                                                           this, m_ui->doubleSpinBoxLightY, m_ui->horizontalSliderLightY);
	QtShiva::SpinBoxSliderConnector *connectorLightZ = new QtShiva::SpinBoxSliderConnector(
                                                                                           this, m_ui->doubleSpinBoxLightZ, m_ui->horizontalSliderLightZ);
	
	connect(connectorLightX,SIGNAL(valueChanged(double)),this,SLOT(changeLightPosX()));
	connect(connectorLightY,SIGNAL(valueChanged(double)),this,SLOT(changeLightPosY()));
	connect(connectorLightZ,SIGNAL(valueChanged(double)),this,SLOT(changeLightPosZ()));
	
	connect(connectorLightX,SIGNAL(valueChanged(double)),this,SLOT(drawAllWinActive()));
	connect(connectorLightY,SIGNAL(valueChanged(double)),this,SLOT(drawAllWinActive()));
	connect(connectorLightZ,SIGNAL(valueChanged(double)),this,SLOT(drawAllWinActive()));
	
	m_ui->doubleSpinBoxLightX->setValue(G3D_WIN->vs.lightPosition[0]);
	m_ui->doubleSpinBoxLightY->setValue(G3D_WIN->vs.lightPosition[1]);
	m_ui->doubleSpinBoxLightZ->setValue(G3D_WIN->vs.lightPosition[2]);
}

void MainWindow::changeLightPosX()
{
	float* lightPosition = G3D_WIN->vs.lightPosition;
	lightPosition[0] = m_ui->doubleSpinBoxLightX->value();
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
	g3d_build_shadow_matrices(G3D_WIN->vs);
	//    cout << "Change X value" << endl;
#ifndef WITH_XFORMS
	this->drawAllWinActive();
#endif
}

void MainWindow::changeLightPosY()
{
	float* lightPosition = G3D_WIN->vs.lightPosition;
	lightPosition[1] = m_ui->doubleSpinBoxLightY->value();
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
	g3d_build_shadow_matrices(G3D_WIN->vs);
	//    cout << "Change Y value" << endl;
#ifndef WITH_XFORMS
	this->drawAllWinActive();
#endif
}

void MainWindow::changeLightPosZ()
{
	float* lightPosition = G3D_WIN->vs.lightPosition;
	lightPosition[2] = m_ui->doubleSpinBoxLightZ->value();
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
	g3d_build_shadow_matrices(G3D_WIN->vs);
	//    cout << "Change Z value" << endl;
#ifndef WITH_XFORMS
	this->drawAllWinActive();
#endif
}

// Viewer Buttons -----------------------------------------------
// --------------------------------------------------------------
extern PlannerHandler* global_plannerHandler;

void MainWindow::initViewerButtons()
{
    connect(this, SIGNAL(selectedPlanner(QString)),
            global_plannerHandler, SLOT(startPlanner(QString)));
    
    connect(m_ui->checkBoxDrawGraph,SIGNAL(toggled(bool)),this,SLOT(drawAllWinActive()),Qt::QueuedConnection);
    connect(m_ui->checkBoxDrawTraj,SIGNAL(toggled(bool)),this,SLOT(drawAllWinActive()),Qt::QueuedConnection);


    connect( ENV.getObject(Env::drawGraph), SIGNAL(valueChanged(bool)), this, SLOT(test()) );

    connectCheckBoxToEnv(m_ui->checkBoxDisableDraw,Env::drawDisabled);
    connectCheckBoxToEnv(m_ui->checkBoxDrawGraph,Env::drawGraph);
    connectCheckBoxToEnv(m_ui->checkBoxDrawExploration,Env::drawExploration);
    connectCheckBoxToEnv(m_ui->checkBoxDrawTraj,Env::drawTraj);
    connectCheckBoxToEnv(m_ui->checkBoxDrawTrajVector,Env::drawTrajVector);
    connectCheckBoxToEnv(m_ui->checkBoxDrawDebug,Env::debugCostOptim);
    connectCheckBoxToEnv(m_ui->checkBoxDrawMultiColoredTraj, Env::drawMultiColorLocalpath );
    
    m_ui->checkBoxDrawGraph->setCheckState(Qt::Checked);

    // Joint to Draw
    new QtShiva::SpinBoxConnector(this, m_ui->spinBoxJointToDraw,Env::jntToDraw);
    connect(m_ui->spinBoxJointToDraw,SIGNAL(valueChanged(int)),this,SLOT(setJointToDraw(int)));
    m_ui->spinBoxJointToDraw->setValue(XYZ_ROBOT->o[XYZ_ROBOT->no-1]->jnt->num);
    setJointToDraw( m_ui->spinBoxJointToDraw->value() );
  
    // Show traj and trace
    connect(m_ui->pushButtonShowTrace,SIGNAL(clicked(bool)),this,SLOT(showTrace()));
    connect(m_ui->pushButtonShowTraj,SIGNAL(clicked(bool)),this,SLOT(showTraj()),Qt::DirectConnection);
    connect(m_ui->pushButtonSaveVideo,SIGNAL(clicked(bool)),this,SLOT(saveVideo()));
    new QtShiva::SpinBoxSliderConnector(
            this, m_ui->doubleSpinBoxTrajSpeed, m_ui->horizontalSliderTrajSpeed , Env::showTrajFPS );

    connect(m_ui->pushButtonRestoreView,SIGNAL(clicked(bool)),this,SLOT(restoreView()),Qt::DirectConnection);
    //	connect(m_ui->pushButtonResetGraph,SIGNAL(clicked()),this,SLOT(ResetGraph()));

    connect(m_ui->pushButtonAddTraj,SIGNAL(clicked()),this,SLOT(addTrajToDraw()));
    connect(m_ui->pushButtonClearTraj,SIGNAL(clicked()),this,SLOT(clearTrajToDraw()));

    connect(m_ui->comboBoxColorTraj, SIGNAL(currentIndexChanged(int)),this,SLOT(colorTrajChange(int)));

    connect(m_ui->pushButtonMobileCamera,SIGNAL(clicked()),this,SLOT(mobileCamera()));
}

void MainWindow::test()
{
	
}

void MainWindow::setJointToDraw(int joint)
{
  p3d_set_user_drawnjnt(joint);
  this->drawAllWinActive();
}

void MainWindow::mobileCamera()
{
	qtMobileCamera* dialog = new qtMobileCamera;
	dialog->show();
}

// Show Trajectory ----------------------------------------------
// --------------------------------------------------------------

GLWidget* ptrOpenGL;

void MainWindow::setCurrentTraj(p3d_traj* traj)
{
	p3d_sel_desc_id(P3D_ROBOT,traj->rob);
	traj->rob->tcur = traj;
}

void MainWindow::showTraj()
{
	emit(selectedPlanner(QString("ShowTraj")));
	isPlanning();
}

void MainWindow::showTrace()
{
    G3D_DRAW_TRACE = !G3D_DRAW_TRACE;
    drawAllWinActive();
}

// --------------------------------------------------------------
void MainWindow::connectCheckBoxes()
{
	connect(m_ui->checkBoxGhosts, SIGNAL(toggled(bool)), this , SLOT(setBoolGhost(bool)), Qt::DirectConnection);
	connect(m_ui->checkBoxGhosts, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));
	
	connect(m_ui->checkBoxBB, SIGNAL(toggled(bool)), this , SLOT(setBoolBb(bool)), Qt::DirectConnection);
	connect(m_ui->checkBoxBB, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));
	
	connect(m_ui->checkBoxFloor, SIGNAL(toggled(bool)), this , SLOT(setBoolFloor(bool)), Qt::DirectConnection);
	connect(m_ui->checkBoxFloor, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));
	if(G3D_WIN->vs.displayFloor){
		m_ui->checkBoxFloor->setCheckState(Qt::Checked);
	}
	
	connect(m_ui->checkBoxTiles, SIGNAL(toggled(bool)), this , SLOT(setBoolTiles(bool)), Qt::DirectConnection);
	connect(m_ui->checkBoxTiles, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));
	if(G3D_WIN->vs.displayTiles){
		m_ui->checkBoxTiles->setCheckState(Qt::Checked);
	}
	
	connect(m_ui->checkBoxWalls, SIGNAL(toggled(bool)), this , SLOT(setBoolWalls(bool)), Qt::DirectConnection);
	connect(m_ui->checkBoxWalls, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));
	
	connect(m_ui->checkBoxShadows, SIGNAL(toggled(bool)), this , SLOT(setBoolShadows(bool)), Qt::DirectConnection);
	connect(m_ui->checkBoxShadows, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));
	
	connect(m_ui->checkBoxSmooth, SIGNAL(toggled(bool)), this , SLOT(setBoolSmooth(bool)), Qt::DirectConnection);
	connect(m_ui->checkBoxSmooth, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));
	if(G3D_WIN->vs.GOURAUD){
		m_ui->checkBoxSmooth->setCheckState(Qt::Checked);
	}
	
	connect(m_ui->checkBoxFilaire, SIGNAL(toggled(bool)), this , SLOT(setBoolFilaire(bool)), Qt::DirectConnection);
	connect(m_ui->checkBoxFilaire, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));
	
	connect(m_ui->checkBoxJoints, SIGNAL(toggled(bool)), this , SLOT(setBoolJoints(bool)), Qt::DirectConnection);
	connect(m_ui->checkBoxJoints, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));
	
	connect(m_ui->checkBoxContour, SIGNAL(toggled(bool)), this , SLOT(setBoolContour(bool)), Qt::DirectConnection);
	connect(m_ui->checkBoxContour, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));
	
	connect(m_ui->checkBoxEnableLight, SIGNAL(toggled(bool)), this , SLOT(setBoolEnableLight(bool)), Qt::DirectConnection);
	connect(m_ui->checkBoxEnableLight, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));
    
        connect(m_ui->checkBoxEnableShaders, SIGNAL(toggled(bool)), this , SLOT(setBoolEnableShaders(bool)), Qt::DirectConnection);
	connect(m_ui->checkBoxEnableShaders, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));

        connect(m_ui->checkBoxFlatFloor, SIGNAL(toggled(bool)), this , SLOT(setBoolFlatBox(bool)), Qt::DirectConnection);
        connect(m_ui->checkBoxFlatFloor, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));
	
	connectCheckBoxToEnv(m_ui->checkBoxAxis, Env::drawFrame);
	connect(m_ui->checkBoxAxis, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));
}

void MainWindow::setBoolGhost(bool value)
{
	G3D_WIN->vs.GHOST = value;
}

void MainWindow::setBoolBb(bool value)
{
	G3D_WIN->vs.BB = value;
}


void MainWindow::setBoolFloor(bool value)
{
	G3D_WIN->vs.displayFloor = value;
}


void MainWindow::setBoolTiles(bool value)
{
	G3D_WIN->vs.displayTiles = value;
}


void MainWindow::setBoolWalls(bool value)
{
	G3D_WIN->vs.displayWalls = value;
}

void MainWindow::setBoolShadows(bool value)
{
	G3D_WIN->vs.displayShadows = value;
}

void MainWindow::setBoolJoints(bool value)
{
	G3D_WIN->vs.displayJoints = value;
}


void MainWindow::setBoolSmooth(bool value)
{
	G3D_WIN->vs.GOURAUD = value;
}

void MainWindow::setBoolFilaire(bool value)
{
	G3D_WIN->vs.FILAIRE = value;
}

void MainWindow::setBoolContour(bool value)
{
	G3D_WIN->vs.CONTOUR = value;
}

void MainWindow::setBoolEnableLight(bool value)
{
	G3D_WIN->vs.enableLight = value;
}

void MainWindow::setBoolEnableShaders(bool value)
{
	G3D_WIN->vs.enableShaders = value;
}

void MainWindow::setBoolFlatBox(bool value)
{
        G3D_WIN->vs.flatBoxFloor = value;
}

void MainWindow::restoreView()
{
	g3d_restore_win_camera(G3D_WIN->vs);
	drawAllWinActive();
}

void MainWindow::addTrajToDraw()
{
	p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
	p3d_traj* CurrentTrajPt = robotPt->tcur;
#ifdef MOVE3D_CORE
	API::Trajectory traj(new Robot(robotPt),CurrentTrajPt);
	trajToDraw.push_back(traj);
#endif
}

void MainWindow::clearTrajToDraw()
{
#ifdef MOVE3D_CORE
	trajToDraw.clear();
#endif
}

void MainWindow::colorTrajChange(int color)
{
	cout << "Change traj color" << endl;
#ifdef CXX_PLANNNER
	for( unsigned int i=0; i<trajToDraw.size(); i++ ) 
	{
		cout << " Change traj " << i << " to : " << color << endl;
		trajToDraw[i].setColor(color);
	}
#endif
	this->drawAllWinActive();
}


// Run Buttons -----------------------------------------------
// -----------------------------------------------------------

void MainWindow::initRunButtons()
{
    connect(m_ui->pushButtonRun,SIGNAL(clicked(bool)),
            this, SIGNAL(runClicked()));
    connect(m_ui->pushButtonStop,SIGNAL(clicked(bool)),
            this, SIGNAL(stopClicked()));
    connect(m_ui->pushButtonReset,SIGNAL(clicked(bool)),
            this, SIGNAL(resetClicked()));
	
	m_ui->pushButtonRun->setDisabled(false);
	m_ui->pushButtonStop->setDisabled(true);
	m_ui->pushButtonReset->setDisabled(true);
	
    connect(ENV.getObject(Env::isPRMvsDiffusion), SIGNAL(valueChanged(bool)), m_ui->radioButtonPRM, SLOT(setChecked(bool)), Qt::DirectConnection);
    connect(m_ui->radioButtonPRM, SIGNAL(toggled(bool)), ENV.getObject(Env::isPRMvsDiffusion), SLOT(set(bool)), Qt::DirectConnection);
     connect(m_ui->radioButtonPRM, SIGNAL(toggled(bool)), SLOT(envSelectedPlannerTypeChanged(bool)), Qt::DirectConnection);
    m_ui->radioButtonDiff->setChecked(!ENV.getBool(Env::isPRMvsDiffusion));
    envSelectedPlannerTypeChanged(ENV.getBool(Env::isPRMvsDiffusion));
	connectCheckBoxToEnv(m_ui->checkBoxWithSmoothing,					PlanParam::withSmoothing);
//	connectCheckBoxToEnv(m_ui->checkBoxUseP3DStructures,      Env::use_p3d_structures);
    
	
	connect( ENV.getObject(Env::isRunning), SIGNAL(valueChanged(bool)), this, SLOT(planningFinished(void)) , Qt::QueuedConnection );
  
  connect( m_ui->pushButtonNextIteration, SIGNAL(clicked(bool)), PlanEnv->getObject(PlanParam::nextIterWaitForGui), SLOT(set(bool)) , Qt::DirectConnection );
}

void MainWindow::envSelectedPlannerTypeChanged(bool isPRMvsDiffusion)
{

        m_ui->tabMotionPlanner->getMui()->tabPRM->setEnabled(isPRMvsDiffusion);

        m_ui->tabMotionPlanner->getMui()->tabDiffu->setEnabled(!isPRMvsDiffusion);

}

//------------------------------------------------------------------------------
void MainWindow::enableRunAndResetButtons()
{
    m_ui->pushButtonRun->setDisabled(false);
    m_ui->pushButtonStop->setDisabled(true);
    m_ui->pushButtonReset->setDisabled(false);
}
//------------------------------------------------------------------------------
void MainWindow::enableStopButton()
{
    this->isPlanning();
    m_ui->pushButtonRun->setDisabled(true);
    m_ui->pushButtonStop->setDisabled(false);
    m_ui->pushButtonReset->setDisabled(true);
}
//------------------------------------------------------------------------------
void MainWindow::enableRunButton()
{
    m_ui->pushButtonRun->setDisabled(false);
    m_ui->pushButtonStop->setDisabled(true);
    m_ui->pushButtonReset->setDisabled(true);
}
//------------------------------------------------------------------------------
void MainWindow::isPlanning()
{
	m_ui->pushButtonRun->setDisabled(true);
	m_ui->pushButtonReset->setDisabled(true);
	m_ui->pushButtonStop->setDisabled(false);
	
	ENV.setBool(Env::isRunning,true);
	
	QPalette pal(Qt::red);
	m_ui->labelRunning->setPalette( pal );
	m_ui->labelRunning->setText("RUNNING" );
	
	//UITHINGQPalette pal(Qt::lightGray); // copy widget's palette to non const QPalette
	//UITHINGm_ui->toolBox->setPalette( pal );        // set the widget's palette
	
	m_ui->labelRunning->setText(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
                                                        "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
                                                        "p, li { white-space: pre-wrap; }\n"
                                                        "</style></head><body style=\" font-family:'Lucida Grande'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
                                                        "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:16pt; color:#FF0000;\">Running</span></p></body></html>",
                                                        0, QApplication::UnicodeUTF8));
	
}

void MainWindow::planningFinished()
{
	if( ENV.getBool(Env::isRunning) == false )
	{
		m_ui->pushButtonStop->setDisabled(true);
		m_ui->pushButtonReset->setDisabled(false);
		
		//        m_ui->labelRunning->setText("Not Running" );
		
		m_ui->labelRunning->setText(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
                                                            "<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
                                                            "p, li { white-space: pre-wrap; }\n"
                                                            "</style></head><body style=\" font-family:'Lucida Grande'; font-size:13pt; font-weight:400; font-style:normal;\">\n"
                                                            "<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" font-size:16pt; color:#008d00;\">Not Running</span></p></body></html>",
                                                            0, QApplication::UnicodeUTF8));
		
		// set the widget's palette
	}
	else
	{
		this->isPlanning();
	}
}

void MainWindow::drawAllWinActive()
{
    if(!ENV.getBool(Env::isRunning))
    {
        m_ui->OpenGL->updateGL();
    }
}

// Key events ---------------------------------------------------
// --------------------------------------------------------------

void MainWindow::keyPressEvent(QKeyEvent *event)
{
	//    cout << "Key pressed" << endl;
	switch(event->key())
	{
    case Qt::Key_X:
			mouse_mode = 1;
			//cout << "Switch to second" << endl;
			break;
      
    case Qt::Key_C:
      mouse_mode = 2;
      //cout << "Switch to third" << endl;
      break;
      
    case Qt::Key_G:
      ENV.setBool(Env::drawGrid,!ENV.getBool(Env::drawGrid));
      drawAllWinActive();
			break;
      
    case Qt::Key_D:
      drawAllWinActive();
			break;
      
	}
}

void MainWindow::keyReleaseEvent(QKeyEvent *e)
{
	mouse_mode = 0;
}


// Basic function -----------------------------------------------
// --------------------------------------------------------------

LabeledSlider* MainWindow::createSlider(QString s, Env::intParameter p,
                                        int lower, int upper)
{
	LabeledSlider* slider = new LabeledSlider(lower, upper, lower, s);
	connect(ENV.getObject(p), SIGNAL(valueChanged(int)), slider,
            SLOT(setValue(int)), Qt::DirectConnection);
	connect(slider, SIGNAL(valueChanged(int)), ENV.getObject(p),
            SLOT(set(int)), Qt::DirectConnection);
	slider->setValue(ENV.getInt(p));
	return (slider);
}

LabeledDoubleSlider* MainWindow::createDoubleSlider(QString s,
                                                    Env::doubleParameter p, double lower, double upper)
{
	LabeledDoubleSlider* slider = new LabeledDoubleSlider(lower, upper, lower,s);
	connect(ENV.getObject(p), SIGNAL(valueChanged(double)), slider,
            SLOT(setValue(double)), Qt::DirectConnection);
	connect(slider, SIGNAL(valueChanged(double)), ENV.getObject(p),
            SLOT(set(double)), Qt::DirectConnection);
	slider->setValue(ENV.getDouble(p));
	return (slider);
}

void MainWindow::changeEvent(QEvent *e)
{
	QWidget::changeEvent(e);
	switch (e->type()) {
        case QEvent::LanguageChange:
			m_ui->retranslateUi(this);
			break;
        default:
			break;
	}
}

static double last_record=0.0;
static double recording_time=0.0;
static int image_id=0;
static bool do_video_record=false;
static bool use_timer=true;

void MainWindow::saveVideo()
{
  if (!isRecording)
  {
    image_id=0;
    recording_time=0.0;
    ChronoTimeOfDayOn();
    getOpenGL()->resetImageVector();
    
    if( use_timer ) 
    {
      //VideoRecorder* video_record = new VideoRecorder(80,getOpenGL());
      //connect(video_record,SIGNAL(timeout()),getOpenGL(),SLOT(addCurrentImage()));
      //do_video_record=true;
      //video_record->start();
      timer->start(80);
    }
    else {
      getOpenGL()->setSaveTraj( true );
    }
    
    cout << "begin recording" << endl;
  }
  else
  {
    timer->stop();
    double time=0.0;
    ChronoTimeOfDayTimes(&time);
    ChronoTimeOfDayOff();
    do_video_record = false;
    getOpenGL()->setSaveTraj( false );
    getOpenGL()->saveImagesToDisk();
    getOpenGL()->resetImageVector();
    //cout << "Nb Image : " << time*80 << endl;
    //cout << "time recording : " << time << endl;
    cout << "end recording" << endl;
  }
  isRecording = !isRecording;
}

void MainWindow::saveVideoTimer()
{
    ChronoTimeOfDayTimes(&recording_time);
//    image_id++;
    cout << ++image_id << " , recording_time : " << recording_time-last_record << endl;
//    if( recording_time-last_record > 0.09 ) {
//      cout << "Error in recording image : " << recording_time-last_record << endl;
//    }
    getOpenGL()->addCurrentImage();
    last_record = recording_time;
    //double time_to_save=0.0;
    //ChronoTimeOfDayTimes(&time_to_save);
    //cout << "time_to_save : " << time_to_save-recording_time << endl;
}

void VideoRecorder::run()
{
  do_video_record = true;
  
  double time_video=0.0;
  double time_last_saved=0.0;

  while (do_video_record) 
  {
    //m_display->addCurrentImage();
    emit(timeout());
    ChronoTimeOfDayTimes(&time_video);
    //cout << "time save : " << time_video-time_last_saved << endl;
    time_last_saved = time_video;
    usleep((m_msec-20)*1000);
    
    bool wait=true;
    
    while (wait) 
    {
      ChronoTimeOfDayTimes(&time_video);
      //cout << "time_video : " << time_video << endl;
      if( time_video-time_last_saved > ((m_msec-0.1)/1000) ) {
        wait = false;
      }
    }
  }
}

