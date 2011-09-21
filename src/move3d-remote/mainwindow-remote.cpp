#include "mainwindow-remote.hpp"
#include "ui_mainwindow-remote.h"
#include "qtOpenGL/glwidget.hpp"

#include "posterreader.hpp"

#include "qtBase/SpinBoxSliderConnector_p.hpp"
#include <iostream>
#include <vector>
#include <QPainter>
#include "qtipl.hpp"

#include "planner_handler.hpp"
#include "move3d-gui.h"
#include "API/scene.hpp"
#include "API/project.hpp"
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"

using namespace std;

MainWindowRemote::MainWindowRemote(QWidget *parent)
  : QMainWindow(parent), m_ui(new Ui::MainWindowRemote)
{
  _qimageLeft = NULL;
  _qimageRight = NULL;
  _dataImageLeft = NULL; 
  _dataImageRight = NULL;

  QTimer *timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(looptest()));
  timer->start(100);

  m_ui->setupUi(this);
  m_posterHandler = new PosterReader(this);
  m_posterHandler->init();

  /* viewer page */
  connectCheckBoxes();
  initLightSource();

  /* menu robots */
  initRobotsMenu();

  /* spark page */
  //m_posterHandler->getSparkPoster()->setRefreshStatus(m_ui->sparkCheckBox->isChecked());
  connect(m_ui->sparkCheckBox, SIGNAL(clicked()), this, SLOT(setSparkRefresh()));
  connect(m_ui->sparkSaveSceBut,SIGNAL(clicked()),this,SLOT(sparkSaveScenario()));
  connect(m_posterHandler, SIGNAL(sparkStatus(bool)),this, SLOT(setSparkStatusText(bool)));
  
  /* nuit page */
  initNiut();
}

void MainWindowRemote::looptest()
{
  QPixmap pm;
    
  //  QPixmap pm2("/home/xbroquer/VIMAN_Right.png");

  if(m_posterHandler->_viamImagePoster->iplImgLeft()) {
  
    /* Convert the IplImage to a QImage one */
    _qimageLeft = IplImageToQImage(m_posterHandler->_viamImagePoster->iplImgLeft(), &_dataImageLeft);

    /* copy the image to free the allocated memory */
    QImage qimageLeft;
    qimageLeft = _qimageLeft->copy();
    delete _dataImageLeft;
    delete _qimageLeft;
    _dataImageLeft = NULL;
    _qimageLeft = NULL;

    qimageLeft =  qimageLeft.scaled(512, 360);
    m_ui->labelImageLeft->setPixmap(pm.fromImage(qimageLeft, 0));
    m_ui->labelImageLeft->show();
  
  }

  //    m_ui->labelImageLeft->setPixmap(pm2);
  //m_ui->labelImageLeft->show();
  if(m_posterHandler->_viamImagePoster->iplImgRight()) {

    /* Convert the IplImage to a QImage one */
    _qimageRight = IplImageToQImage(m_posterHandler->_viamImagePoster->iplImgRight(), &_dataImageRight);
    
    /* copy the image to free the allocated memory */
    QImage qimageRight;
    qimageRight= _qimageRight->copy();
    delete _dataImageRight;
    delete _qimageRight;
    _dataImageRight = NULL;
    _qimageRight = NULL;

    qimageRight =  qimageRight.scaled(512, 360);
    m_ui->labelImageRight->setPixmap(pm.fromImage(qimageRight, 0));
    m_ui->labelImageRight->show();


  }
}

MainWindowRemote::~MainWindowRemote()
{
  if(_qimageLeft)
    {
      delete _qimageLeft;
    }
  if(_qimageRight)
    {
      delete _qimageRight;
    }
  delete m_posterHandler;
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


void MainWindowRemote::setSparkRefresh()
{
  m_posterHandler->getSparkPoster()->setRefreshStatus(m_ui->sparkCheckBox->isChecked());
}

void MainWindowRemote::sparkSaveScenario()
{
  QString fileName = QFileDialog::getSaveFileName(this);
  if (!fileName.isEmpty())
    {
      qt_fileName = fileName.toStdString().c_str();
      qt_saveScenario();
      this->drawAllWinActive();
    }
}

void MainWindowRemote::setSparkStatusText(bool updating)
{
  if(updating)
    {
      m_ui->sparkStatusLineEdit->setText(QString("updating"));
    }
  else
    {
      m_ui->sparkStatusLineEdit->setText(QString("not updating"));
    }
}

//! Return the OpenGl display
GLWidget* MainWindowRemote::getOpenGL()
{
  return m_ui->OpenGL;
}

void MainWindowRemote::drawAllWinActive()
{
  //    if(!ENV.getBool(Env::isRunning))
  //    {
  //    std::cout << "update" << std::endl;

  m_ui->OpenGL->updateGL();
  //    }
}

void MainWindowRemote::keyPressEvent(QKeyEvent *event)
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
    }
}

void MainWindowRemote::keyReleaseEvent(QKeyEvent *e)
{
  mouse_mode = 0;

}

void MainWindowRemote::connectCheckBoxes()
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

  //        connect(m_ui->checkBoxEnableShaders, SIGNAL(toggled(bool)), this , SLOT(setBoolEnableShaders(bool)), Qt::DirectConnection);
  //        connect(m_ui->checkBoxEnableShaders, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));

  connectCheckBoxToEnv(m_ui->checkBoxAxis, Env::drawFrame);
  connect(m_ui->checkBoxAxis, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));


  connect(m_ui->pushButtonRestoreView,SIGNAL(clicked(bool)),this,SLOT(restoreView()),Qt::DirectConnection);
}


void MainWindowRemote::connectCheckBoxToEnv(QCheckBox* box, Env::boolParameter p)
{
  connect(ENV.getObject(p), SIGNAL(valueChanged(bool)), box, SLOT(setChecked(bool)), Qt::DirectConnection);
  connect(box, SIGNAL(toggled(bool)), ENV.getObject(p), SLOT(set(bool)), Qt::DirectConnection);
  box->setChecked(ENV.getBool(p));
}


void MainWindowRemote::setBoolGhost(bool value)
{
  G3D_WIN->vs.GHOST = value;
}

void MainWindowRemote::setBoolBb(bool value)
{
  G3D_WIN->vs.BB = value;
}


void MainWindowRemote::setBoolFloor(bool value)
{
  G3D_WIN->vs.displayFloor = value;
}

void MainWindowRemote::setBoolSky(bool value)
{
  G3D_WIN->vs.displaySky = value;
}

void MainWindowRemote::setBoolTiles(bool value)
{
  G3D_WIN->vs.displayTiles = value;
}


void MainWindowRemote::setBoolWalls(bool value)
{
  G3D_WIN->vs.displayWalls = value;
}

void MainWindowRemote::setBoolShadows(bool value)
{
  G3D_WIN->vs.displayShadows = value;
}

void MainWindowRemote::setBoolJoints(bool value)
{
  G3D_WIN->vs.displayJoints = value;
}


void MainWindowRemote::setBoolSmooth(bool value)
{
  G3D_WIN->vs.GOURAUD = value;
}

void MainWindowRemote::setBoolFilaire(bool value)
{
  G3D_WIN->vs.FILAIRE = value;
}

void MainWindowRemote::setBoolContour(bool value)
{
  G3D_WIN->vs.CONTOUR = value;
}

void MainWindowRemote::setBoolEnableLight(bool value)
{
  G3D_WIN->vs.enableLight = value;
}

void MainWindowRemote::setBoolEnableShaders(bool value)
{
  G3D_WIN->vs.enableShaders = value;
}

void MainWindowRemote::restoreView()
{
  g3d_restore_win_camera(G3D_WIN->vs);
  drawAllWinActive();
}

// Light sliders ------------------------------------------------
// --------------------------------------------------------------
void MainWindowRemote::initLightSource()
{
  connectCheckBoxToEnv(m_ui->checkBoxDrawLightSource,      Env::drawLightSource);
  connect(m_ui->checkBoxDrawLightSource, SIGNAL(clicked()), m_ui->OpenGL , SLOT(updateGL()));

  std::vector<double>  envSize(6);
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


  setBoolSky(true);

  g3d_set_win_camera(G3D_WIN->vs, 7.0, -4.0,  1.0, 17, -0.7, 0.7, 0.0, 0.0, 1.0);
}


void MainWindowRemote::changeLightPosX()
{
  float* lightPosition = G3D_WIN->vs.lightPosition;
  lightPosition[0] = (float) m_ui->doubleSpinBoxLightX->value();
  glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
  g3d_build_shadow_matrices(G3D_WIN->vs);
  //    cout << "Change X value" << endl;
  this->drawAllWinActive();

  //        cout << " light pose " << lightPosition[0]<< " " << lightPosition[1]<< " " <<lightPosition[2] << endl;
}

void MainWindowRemote::changeLightPosY()
{
  float* lightPosition = G3D_WIN->vs.lightPosition;
  lightPosition[1] = (float) m_ui->doubleSpinBoxLightY->value();
  glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
  g3d_build_shadow_matrices(G3D_WIN->vs);
  //    cout << "Change Y value" << endl;
  this->drawAllWinActive();
  //               cout << " light pose " << lightPosition[0]<< " " << lightPosition[1]<< " " <<lightPosition[2] << endl;
}

void MainWindowRemote::changeLightPosZ()
{
  float* lightPosition = G3D_WIN->vs.lightPosition;
  lightPosition[2] = (float) m_ui->doubleSpinBoxLightZ->value();
  glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
  g3d_build_shadow_matrices(G3D_WIN->vs);
  //    cout << "Change Z value" << endl;
  this->drawAllWinActive();
  //               cout << " light pose " << lightPosition[0]<< " " << lightPosition[1]<< " " <<lightPosition[2] << endl;
}

// Niut Window --------------------------------------------------
// --------------------------------------------------------------
void MainWindowRemote::initNiut()
{
  _niutLabels.push_back( m_ui->labelNiut1 );
  _niutLabels.push_back( m_ui->labelNiut2 );
  _niutLabels.push_back( m_ui->labelNiut3 );
  _niutLabels.push_back( m_ui->labelNiut4 );
  _niutLabels.push_back( m_ui->labelNiut5 );
  _niutLabels.push_back( m_ui->labelNiut6 );
  _niutLabels.push_back( m_ui->labelNiut7 );
  _niutLabels.push_back( m_ui->labelNiut8 );

  _niutLabels.push_back( m_ui->labelNiut9 );
  _niutLabels.push_back( m_ui->labelNiut10 );
  _niutLabels.push_back( m_ui->labelNiut11 );
  _niutLabels.push_back( m_ui->labelNiut12 );
  _niutLabels.push_back( m_ui->labelNiut13 );
  _niutLabels.push_back( m_ui->labelNiut14 );
  _niutLabels.push_back( m_ui->labelNiut15 );
  _niutLabels.push_back( m_ui->labelNiut16 );
  
  _niutPmDead = QPixmap("/home/jmainpri/workspace/move3d-studio/src/move3d-remote/yellow-death.jpg");;
  
  _niutPmRed = QPixmap("/home/jmainpri/workspace/move3d_machine_gs/move3d-studio/src/move3d-remote/red-man.jpg");
  _niutPmOrange = QPixmap("/home/jmainpri/workspace/move3d_machine_gs/move3d-studio/src/move3d-remote/orange-man.jpg");
  _niutPmYellow = QPixmap("/home/jmainpri/workspace/move3d_machine_gs/move3d-studio/src/move3d-remote/yellow-man.jpg");
  _niutPmGreen = QPixmap("/home/jmainpri/workspace/move3d_machine_gs/move3d-studio/src/move3d-remote/green-man.jpg");
  
  _niutPmRed = _niutPmRed.scaledToHeight(70);
  _niutPmOrange = _niutPmOrange.scaledToHeight(70);
  _niutPmYellow = _niutPmYellow.scaledToHeight(70);
  _niutPmGreen = _niutPmGreen.scaledToHeight(70);
  
  for (unsigned int i=0; i<_niutLabels.size(); i++) 
  {
    _niutLabels[i]->setPixmap(_niutPmRed);
    _niutLabels[i]->show();
  }
}

void MainWindowRemote::setNiutIsAlive(bool state)
{
//    cout << "Niut is : " << state << endl;
}

void MainWindowRemote::setNiutColorLabel(int id, int color)
{
  if (id<0 || id>=((int)_niutLabels.size())) {
    cout << "Error in " << __FILE__ << __func__ << endl;
    return;
  }
  
  switch (color) {
    case 0:
      _niutLabels[id]->setPixmap(_niutPmRed);
      break;
      
    case 1:
      _niutLabels[id]->setPixmap(_niutPmOrange);
      break;
      
    case 2:
      _niutLabels[id]->setPixmap(_niutPmYellow);
      break;
      
    case 3:
      _niutLabels[id]->setPixmap(_niutPmGreen);
      break;
      
    default:
      break;
  }
}

