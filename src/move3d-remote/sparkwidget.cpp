#include "sparkwidget.hpp"
#include "ui_sparkwidget.h"


#include <QPainter>

#include "planner_handler.hpp"
#include "move3d-gui.h"
#include "API/scene.hpp"
#include "API/project.hpp"
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"

#include "qtBase/SpinBoxSliderConnector_p.hpp"

using namespace std;

sparkWidget::sparkWidget(QWidget *parent) :
    QWidget(parent),
    m_ui(new Ui::sparkWidget)
{
    m_ui->setupUi(this);
}

sparkWidget::~sparkWidget()
{
    delete m_ui;
}

void sparkWidget::init(PosterReader *pr, Ui::ParamWidget *ui_param)
{
  if( (!pr) || (!ui_param))
  {
    cout << "cameraWidget not well initialized!!!" << endl;
    return;
  }
  
  m_pr = pr;
  m_ui_p = ui_param;
    
  connect(m_pr,SIGNAL(drawAllWinActive()), this, SLOT(drawAllWinActive()));
  /* viewer page */
  connectCheckBoxes();
  initLightSource();
}

void sparkWidget::changeEvent(QEvent *e)
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


//! Return the OpenGl display
GLWidget* sparkWidget::getOpenGL()
{
    return m_ui->OpenGL;
}

void sparkWidget::drawAllWinActive()
{
    m_ui->OpenGL->updateGL();
}


void sparkWidget::connectCheckBoxes()
{
    connect(m_ui_p->checkBoxGhosts, SIGNAL(toggled(bool)), this , SLOT(setBoolGhost(bool)), Qt::DirectConnection);
    connect(m_ui_p->checkBoxGhosts, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));

    connect(m_ui_p->checkBoxBB, SIGNAL(toggled(bool)), this , SLOT(setBoolBb(bool)), Qt::DirectConnection);
    connect(m_ui_p->checkBoxBB, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));

    connect(m_ui_p->checkBoxFloor, SIGNAL(toggled(bool)), this , SLOT(setBoolFloor(bool)), Qt::DirectConnection);
    connect(m_ui_p->checkBoxFloor, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));
    if(G3D_WIN->vs.displayFloor){
        m_ui_p->checkBoxFloor->setCheckState(Qt::Checked);
    }

    connect(m_ui_p->checkBoxTiles, SIGNAL(toggled(bool)), this , SLOT(setBoolTiles(bool)), Qt::DirectConnection);
    connect(m_ui_p->checkBoxTiles, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));
    if(G3D_WIN->vs.displayTiles){
        m_ui_p->checkBoxTiles->setCheckState(Qt::Checked);
    }

    connect(m_ui_p->checkBoxWalls, SIGNAL(toggled(bool)), this , SLOT(setBoolWalls(bool)), Qt::DirectConnection);
    connect(m_ui_p->checkBoxWalls, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));

    connect(m_ui_p->checkBoxShadows, SIGNAL(toggled(bool)), this , SLOT(setBoolShadows(bool)), Qt::DirectConnection);
    connect(m_ui_p->checkBoxShadows, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));

    connect(m_ui_p->checkBoxSmooth, SIGNAL(toggled(bool)), this , SLOT(setBoolSmooth(bool)), Qt::DirectConnection);
    connect(m_ui_p->checkBoxSmooth, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));
    if(G3D_WIN->vs.GOURAUD){
        m_ui_p->checkBoxSmooth->setCheckState(Qt::Checked);
    }

    connect(m_ui_p->checkBoxFilaire, SIGNAL(toggled(bool)), this , SLOT(setBoolFilaire(bool)), Qt::DirectConnection);
    connect(m_ui_p->checkBoxFilaire, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));

    connect(m_ui_p->checkBoxJoints, SIGNAL(toggled(bool)), this , SLOT(setBoolJoints(bool)), Qt::DirectConnection);
    connect(m_ui_p->checkBoxJoints, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));

    connect(m_ui_p->checkBoxContour, SIGNAL(toggled(bool)), this , SLOT(setBoolContour(bool)), Qt::DirectConnection);
    connect(m_ui_p->checkBoxContour, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));

    connect(m_ui_p->checkBoxEnableLight, SIGNAL(toggled(bool)), this , SLOT(setBoolEnableLight(bool)), Qt::DirectConnection);
    connect(m_ui_p->checkBoxEnableLight, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));

    //        connect(m_ui->checkBoxEnableShaders, SIGNAL(toggled(bool)), this , SLOT(setBoolEnableShaders(bool)), Qt::DirectConnection);
    //        connect(m_ui->checkBoxEnableShaders, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));

    connectCheckBoxToEnv(m_ui_p->checkBoxAxis, Env::drawFrame);
    connect(m_ui_p->checkBoxAxis, SIGNAL(toggled(bool)), m_ui->OpenGL , SLOT(updateGL()));


    connect(m_ui_p->pushButtonRestoreView,SIGNAL(clicked(bool)),this,SLOT(restoreView()),Qt::DirectConnection);
}


void sparkWidget::connectCheckBoxToEnv(QCheckBox* box, Env::boolParameter p)
{
    connect(ENV.getObject(p), SIGNAL(valueChanged(bool)), box, SLOT(setChecked(bool)), Qt::DirectConnection);
    connect(box, SIGNAL(toggled(bool)), ENV.getObject(p), SLOT(set(bool)), Qt::DirectConnection);
    box->setChecked(ENV.getBool(p));
}


void sparkWidget::setBoolGhost(bool value)
{
    G3D_WIN->vs.GHOST = value;
}

void sparkWidget::setBoolBb(bool value)
{
    G3D_WIN->vs.BB = value;
}


void sparkWidget::setBoolFloor(bool value)
{
    G3D_WIN->vs.displayFloor = value;
}

void sparkWidget::setBoolSky(bool value)
{
    G3D_WIN->vs.displaySky = value;
}

void sparkWidget::setBoolTiles(bool value)
{
    G3D_WIN->vs.displayTiles = value;
}


void sparkWidget::setBoolWalls(bool value)
{
    G3D_WIN->vs.displayWalls = value;
}

void sparkWidget::setBoolShadows(bool value)
{
    G3D_WIN->vs.displayShadows = value;
}

void sparkWidget::setBoolJoints(bool value)
{
    G3D_WIN->vs.displayJoints = value;
}


void sparkWidget::setBoolSmooth(bool value)
{
    G3D_WIN->vs.GOURAUD = value;
}

void sparkWidget::setBoolFilaire(bool value)
{
    G3D_WIN->vs.FILAIRE = value;
}

void sparkWidget::setBoolContour(bool value)
{
    G3D_WIN->vs.CONTOUR = value;
}

void sparkWidget::setBoolEnableLight(bool value)
{
    G3D_WIN->vs.enableLight = value;
}

void sparkWidget::setBoolEnableShaders(bool value)
{
    G3D_WIN->vs.enableShaders = value;
}

void sparkWidget::restoreView()
{
    g3d_restore_win_camera(G3D_WIN->vs);
    drawAllWinActive();
}

//void sparkWidget::on_checkBox_clicked(bool checked)
//{
//    this->menuBar()->setVisible(checked);
//}

// Light sliders ------------------------------------------------
// --------------------------------------------------------------
void sparkWidget::initLightSource()
{
    connectCheckBoxToEnv(m_ui_p->checkBoxDrawLightSource,      Env::drawLightSource);
    connect(m_ui_p->checkBoxDrawLightSource, SIGNAL(clicked()), m_ui->OpenGL , SLOT(updateGL()));

    std::vector<double>  envSize(6);
    envSize[0] = XYZ_ENV->box.x1; envSize[1] = XYZ_ENV->box.x2;
    envSize[2] = XYZ_ENV->box.y1; envSize[3] = XYZ_ENV->box.y2;
    envSize[4] = XYZ_ENV->box.z1; envSize[5] = XYZ_ENV->box.z2;

    m_ui_p->doubleSpinBoxLightX->setMinimum(2*envSize[0]);
    m_ui_p->doubleSpinBoxLightX->setMaximum(2*envSize[1]);
    m_ui_p->doubleSpinBoxLightY->setMinimum(2*envSize[2]);
    m_ui_p->doubleSpinBoxLightY->setMaximum(2*envSize[3]);
    m_ui_p->doubleSpinBoxLightZ->setMinimum(2*envSize[4]);
    m_ui_p->doubleSpinBoxLightZ->setMaximum(2*envSize[5]);



    QtShiva::SpinBoxSliderConnector *connectorLightX = new QtShiva::SpinBoxSliderConnector(
            this, m_ui_p->doubleSpinBoxLightX, m_ui_p->horizontalSliderLightX);
    QtShiva::SpinBoxSliderConnector *connectorLightY = new QtShiva::SpinBoxSliderConnector(
            this, m_ui_p->doubleSpinBoxLightY, m_ui_p->horizontalSliderLightY);
    QtShiva::SpinBoxSliderConnector *connectorLightZ = new QtShiva::SpinBoxSliderConnector(
            this, m_ui_p->doubleSpinBoxLightZ, m_ui_p->horizontalSliderLightZ);

    connect(connectorLightX,SIGNAL(valueChanged(double)),this,SLOT(changeLightPosX()));
    connect(connectorLightY,SIGNAL(valueChanged(double)),this,SLOT(changeLightPosY()));
    connect(connectorLightZ,SIGNAL(valueChanged(double)),this,SLOT(changeLightPosZ()));

    connect(connectorLightX,SIGNAL(valueChanged(double)),this,SLOT(drawAllWinActive()));
    connect(connectorLightY,SIGNAL(valueChanged(double)),this,SLOT(drawAllWinActive()));
    connect(connectorLightZ,SIGNAL(valueChanged(double)),this,SLOT(drawAllWinActive()));

    m_ui_p->doubleSpinBoxLightX->setValue(G3D_WIN->vs.lightPosition[0]);
    m_ui_p->doubleSpinBoxLightY->setValue(G3D_WIN->vs.lightPosition[1]);
    m_ui_p->doubleSpinBoxLightZ->setValue(G3D_WIN->vs.lightPosition[2]);


    setBoolSky(true);

    g3d_set_win_camera(G3D_WIN->vs, 7.0, -4.0,  1.0, 17, -0.7, 0.7, 0.0, 0.0, 1.0);
}


void sparkWidget::changeLightPosX()
{
    float* lightPosition = G3D_WIN->vs.lightPosition;
    lightPosition[0] = (float) m_ui_p->doubleSpinBoxLightX->value();
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    g3d_build_shadow_matrices(G3D_WIN->vs);
    //    cout << "Change X value" << endl;
    this->drawAllWinActive();

    //        cout << " light pose " << lightPosition[0]<< " " << lightPosition[1]<< " " <<lightPosition[2] << endl;
}

void sparkWidget::changeLightPosY()
{
    float* lightPosition = G3D_WIN->vs.lightPosition;
    lightPosition[1] = (float) m_ui_p->doubleSpinBoxLightY->value();
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    g3d_build_shadow_matrices(G3D_WIN->vs);
    //    cout << "Change Y value" << endl;
    this->drawAllWinActive();
    //               cout << " light pose " << lightPosition[0]<< " " << lightPosition[1]<< " " <<lightPosition[2] << endl;
}

void sparkWidget::changeLightPosZ()
{
    float* lightPosition = G3D_WIN->vs.lightPosition;
    lightPosition[2] = (float) m_ui_p->doubleSpinBoxLightZ->value();
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);
    g3d_build_shadow_matrices(G3D_WIN->vs);
    //    cout << "Change Z value" << endl;
    this->drawAllWinActive();
    //               cout << " light pose " << lightPosition[0]<< " " << lightPosition[1]<< " " <<lightPosition[2] << endl;
}




