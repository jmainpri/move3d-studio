#include "mainwindow-remote.hpp"
#include "ui_mainwindow-remote.h"
//#include "qtOpenGL/glwidget.hpp"

#include "posterreader.hpp"

#include "qtBase/SpinBoxSliderConnector_p.hpp"
#include <iostream>
#include <vector>
#include <QPainter>
#include <QSettings>
#include <QMessageBox>
#include "qtipl.hpp"


#include "planner_handler.hpp"
#include "move3d-gui.h"
#include "API/scene.hpp"
#include "API/project.hpp"
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"


using namespace std;


MainWindowRemote::MainWindowRemote(PosterReader *pr, QWidget *parent) :
        QMainWindow(parent),
        m_pr(pr),
        m_ui(new Ui::MainWindowRemote)
{
    //    QTimer *timer = new QTimer(this);
    //    connect(timer, SIGNAL(timeout()), this, SLOT(looptest()));
    //    timer->start(100);

    m_ui->setupUi(this);

    /* menu robots */
    initRobotsMenu();
    this->menuBar()->setVisible(true);

    /* spark page */
    //m_pr->getSparkPoster()->setRefreshStatus(m_ui->sparkCheckBox->isChecked());
    connect(m_ui->sparkCheckBox, SIGNAL(clicked()), this, SLOT(setSparkRefresh()));
    connect(m_ui->sparkSaveSceBut,SIGNAL(clicked()),this,SLOT(sparkSaveScenario()));
    connect(m_pr, SIGNAL(sparkStatus(bool)),this, SLOT(setSparkStatusText(bool)));

    on_pushButtonLoadSettings_clicked();

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


void MainWindowRemote::setSparkRefresh()
{
    m_pr->getSparkPoster()->setRefreshStatus(m_ui->sparkCheckBox->isChecked());
}

void MainWindowRemote::sparkSaveScenario()
{
    QString fileName = QFileDialog::getSaveFileName(this);
    if (!fileName.isEmpty())
    {
        qt_fileName = fileName.toStdString().c_str();
        qt_saveScenario();
        emit drawAllWinActive();
        //this->drawAllWinActive();
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
