#include "mainwindow-remote.hpp"
#include "ui_mainwindow-remote.h"
#include "qtOpenGL/glwidget.hpp"
#include "P3d-pkg.h"
#include "Util-pkg.h"
#include "pocolibsPoster.hpp"
#include "planner_handler.hpp"

MainWindowRemote::MainWindowRemote(QWidget *parent)
: QMainWindow(parent), m_ui(new Ui::MainWindowRemote)
{
        m_ui->setupUi(this);
        m_posterHandler = new FetchEnvironment();
        m_posterHandler->init(this);
        QTimer *timer = new QTimer(this);
        connect(timer, SIGNAL(timeout()), this, SLOT(myLoop()));
        timer->start(50);

        /* spark page */
        m_posterHandler->setSparkRefresh(m_ui->sparkCheckBox->isChecked());
        connect(m_ui->sparkCheckBox, SIGNAL(clicked()), this, SLOT(setSparkRefresh()));
        connect(m_ui->sparkSaveSceBut,SIGNAL(clicked()),this,SLOT(sparkSaveScenario()));

        /* nuit page */


        /* viman page */


}

MainWindowRemote::~MainWindowRemote()
{

}

void MainWindowRemote::setSparkRefresh()
{
    m_posterHandler->setSparkRefresh(m_ui->sparkCheckBox->isChecked());
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

void MainWindowRemote::myLoop() {
    /* Set spark status */
    if(m_posterHandler->getSparkStatus()) {
     m_ui->sparkStatusLineEdit->setText(QString("updating"));
    } else {
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
    if(!ENV.getBool(Env::isRunning))
    {
        m_ui->OpenGL->updateGL();
    }
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



