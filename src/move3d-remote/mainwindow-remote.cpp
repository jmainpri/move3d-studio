#include "mainwindow-remote.hpp"
#include "ui_mainwindow-remote.h"

#include "qtOpenGL/glwidget.hpp"

#include "P3d-pkg.h"
#include "Util-pkg.h"

#include "pocolibsPoster.hpp"


MainWindowRemote::MainWindowRemote(QWidget *parent)
: QMainWindow(parent), m_ui(new Ui::MainWindowRemote)
{

        m_ui->setupUi(this);

}

MainWindowRemote::~MainWindowRemote()
{

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

void MainWindowRemote::restoreView()
{
        //g3d_restore_win_camera(G3D_WIN->vs);
        drawAllWinActive();


        FetchEnvironment* posterHandler = new FetchEnvironment();

        if( !posterHandler->init(this) )
        {
            std::cout << " YESSSS : Poster Found!!!" << endl;
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

                        //            if(mouse_mode == 2)
                        //            {
                        //                mouse_mode = 0;
                        //                cout << "Switch to normal" << endl;
                        //                return;
                        //            }
                        //                break;
        }
}

void MainWindowRemote::keyReleaseEvent(QKeyEvent *e)
{
        mouse_mode = 0;
}



