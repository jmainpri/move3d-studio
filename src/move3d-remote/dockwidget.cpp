#include "dockwidget.hpp"
#include "ui_dockwidget.h"

using namespace std;


DockWindow::DockWindow(PosterReader *pr, Ui::MainWindowRemote *ui_parent, QWidget *parent) :
        QWidget(parent),
        m_ui(new Ui::DockWindow)
{
  m_ui->setupUi(this);
  
  m_ui->niut->init(pr,ui_parent);
  m_ui->spark->init(pr,ui_parent);
  m_ui->camera->init(pr,ui_parent);
}


DockWindow::~DockWindow()
{
  delete m_ui;
}

