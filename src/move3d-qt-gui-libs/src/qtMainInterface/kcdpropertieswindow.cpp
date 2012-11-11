#include "kcdpropertieswindow.hpp"
#include "ui_kcdpropertieswindow.h"

#include "Collision-pkg.h"
#include <iostream>

using namespace std;

KCDpropertiesWindow::KCDpropertiesWindow(QWidget *parent) :
    QDialog(parent),
    m_ui(new Ui::KCDpropertiesWindow)
{
    m_ui->setupUi(this);
	
	connect(m_ui->comboBoxTrajMethod, SIGNAL(currentIndexChanged(int)),this, SLOT(setTestTrajMethod(int)));   
  //  connect(ENV.getObject(Env::costDeltaMethod), SIGNAL(valueChanged(int)),m_ui->comboBoxTrajMethod, SLOT(setCurrentIndex(int)));
	
	p3d_traj_test_type TrajMethod = p3d_col_env_get_traj_method();
  m_ui->comboBoxTrajMethod->setCurrentIndex( TrajMethod );

  connect( m_ui->pushButtonDrawAllOBBs, SIGNAL(clicked()), this, SLOT(drawOBBs()));  
}

KCDpropertiesWindow::~KCDpropertiesWindow()
{
    delete m_ui;
}

void KCDpropertiesWindow::changeEvent(QEvent *e)
{
    QDialog::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        m_ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

void KCDpropertiesWindow::setTestTrajMethod(int TrajMethod)
{
	p3d_col_env_set_traj_method((p3d_traj_test_type)TrajMethod);
}

bool all_obbs = false;
void KCDpropertiesWindow::drawOBBs()
{
  if( all_obbs )
  {
    g3d_set_kcd_draw_all_obbs(false);
  }
  else 
  {
    g3d_set_kcd_draw_all_obbs(true);
  }
}

