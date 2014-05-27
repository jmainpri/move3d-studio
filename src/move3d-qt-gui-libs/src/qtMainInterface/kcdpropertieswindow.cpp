/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
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
    all_obbs =true;
  }
  else 
  {
    g3d_set_kcd_draw_all_obbs(true);
    all_obbs = false;
  }
}

