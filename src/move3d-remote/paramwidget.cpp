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
#include "paramwidget.hpp"
#include "ui_paramwidget.h"
#include "posterreader.hpp"
#include "planner_handler.hpp"
#include "planner/planEnvironment.hpp"
#include "hri_costspace/HRICS_natural.hpp"
#include "hri_costspace/HRICS_costspace.hpp"

ParamWidget::ParamWidget(QWidget *parent) :
  QWidget(parent),
  m_ui(new Ui::ParamWidget)
{
  m_ui->setupUi(this);
}

ParamWidget::~ParamWidget()
{
  delete m_ui;
}

void ParamWidget::setAndConnectPosterReader(PosterReader *pr)
{
  m_pr = pr;
  /* spark page */
  //m_pr->getSparkPoster()->setRefreshStatus(m_ui->sparkCheckBox->isChecked());
  connect(m_ui->sparkCheckBox, SIGNAL(clicked()), this, SLOT(setSparkRefresh()));
  connect(m_ui->sparkSaveSceBut,SIGNAL(clicked()),this,SLOT(sparkSaveScenario()));
  connect(m_ui->checkBoxDrawGoto, SIGNAL(toggled(bool)), pr , SLOT(setDrawGoTo(bool)), Qt::DirectConnection);
//  connect(m_ui->checkBoxColorCost,SIGNAL(toggled(bool)), pr,SLOT(changeHumanColor(bool)));

  connect(PlanEnv->getObject(PlanParam::drawColorConfig), SIGNAL(valueChanged(bool)), m_ui->checkBoxColorCost, SLOT(setChecked(bool)), Qt::DirectConnection);
  connect(m_ui->checkBoxColorCost, SIGNAL(toggled(bool)), PlanEnv->getObject(PlanParam::drawColorConfig), SLOT(set(bool)), Qt::DirectConnection);
  m_ui->checkBoxColorCost->setChecked(PlanEnv->getBool(PlanParam::drawColorConfig));

  //connect(m_pr, SIGNAL(sparkStatus(bool)),this, SLOT(setSparkStatusText(bool)));
}


void ParamWidget::setSparkRefresh()
{
  m_pr->getSparkPoster()->setRefreshStatus(m_ui->sparkCheckBox->isChecked());
}

void ParamWidget::sparkSaveScenario()
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

void ParamWidget::setSparkStatusText(bool updating)
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

void ParamWidget::on_checkBoxColorCost_toggled(bool checked)
{

}

void ParamWidget::on_pushButtonNiutUpdate_toggled(bool checked)
{
    m_pr->getNiutPoster()->setRefreshStatus(checked);
}
