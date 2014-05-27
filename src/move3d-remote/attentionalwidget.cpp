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
#include "attentionalwidget.hpp"
#include "ui_attentionalwidget.h"

#include "posterreader.hpp"

using namespace std;

AttentionalWidget::AttentionalWidget(QWidget *parent) :
  QWidget(parent),
  m_ui(new Ui::AttentionalWidget)
{
  m_ui->setupUi(this);


}

AttentionalWidget::~AttentionalWidget()
{
  delete m_ui;
}

void AttentionalWidget::init(PosterReader *pr, Ui::ParamWidget *ui_param)
{
  if( (!pr) || (!ui_param))
  {
    cout << "cameraWidget not well initialized!!!" << endl;
    return;
  }
  m_pr = pr;
  m_ui_p = ui_param;

  connect(m_pr->_attentionalPoster, SIGNAL(dataReady()), this,SLOT(update()));

   connect(m_pr->_attentionalOutputPoster, SIGNAL(dataReady()), this,SLOT(updateOutput()));
}

//!
//
void AttentionalWidget::update()
{
  ATTENTIONAL_REPORT_STR *attentionalReport;



  if(m_pr->_attentionalPoster->getPosterStuct((char *)(&m_pr->_attentionalPosterStruct)) == false) {
    cout << " AttentionalWidget::Update() getPosterStuct() ERROR " << endl;
    return;
  }
  attentionalReport = &(m_pr->_attentionalPosterStruct);


 // printf("attentionalReport->current_behavior %d %d %d\n", attentionalReport->current_behavior, sizeof(*attentionalReport), sizeof(ATTENTIONAL_REPORT_STR) );
  m_ui->doubleSpinBoxTotalCost->setValue(attentionalReport->total_cost);

  if(attentionalReport->holding == 1) {
    m_ui->checkBoxRobotHolding->setCheckState(Qt::Checked);
  } else {
    m_ui->checkBoxRobotHolding->setCheckState(Qt::Unchecked);
  }

  if(attentionalReport->active_behaviors[0] == 1) {
    m_ui->checkBoxAvoid->setCheckState(Qt::Checked);
  } else {
    m_ui->checkBoxAvoid->setCheckState(Qt::Unchecked);
  }
  m_ui->doubleSpinBoxAvoid->setValue(attentionalReport->cost_behaviors[0]);

  if(attentionalReport->active_behaviors[1] == 1) {
    m_ui->checkBoxPicking->setCheckState(Qt::Checked);
  } else {
    m_ui->checkBoxPicking->setCheckState(Qt::Unchecked);
  }
  m_ui->doubleSpinBoxPicking->setValue(attentionalReport->cost_behaviors[1]);

  if(attentionalReport->active_behaviors[2] == 1) {
    m_ui->checkBoxReceiving->setCheckState(Qt::Checked);
  } else {
    m_ui->checkBoxReceiving->setCheckState(Qt::Unchecked);
  }
  m_ui->doubleSpinBoxReceiving->setValue(attentionalReport->cost_behaviors[2]);

  if(attentionalReport->active_behaviors[3] == 1) {
    m_ui->checkBoxGiving->setCheckState(Qt::Checked);
  } else {
    m_ui->checkBoxGiving->setCheckState(Qt::Unchecked);
  }
  m_ui->doubleSpinBoxGiving->setValue(attentionalReport->cost_behaviors[3]);

  if(attentionalReport->active_behaviors[4] == 1) {
    m_ui->checkBoxPlacing->setCheckState(Qt::Checked);
  } else {
    m_ui->checkBoxPlacing->setCheckState(Qt::Unchecked);
  }
  m_ui->doubleSpinBoxPlacing->setValue(attentionalReport->cost_behaviors[4]);

  m_ui->spinBoxHumanTarget->setValue(attentionalReport->human_target);

  m_ui->spinBoxObjectTarget->setValue(attentionalReport->object_target);

  m_ui->spinBoxActionChanged->setValue(attentionalReport->actionChange);

  switch(attentionalReport->current_behavior) {
    case 0:
    m_ui->labelBehaviorSelected->setText(QString("Avoid"));
    break;
  case 1:
    m_ui->labelBehaviorSelected->setText(QString("Picking"));
    break;
  case 2:
    m_ui->labelBehaviorSelected->setText(QString("Receiving"));
    break;
  case 3:
    m_ui->labelBehaviorSelected->setText(QString("Giving"));
    break;
  case 4:
    m_ui->labelBehaviorSelected->setText(QString("Placing"));
    break;
  default:
    m_ui->labelBehaviorSelected->setText(QString("NONE"));
  }

  switch(attentionalReport->executed_behavior) {
    case 0:
    m_ui->labelBehaviorExecuted->setText(QString("Avoid"));
    break;
  case 1:
    m_ui->labelBehaviorExecuted->setText(QString("Picking"));
    break;
  case 2:
    m_ui->labelBehaviorExecuted->setText(QString("Receiving"));
    break;
  case 3:
    m_ui->labelBehaviorExecuted->setText(QString("Giving"));
    break;
  case 4:
    m_ui->labelBehaviorExecuted->setText(QString("Placing"));
    break;
  default:
    m_ui->labelBehaviorExecuted->setText(QString("NONE"));
  }
  return;
}


void AttentionalWidget::updateOutput()
{
  ATTENTIONAL_OUTPUT_STR *attentionalOutput;



  if(m_pr->_attentionalOutputPoster->getPosterStuct((char *)(&m_pr->_attentionalOutputPosterStruct)) == false) {
    cout << " AttentionalWidget::UpdateOutput() getPosterStuct() ERROR " << endl;
    return;
  }
  attentionalOutput = &(m_pr->_attentionalOutputPosterStruct);

   m_ui->doubleSpinBoxOutputCost->setValue(attentionalOutput->cost);

  m_ui->spinBoxOutputHumanTarget->setValue(attentionalOutput->partner);

  m_ui->spinBoxOutputObjectTarget->setValue(attentionalOutput->object);

  m_ui->spinBoxOutputActionChanged->setValue(attentionalOutput->actionChange);

  switch(attentionalOutput->action) {
    case 0:
    m_ui->labelOutputBehaviorSelectet->setText(QString("Avoid"));
    break;
  case 1:
    m_ui->labelOutputBehaviorSelectet->setText(QString("Picking"));
    break;
  case 2:
    m_ui->labelOutputBehaviorSelectet->setText(QString("Receiving"));
    break;
  case 3:
    m_ui->labelOutputBehaviorSelectet->setText(QString("Giving"));
    break;
  case 4:
    m_ui->labelOutputBehaviorSelectet->setText(QString("Placing"));
    break;
  default:
    m_ui->labelOutputBehaviorSelectet->setText(QString("NONE"));
  }
}
