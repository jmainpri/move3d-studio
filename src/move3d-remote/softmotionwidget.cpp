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
#include "softmotionwidget.hpp"
#include "ui_softmotionwidget.h"
#include <iostream>


#include "planner_handler.hpp"
#include "move3d-gui.h"
#include "API/scene.hpp"
#include "API/project.hpp"
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"

#include "qtBase/SpinBoxSliderConnector_p.hpp"

using namespace std;

extern bool drawTrajOnRemote;
extern bool drawMonitoringSpheresOnRemote;

softmotionWidget::softmotionWidget(QWidget *parent) :
        QWidget(parent),
        m_ui(new Ui::softmotionWidget)
{
    m_ui->setupUi(this);

}


softmotionWidget::~softmotionWidget()
{
    delete m_ui;
}



void softmotionWidget::init(PosterReader *pr, Ui::ParamWidget *ui_param)
{
  if( (!pr) || (!ui_param))
  {
    cout << "niutWidget not well initialized!!!" << endl;
    return;
  }
  m_pr = pr;
  m_ui_p = ui_param;
  connect(m_ui->pushButtonSMPlot,SIGNAL(clicked()),pr,SLOT(softmotionPlotTraj()));
  connect(m_ui->checkBoxSoftMotionDrawTraj, SIGNAL(toggled(bool)), pr , SLOT(softmotionDrawTraj(bool)), Qt::DirectConnection);
  connect(m_ui->checkBoxMonitoringSphere, SIGNAL(toggled(bool)), pr , SLOT(monitoringSpheresDraw(bool)), Qt::DirectConnection);

  QtShiva::SpinBoxSliderConnector *connectordt= new QtShiva::SpinBoxSliderConnector(
          this, m_ui->doubleSpinBoxdt, m_ui->horizontalSliderdt);

  QtShiva::SpinBoxSliderConnector *connectorOpacity= new QtShiva::SpinBoxSliderConnector(
          this, m_ui->doubleSpinBoxOpacity, m_ui->horizontalSliderOpacity);
  double initialOpacity = 0.5;
  connectorOpacity->setValue(initialOpacity);
  pr->changesOpacity(initialOpacity);

  connect(connectordt, SIGNAL(valueChanged(double)), pr, SLOT(changesoftmotiondt(double)));
  connect(connectorOpacity, SIGNAL(valueChanged(double)), pr, SLOT(changesOpacity(double)));
}



void softmotionWidget::on_checkBoxMonitoringSphere_toggled(bool checked)
{
    drawMonitoringSpheresOnRemote = checked;
}

void softmotionWidget::on_checkBoxSoftMotionDrawTraj_toggled(bool checked)
{
    drawTrajOnRemote = checked;
}
