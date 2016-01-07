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
#include "qtmovinghuman.hpp"
#include "ui_qtmovinghuman.h"

#include "qtMainInterface/mainwindow.hpp"
//#include "qtMainInterface/mainwindowGenerated.hpp"
#include "planner_handler.hpp"

#include "API/project.hpp"
#include "API/Device/robot.hpp"

using namespace Move3D;

MovingHuman::MovingHuman(QWidget *parent)
    : QWidget(parent), ui(new Ui::MovingHuman) {
  ui->setupUi(this);
  // init(0,0,0,0);
  Robot *human =
      global_Project->getActiveScene()->getRobotByNameContaining("HERAKLES");
  if (human == NULL) {
    init(0, 0, 0, 0);
  } else {
    confPtr_t q = human->getCurrentPos();
    init((*q)[6], (*q)[7], (*q)[8], (*q)[11]);
  }
}

MovingHuman::MovingHuman(double x, double y, double rz, QWidget *parent)
    : QWidget(parent), ui(new Ui::MovingHuman) {
  ui->setupUi(this);
  init(x, y, 0, rz);
}

MovingHuman::~MovingHuman() {
  PlanEnv->setBool(PlanParam::env_drawHumanModel, false);
  delete ui;
}

void MovingHuman::init(double x, double y, double z, double rz) {
  std::vector<double> bounds = global_Project->getActiveScene()->getBounds();

  std::cout << "Set MovingHuman Bounds to : (";
  std::cout << bounds[0] << " , " << bounds[1] << " , " << bounds[2] << " , ";
  std::cout << bounds[3] << " , " << bounds[4] << " , " << bounds[5] << " )"
            << std::endl;

  ui->doubleSpinBoxX->setMinimum(bounds[0]);
  ui->doubleSpinBoxX->setMaximum(bounds[1]);
  ui->doubleSpinBoxY->setMinimum(bounds[2]);
  ui->doubleSpinBoxY->setMaximum(bounds[3]);
  ui->doubleSpinBoxZ->setMinimum(bounds[4]);
  ui->doubleSpinBoxZ->setMaximum(bounds[5]);

  m_k_x = new QtShiva::SpinBoxSliderConnector(
      this,
      ui->doubleSpinBoxX,
      ui->horizontalSliderX,
      PlanEnv->getObject(PlanParam::env_futurX));
  m_k_x->setValue(x);
  m_k_y = new QtShiva::SpinBoxSliderConnector(
      this,
      ui->doubleSpinBoxY,
      ui->horizontalSliderY,
      PlanEnv->getObject(PlanParam::env_futurY));
  m_k_y->setValue(y);
  m_k_z = new QtShiva::SpinBoxSliderConnector(
      this,
      ui->doubleSpinBoxZ,
      ui->horizontalSliderZ,
      PlanEnv->getObject(PlanParam::env_futurZ));
  m_k_z->setValue(z);
  m_k_rz = new QtShiva::SpinBoxSliderConnector(
      this,
      ui->doubleSpinBoxRZ,
      ui->horizontalSliderRZ,
      PlanEnv->getObject(PlanParam::env_futurRZ));
  m_k_rz->setValue(rz);

  connect(m_k_x,
          SIGNAL(valueChanged(double)),
          this,
          SLOT(updateMainWindow(double)));
  connect(m_k_y,
          SIGNAL(valueChanged(double)),
          this,
          SLOT(updateMainWindow(double)));
  connect(m_k_rz,
          SIGNAL(valueChanged(double)),
          this,
          SLOT(updateMainWindow(double)));
  //    connect(m_k_x,SIGNAL(valueChanged(double)),this,SLOT(printValues()));
  //    connect(m_k_y,SIGNAL(valueChanged(double)),this,SLOT(printValues()));
  //    connect(m_k_rz,SIGNAL(valueChanged(double)),this,SLOT(printValues()));

  PlanEnv->setBool(PlanParam::env_drawHumanModel, true);
  //    m_mainWindow->drawAllWinActive();
}

void MovingHuman::changeEvent(QEvent *e) {
  QWidget::changeEvent(e);
  switch (e->type()) {
    case QEvent::LanguageChange:
      ui->retranslateUi(this);
      break;
    default:
      break;
  }
}

void MovingHuman::updateMainWindow(double) { m_mainWindow->drawAllWinActive(); }

void MovingHuman::setX(double x) {
  PlanEnv->setDouble(PlanParam::env_futurX, x);
  m_k_x->setValue(x);
  m_mainWindow->drawAllWinActive();
}

void MovingHuman::setY(double y) {
  PlanEnv->setDouble(PlanParam::env_futurY, y);
  m_k_y->setValue(y);
  m_mainWindow->drawAllWinActive();
}

void MovingHuman::setRZ(double rz) {
  PlanEnv->setDouble(PlanParam::env_futurRZ, rz);
  m_k_rz->setValue(rz);
  m_mainWindow->drawAllWinActive();
}

void MovingHuman::printValues() {
  std::cout << "PlanEnv->getDouble(PlanParam::env_futurX) = "
            << PlanEnv->getDouble(PlanParam::env_futurX) << std::endl;
  std::cout << "PlanEnv->getDouble(PlanParam::env_futurY) = "
            << PlanEnv->getDouble(PlanParam::env_futurY) << std::endl;
  std::cout << "PlanEnv->getDouble(PlanParam::env_futurZ) = "
            << PlanEnv->getDouble(PlanParam::env_futurZ) << std::endl;
  std::cout << "PlanEnv->getDouble(PlanParam::env_futurRZ) = "
            << PlanEnv->getDouble(PlanParam::env_futurRZ) << std::endl;
}

double MovingHuman::getX() { return PlanEnv->getDouble(PlanParam::env_futurX); }

double MovingHuman::getY() { return PlanEnv->getDouble(PlanParam::env_futurY); }

double MovingHuman::getRZ() {
  return PlanEnv->getDouble(PlanParam::env_futurRZ);
}
