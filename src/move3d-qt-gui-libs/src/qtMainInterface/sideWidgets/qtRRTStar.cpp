//
//  qtRRTStar.cpp
//  move3d-studio
//
//  Created by Jim Mainprice on 28/07/12.
//  Copyright (c) 2012 LAAS/CNRS. All rights reserved.
//

#include "qtRRTStar.hpp"
#include "ui_qtRRTStar.h"

#include "qtBase/SpinBoxSliderConnector_p.hpp"

#include "planner/planEnvironment.hpp"

using namespace QtShiva;

RRTStarWidget::RRTStarWidget(QWidget *parent) :
QWidget(parent),
m_ui(new Ui::RRTStarWidget)
{
	m_ui->setupUi(this);
  
  init();
}

RRTStarWidget::~RRTStarWidget()
{
	delete m_ui;
}

void RRTStarWidget::init()
{
  new connectCheckBoxToEnv( m_ui->checkBoxRRTstar,	PlanEnv->getObject(PlanParam::starRRT));
  new connectCheckBoxToEnv( m_ui->checkBoxWithRewire,	PlanEnv->getObject(PlanParam::starRewire));
  
  // Radius
  new SpinBoxSliderConnector( this, m_ui->doubleSpinBoxRadius, m_ui->horizontalSliderRadius, PlanEnv->getObject(PlanParam::starRadius));
  new SpinBoxSliderConnector( this, m_ui->doubleSpinBoxGoal, m_ui->horizontalSliderGoal, PlanEnv->getObject(PlanParam::starFinish));
}

//---------------------------------------------------------------------
// RRT*
//---------------------------------------------------------------------
//void UtilWidget::initRRTstar()
//{
//	new connectCheckBoxToEnv(m_ui->checkBoxRRTstar,	Env::costStarRRT);
//}
