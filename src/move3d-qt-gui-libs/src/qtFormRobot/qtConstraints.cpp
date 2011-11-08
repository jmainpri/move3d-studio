/*
 *  qtMultiLocalPath.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 05/10/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "qtConstraints.hpp"
#include "moverobot.hpp"

#include "API/Device/robot.hpp"

#include "P3d-pkg.h"
#include "Localpath-pkg.h"

//////////////////////////////////////////////////////////////////////////////

ConstraintsWidget::ConstraintsWidget(QWidget *parent, FormRobot* fr )
{
  m_currentRobot = fr->getRobot()->getRobotStruct();
  m_form = fr;
	init();
  
  setAttribute(Qt::WA_DeleteOnClose, true);
}

void ConstraintsWidget::init()
{
	m_verticalLayout = new QVBoxLayout(this);
	m_verticalLayout->setSpacing(6);
	m_verticalLayout->setContentsMargins(3, 3, 3, 3);
	
	this->createCntrts();
	
	m_deactivButton = new QPushButton();
	m_deactivButton->setText("Deactivate");
	
	m_verticalLayout->addWidget( m_deactivButton );
	
	connect( m_deactivButton , SIGNAL(clicked()), this , SLOT(deactivateButton()) );
}

void ConstraintsWidget::deactivateButton()
{
  for(int i=0; i<m_currentRobot->cntrt_manager->ncntrts; i++)
	{
    m_Cntrts_CheckBoxes[i]->setChecked(false);
  }
}

void ConstraintsWidget::createCntrts() 
{
	QGroupBox* groupBox = new QGroupBox();
	groupBox->setTitle("Constraints");
	
	QVBoxLayout* groupLayout = new QVBoxLayout(groupBox);
	groupLayout->setSpacing(6);
	groupLayout->setContentsMargins(11, 11, 11, 11);
	
  // over all constraints
	for(int i=0; i<m_currentRobot->cntrt_manager->ncntrts; i++) 
	{
		m_Cntrts_CheckBoxes.push_back( new QCheckBox(groupBox) );
    
    // get constraint from the cntrts manager
    p3d_cntrt* ct = m_currentRobot->cntrt_manager->cntrts[i];
    
    std::string cntName = ct->namecntrt;
    std::string jntName = ct->pasjnts[0]->name;
    
    std::string str =  cntName + " ( " + jntName + " )";
    
		m_Cntrts_CheckBoxes.back()->setText( str.c_str() );
    
		groupLayout->addWidget( m_Cntrts_CheckBoxes.back() );
		
		m_Cntrts_Validator.push_back( new CntrtsValidator( 0, m_form, ct ) );
		
		if( ct->active == 1) 
		{
			m_Cntrts_CheckBoxes.back()->setChecked(true);
		} 
		else 
		{
			m_Cntrts_CheckBoxes.back()->setChecked(false);
		}

		connect(	m_Cntrts_CheckBoxes.back() , SIGNAL(toggled(bool)), 
							m_Cntrts_Validator.back() ,	SLOT(checkCntrts(bool)) );
	}
	
	m_verticalLayout->addWidget( groupBox );
}

//------------------------------------------------------------------------
// Validator for one constraint
//------------------------------------------------------------------------
CntrtsValidator::CntrtsValidator(QWidget *parent, FormRobot* fr, p3d_cntrt* cntrt)
{
	m_currentRobot = fr->getRobot()->getRobotStruct();;
  m_cntrt = cntrt;
  m_form = fr;
}


void CntrtsValidator::checkCntrts(bool value) 
{
  if (value) {
    p3d_activateCntrt( m_currentRobot, m_cntrt );
  }
  else {
    p3d_desactivateCntrt( m_currentRobot, m_cntrt );
  }
  m_form->resetConstraintedDoFs();
}


//------------------------------------------------------------------------
// Validator for one DoF constraint
//------------------------------------------------------------------------
CntrtDoFWidget::CntrtDoFWidget(QWidget *parent, FormRobot* fr, int numDof) :   m_numDof( numDof ) 
{
  m_currentRobot = fr->getRobot()->getRobotStruct();;
  m_form = fr;
  
  m_jnt = p3d_robot_dof_to_jnt(m_currentRobot, m_numDof, &m_jntDoFId);

	init();
}

void CntrtDoFWidget::init()
{
  m_verticalLayout = new QVBoxLayout(this);
	m_verticalLayout->setSpacing(6);
	m_verticalLayout->setContentsMargins(3, 3, 3, 3);
  
	QGroupBox* groupBox = new QGroupBox();
	groupBox->setTitle("Groups");
	
	QVBoxLayout* groupLayout = new QVBoxLayout(groupBox);
	groupLayout->setSpacing(6);
	groupLayout->setContentsMargins(11, 11, 11, 11);
  
  //-------------------------------------------------------------------------
  m_Cntrts_CheckBoxes.push_back( new QCheckBox(groupBox) );
  m_Cntrts_CheckBoxes.back()->setText("Jnt is active for planner");
  groupLayout->addWidget( m_Cntrts_CheckBoxes.back() );
  
  if( p3d_jnt_get_is_active_for_planner(m_jnt) == 1) 
  {
    m_Cntrts_CheckBoxes.back()->setChecked(true);
  } 
  else {
    m_Cntrts_CheckBoxes.back()->setChecked(false);
  }
  
  connect(	m_Cntrts_CheckBoxes.back() , SIGNAL(toggled(bool)), 
          this ,	SLOT(setJntActiveForPlanner(bool)) );
  
  //-------------------------------------------------------------------------
  m_Cntrts_CheckBoxes.push_back( new QCheckBox(groupBox) );
  m_Cntrts_CheckBoxes.back()->setText("Dof is active for planner");
  groupLayout->addWidget( m_Cntrts_CheckBoxes.back() );
  
  if( p3d_jnt_get_dof_is_active_for_planner(m_jnt,m_jntDoFId) == 1) 
  {
    m_Cntrts_CheckBoxes.back()->setChecked(true);
  } 
  else {
    m_Cntrts_CheckBoxes.back()->setChecked(false);
  }
  
  connect(	m_Cntrts_CheckBoxes.back() , SIGNAL(toggled(bool)), 
          this ,	SLOT(setActiveForPlanner(bool)) );
  
  //-------------------------------------------------------------------------
  m_Cntrts_CheckBoxes.push_back( new QCheckBox(groupBox) );
  m_Cntrts_CheckBoxes.back()->setText("Dof is user");
  groupLayout->addWidget( m_Cntrts_CheckBoxes.back() );
  
  if( p3d_jnt_get_dof_is_user(m_jnt,m_jntDoFId) == 1 ) 
  {
    m_Cntrts_CheckBoxes.back()->setChecked(true);
  } 
  else {
    m_Cntrts_CheckBoxes.back()->setChecked(false);
  }
  
  connect(	m_Cntrts_CheckBoxes.back() , SIGNAL(toggled(bool)), 
          this ,	SLOT(setUser(bool)) );
  
  //-------------------------------------------------------------------------
  if( m_currentRobot->cntrt_manager->in_cntrt[m_numDof] == 1 || 
     m_currentRobot->cntrt_manager->in_cntrt[m_numDof] == 2 )
  {
    m_Cntrts_CheckBoxes.push_back( new QCheckBox(groupBox) );
    m_Cntrts_CheckBoxes.back()->setText("Dof Active Cntrts");
    groupLayout->addWidget( m_Cntrts_CheckBoxes.back() );
    
    if( m_currentRobot->cntrt_manager->in_cntrt[m_numDof] == 1) 
    {
      m_Cntrts_CheckBoxes.back()->setChecked(true);
    } 
    else {
      m_Cntrts_CheckBoxes.back()->setChecked(false);
    }
  }
  //-------------------------------------------------------------------------
	
	m_verticalLayout->addWidget( groupBox );
}

// Set User DoF
void CntrtDoFWidget::setUser(bool set)
{
  p3d_jnt_set_dof_is_user(m_jnt,m_jntDoFId,set);
  m_form->resetConstraintedDoFs();
}

// Set Is Active For Planner
void CntrtDoFWidget::setActiveForPlanner(bool set)
{
  p3d_jnt_set_dof_is_active_for_planner(m_jnt,m_jntDoFId,set);
  m_form->resetConstraintedDoFs();
}

// Set all joint active
void CntrtDoFWidget::setJntActiveForPlanner(bool set)
{
  p3d_jnt_set_is_active_for_planner(m_jnt,set);
  m_form->resetConstraintedDoFs();
}
