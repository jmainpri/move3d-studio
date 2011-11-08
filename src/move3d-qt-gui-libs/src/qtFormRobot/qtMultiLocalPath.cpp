/*
 *  qtMultiLocalPath.cpp
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 05/10/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#include "qtMultiLocalPath.hpp"

#include "P3d-pkg.h"
#include "Localpath-pkg.h"

//////////////////////////////////////////////////////////////////////////////
                                
//------------------------------------------------------------------------
// Validator for one localpath
//------------------------------------------------------------------------
//p3d_multiLocalPath_disable_all_groupToPlan( m_robot->getRobotStruct() , false );
//p3d_multiLocalPath_set_groupToPlan( m_robot->getRobotStruct(), m_UpBodyMLP, 1, false);

GroupValidator::GroupValidator( QWidget *parent, p3d_rob* rob, int group ) : 
m_currentRobot(rob) ,
m_group(group)
{
  
}

GroupValidator::~GroupValidator()
{
	
}

void GroupValidator::multiLocalPathList_obj(bool value) 
{
 	p3d_multiLocalPath_set_groupToPlan( m_currentRobot, m_group , value, false );
  p3d_multilocapath_print_group_info( m_currentRobot );
}

//------------------------------------------------------------------------
// Validator for all localpaths
//------------------------------------------------------------------------
MultiLocalPathWidget::MultiLocalPathWidget(QWidget *parent, p3d_rob* rob) : m_currentRobot(rob)
{
	initMultiLocalPathForm();
  
  setAttribute(Qt::WA_DeleteOnClose, true);
}

MultiLocalPathWidget::~MultiLocalPathWidget()
{

}

//! initialisation
void MultiLocalPathWidget::initMultiLocalPathForm()
{
	m_verticalLayout = new QVBoxLayout(this);
	m_verticalLayout->setSpacing(6);
	m_verticalLayout->setContentsMargins(3, 3, 3, 3);
	
	this->createMultiLocalPathList_obj();
	
	m_deactivButton = new QPushButton();
	m_deactivButton->setText("Deactivate");
	
	m_verticalLayout->addWidget( m_deactivButton );
	
	connect( m_deactivButton , SIGNAL(clicked()), this , SLOT(deactivateButton()) );
}

void MultiLocalPathWidget::deactivateButton()
{
  p3d_multiLocalPath_disable_all_groupToPlan( m_currentRobot , false );
	
  for(int i=0; i<m_currentRobot->mlp->nblpGp; i++)
	{
    m_MLP_CheckBoxes[i]->setChecked(false);
  }
}

void MultiLocalPathWidget::createMultiLocalPathList_obj() 
{
	QGroupBox* groupBox = new QGroupBox();
	groupBox->setTitle("Groups");
	
	QVBoxLayout* groupLayout = new QVBoxLayout(groupBox);
	groupLayout->setSpacing(6);
	groupLayout->setContentsMargins(11, 11, 11, 11);
	
	for(int i=0; i<m_currentRobot->mlp->nblpGp; i++) 
	{
		m_MLP_CheckBoxes.push_back( new QCheckBox(groupBox) );
		m_MLP_CheckBoxes.back()->setText( m_currentRobot->mlp->mlpJoints[i]->gpName );
		groupLayout->addWidget( m_MLP_CheckBoxes.back() );
    
		m_MLP_Validator.push_back( new GroupValidator( 0, m_currentRobot, i ) );
		
		if( p3d_multiLocalPath_get_value_groupToPlan( m_currentRobot, i) == 1) 
			m_MLP_CheckBoxes.back()->setChecked(true);
		else 
			m_MLP_CheckBoxes.back()->setChecked(false);

		connect(	m_MLP_CheckBoxes.back() , SIGNAL(toggled(bool)), 
							m_MLP_Validator.back() ,	SLOT(multiLocalPathList_obj(bool)) );
	}
	
	m_verticalLayout->addWidget( groupBox );
}

