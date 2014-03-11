/*
 *  qtMultiLocalPath.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 05/10/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef QT_CONSTRAINTS_HPP
#define QT_CONSTRAINTS_HPP

#include "qtLibrary.hpp"
#include "cntrt.h"

#ifndef MOVEROBOT_HPP
class FormRobot;
#endif

#include "API/Device/robot.hpp"
#include <QtCore/QSettings>

void qt_saveCntrts( Move3D::Robot* rob, QSettings& settings);
void qt_loadCntrts( Move3D::Robot* rob, QSettings& settings);

/**
 * Iner class for validating group
 */
class CntrtsValidator : public QObject
{
	Q_OBJECT
	
public:
	
	CntrtsValidator(QWidget *parent = 0, FormRobot* FR = NULL, p3d_cntrt* c = NULL);
	~CntrtsValidator() { }
	
	public slots:
	
	void checkCntrts(bool value);
	
private:
  p3d_rob*                        m_currentRobot;
  p3d_cntrt*                      m_cntrt;
  FormRobot*                      m_form;
};

/**
 * @ingroup qtMainWindow
 * @brief Qt Util Widget to be changed by user
 */
class ConstraintsWidget : public QWidget
{
	Q_OBJECT
	
public:
	ConstraintsWidget(QWidget *parent = 0, FormRobot* FR = NULL);
	~ConstraintsWidget() { } 
	
	void init();
	void createCntrts();

public slots:
	
	void deactivateButton();
	
private:
  p3d_rob*                        m_currentRobot;
	FormRobot*                      m_form;
	QVBoxLayout*										m_verticalLayout;
	QPushButton*										m_deactivButton;
	
	std::vector<QCheckBox*>					m_Cntrts_CheckBoxes;
	std::vector<CntrtsValidator*>		m_Cntrts_Validator;
};

/**
 * @ingroup qtMainWindow
 * @brief Qt Util Widget to be changed by user
 */
class CntrtDoFWidget : public QWidget
{
	Q_OBJECT
	
public:
	CntrtDoFWidget(QWidget *parent = 0, FormRobot* FR = NULL, int numDof = 0);
	~CntrtDoFWidget() { }
	
	void init();
  
public slots:
	void setUser(bool set);
  void setActiveForPlanner(bool set);
  void setJntActiveForPlanner(bool set);
  
private:
  p3d_rob*                        m_currentRobot;
  p3d_jnt*                        m_jnt;
  int                             m_jntDoFId;
  int                             m_numDof;
  
    QVBoxLayout*				  m_verticalLayout;
  FormRobot*                      m_form;
	
	std::vector<QCheckBox*>					m_Cntrts_CheckBoxes;
};

#endif
