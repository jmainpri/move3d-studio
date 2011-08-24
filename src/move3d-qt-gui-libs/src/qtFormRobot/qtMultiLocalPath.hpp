/*
 *  qtMultiLocalPath.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 05/10/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef QT_MULTILOCALPATH_H
#define QT_MULTILOCALPATH_H

#if defined( MOVE3D_CORE )

#include "qtLibrary.hpp"
#include "qtMainInterface/mainwindow.hpp"

#endif

/**
 * Iner class for validating group
 */
class GroupValidator : public QObject
{
	Q_OBJECT
	
public:
	
	GroupValidator(QWidget *parent = 0, p3d_rob* rob = NULL);
	~GroupValidator();
	
	public slots:
	
	void multiLocalPathList_obj(bool value);
	
private:
  p3d_rob*                        m_currentRobot;
	int                             m_group;
};

/**
 * @ingroup qtMainWindow
 * @brief Qt Util Widget to be changed by user
 */
class MultiLocalPathWidget : public QWidget
{
	Q_OBJECT
	
public:
	MultiLocalPathWidget(QWidget *parent = 0, p3d_rob* rob = NULL);
	~MultiLocalPathWidget();
	
	void initMultiLocalPathForm();
	void createMultiLocalPathList_obj();
	void setMainWindow(MainWindow *ptrMW) { m_mainWindow = ptrMW; }

public slots:
	
	void deactivateButton();
	
private:
  p3d_rob*                        m_currentRobot;
	MainWindow*											m_mainWindow;
	QVBoxLayout*										m_verticalLayout;
	QPushButton*										m_deactivButton;
	
	std::vector<QCheckBox*>					m_MLP_CheckBoxes;
	std::vector<GroupValidator*>		m_MLP_Validator;
};

#endif
