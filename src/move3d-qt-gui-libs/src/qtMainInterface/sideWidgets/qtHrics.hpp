/*
 *  qtCost.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef QT_HRICS_H
#define QT_HRICS_H

#include "qtLibrary.hpp"
#include "qtMainInterface/mainwindow.hpp"
#include "qtMainInterface/sideWidgets/qtMotionPlanner.hpp"
#include "qtFormRobot/qtmovinghuman.hpp"

#include "API/Grids/gridsAPI.hpp"

namespace Ui
{
class HricsWidget;
};

/**
 * @ingroup qtMainWindow
 * @brief Qt Motion Planner
 */
class HricsWidget : public QWidget
{
    Q_OBJECT

public:

    HricsWidget(QWidget *parent = 0);
    ~HricsWidget();

    void initHRI();
    void initVectorField();
    void initDrawOneLineInGrid();
    void initObjectTransferPoint();
    void initHumanLike();

    void setMainWindow(MainWindow *ptrMW) { m_mainWindow = ptrMW; }
    void setMotionWidget(MotionPlanner* ptrMLPW) { m_motionWidget = ptrMLPW; }
    void setGroupBoxDisabled(bool disable);

    void drawAllWinActive();

signals:
    void selectedPlanner(QString);

private slots:

    void on_checkBoxPosOr_toggled(bool checked);
    void set_X_line(bool);
    void set_Y_line(bool);
    void set_Z_line(bool);
    void on_pushButton_initColPos_clicked();

    // Vector Field
    void computeVectorField();

    // OTP
    void computeObjectTransferPoint();

    // Workspace
    void make3DHriGrid();
    void delete3DHriGrid();

    // Human costmaps
    void initGrids();
    void deleteGrids();
    void computeAllCellCost();
    void loadGrid();
    void saveGrid();

    void AStarIn3DGrid();
    void HRICSRRT();
    void zoneSizeChanged();
    void resetRandomPoints();
    void setWhichTestSlot(int);
    void computeOtpConfig();
    void computeAStarGrid();

    void initRecordedMotion();
    void loadRecordedMotion();
    void showRecordedMotion();
    void getIthConfigurationInMotion();
    void setSourceRM();
    void setTargetRM();
    void extractAndSaveRM();

private:

    int m_id_source_rm;
    int m_id_target_rm;
    int m_saved_file_id;

    QtShiva::SpinBoxSliderConnector*	m_k_distance;
    QtShiva::SpinBoxSliderConnector*  m_k_visbility;
    QtShiva::SpinBoxSliderConnector*  m_k_reachability;
    QtShiva::SpinBoxSliderConnector*  m_k_naturality;

    MotionPlanner*                          m_motionWidget;
    MainWindow*                             m_mainWindow;
    MovingHuman*                            m_mh;

    Ui::HricsWidget*                        m_ui;
};

#endif
