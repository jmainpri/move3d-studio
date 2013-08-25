/*
 *  qtCost.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef QT_HRIGESTURE_H
#define QT_HRIGESTURE_H

#include "qtLibrary.hpp"
#include "qtMainInterface/mainwindow.hpp"
#include "qtMainInterface/sideWidgets/qtMotionPlanner.hpp"
#include "qtFormRobot/qtmovinghuman.hpp"

#include "API/Grids/gridsAPI.hpp"

namespace Ui
{
class HriGestureWidget;
};

/**
 * @ingroup qtMainWindow
 * @brief Qt Motion Planner
 */
class HriGestureWidget : public QWidget
{
    Q_OBJECT

public:

    HriGestureWidget(QWidget *parent = 0);
    ~HriGestureWidget();
    void setMainWindow(MainWindow *ptrMW) { m_mainWindow = ptrMW; }
    void drawAllWinActive();

signals:

    void selectedPlanner(QString);

private slots:

    // Record Motion
    void initRecordedMotion();
    void loadRecordedMotion();
    void showRecordedMotion();
    void getIthConfigurationInMotion();
    void setSourceRM();
    void setTargetRM();
    void extractAndSaveRM();
    void loadFolder();
    void convertFolderToCSV();
    void loadFromCSV();

    // Occupancy
    void initWorkspaceOccupancy();
    void computeWorkspaceOccupancy();
    void setClassToDraw(int id);
    void classifyMotion();
    void startGestureSimulation();

    // Gesture Recognition
    void initGestureRecognition();

    // IOC
    void initHriIOC();
    void samplemap();

    // Legible
    void initLegibleCost();

private:

    int m_id_source_rm;
    int m_id_target_rm;
    int m_saved_file_id;

    MotionPlanner*                          m_motionWidget;
    MainWindow*                             m_mainWindow;

    Ui::HriGestureWidget*                        m_ui;
};

#endif
