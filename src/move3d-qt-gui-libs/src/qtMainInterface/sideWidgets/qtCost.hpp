/*
 *  qtCost.h
 *  BioMove3D
 *
 *  Created by Jim Mainprice on 01/04/10.
 *  Copyright 2010 LAAS/CNRS. All rights reserved.
 *
 */

#ifndef QT_COST_H
#define QT_COST_H

#ifdef HRI_COSTSPACE
#include "qtHrics.hpp"
#include "qtOtp.hpp"
#endif

#include "qtDistanceField.hpp"
#include "qtRRTStar.hpp"
#if defined(LIGHT_PLANNER) && defined(MULTILOCALPATH)
#include "qtReplanning.hpp"
#endif

#ifdef USE_QWT
#include "qtPlot/basicPlotWindow.hpp"
#endif

//#ifdef MIGHTABILITY_MAPS
#include "qtmightability.hpp"
//#endif

namespace Ui
{
class CostWidget;
};

class NaturalWidget;
class HriGestureWidget;

/**
 * @ingroup qtMainWindow
 * @brief Qt Motion Planner
 */
class CostWidget : public QWidget
{
    Q_OBJECT

public:
    CostWidget(QWidget *parent = 0);
    ~CostWidget();

    void initCost();
    void initCostFunctions();
    void resetCostFunctions();
    void initThreshold();

    void setMainWindow(MainWindow *ptrMW);
    void setMotionWidget(MotionPlanner* ptrMLPW) { m_motionWidget = ptrMLPW; }

#ifdef HRI_COSTSPACE
    HricsWidget* getHriWidget();
    OtpWidget* getOtpWidget();
    HriGestureWidget* getGestureWidget();
#endif
#if defined(LIGHT_PLANNER) && defined(MULTILOCALPATH)
    ReplanningWidget* getReplanningWidget();
#endif
    DistFieldWidget* getDistFieldWidget();

public slots:
    void setCostFunction(std::string function);
    void setCostFunction(int costFunctionId);
    void initCostSpace();

private slots:

    // General Cost --------------------------------
    void stonesGraph();
    void extractBestPath();
    void newGraphAndReComputeCost();
    void showTrajCost();
    void showCostProfile();
    void showHRITrajCost();
    void showSTOMPTrajCost();
    void showTemperature();
    void setPlotedVector(std::vector<double> v);
    void putGridInGraph();
    void computeAStar();
    //void computeGridAndExtract();
    void graphSearchTest();
    void setCostCriterium(int);
    void setDistanceCriterium(int);
    void envUseTRRTValueChanged( bool state );
    void envIsCostSpaceValueChanged( bool state );

private:
    Ui::CostWidget*		m_ui;

    MotionPlanner*	m_motionWidget;
    MainWindow*			m_mainWindow;

#ifdef USE_QWT
    BasicPlotWindow *m_plot;
#endif

#ifdef HRI_COSTSPACE
    HricsWidget* m_tabHri;
    OtpWidget* m_tabOtp;
    NaturalWidget* m_tabNatural;
    HriGestureWidget* m_tabGesture;
#endif
#if defined(LIGHT_PLANNER) && defined(MULTILOCALPATH)
    ReplanningWidget* m_tabReplan;
#endif

    RRTStarWidget*   m_tabRRTStar;

#ifdef MIGHTABILITY_MAPS
    qtMightability* m_tabMightabiliby;
#endif

};

#endif
