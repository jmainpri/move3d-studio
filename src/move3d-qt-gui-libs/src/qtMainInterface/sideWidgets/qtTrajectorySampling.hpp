#ifndef QT_TRAJECTORY_SAMPLING_HPP
#define QT_TRAJECTORY_SAMPLING_HPP

#include "qtLibrary.hpp"
#include "qtMainInterface/mainwindow.hpp"
#include "qtMainInterface/sideWidgets/qtMotionPlanner.hpp"

namespace Ui
{
class TrajectorySamplingWidget;
};

/**
 * @ingroup qtMainWindow
 * @brief Qt Motion Planner
 */
class TrajectorySamplingWidget : public QWidget
{
    Q_OBJECT

public:

    TrajectorySamplingWidget(QWidget *parent = 0);
    ~TrajectorySamplingWidget();

    void init();
    void setMainWindow(MainWindow *ptrMW) { m_mainWindow = ptrMW; }

private:

    MainWindow*                             m_mainWindow;
    Ui::TrajectorySamplingWidget*           m_ui;
};

#endif
