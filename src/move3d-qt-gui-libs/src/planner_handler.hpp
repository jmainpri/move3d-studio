#ifndef PLANNER_HANDLER_HPP_INCLUDED
#define PLANNER_HANDLER_HPP_INCLUDED

#include <QtCore/QObject>
#include <QtCore/QString>

#include "P3d-pkg.h"

#include <string>

// UI functions
void qt_add_traj(char* name, int id, p3d_rob* rob, p3d_traj* traj);
void qt_add_config_to_ui(char* name, p3d_rob* rob, double* q);

// Read/Write functions
void qt_readScenario();
void qt_saveScenario();
void qt_readTraj();
void qt_load_HRICS_Grid(std::string gridName);
void qt_init_after_params();

// Variable used for file reading
extern const char *qt_fileName;

// Planner Handler object
class PlannerHandler : public QObject
{
    Q_OBJECT;

public:
    enum state {
        running, // A planning algorithm instance is running
        stopped, // A planning algorithm instance has been created, but is stopped
        none }; // There is no instance created

    PlannerHandler(int argc, char** argv);

public slots:
    void init();
    void startPlanner(QString plannerName);
    void stopPlanner();
    void resetPlanner();

signals:
    void initIsDone();
    void plannerIsStopped();
    void plannerIsReset();

protected:
    state mState;
    int mArgc;
    char** mArgv;
};

extern PlannerHandler* global_plannerHandler;

#endif
