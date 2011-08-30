/*
 *  pocolibsPoster.hpp
 *  Move3D-Qt-Gui
 *
 *  Created by Jim Mainprice on 28/01/11.
 *  Copyright 2011 LAAS/CNRS. All rights reserved.
 *
 */

#include "qtLibrary.hpp"

#include "mainwindow-remote.hpp"


#include <portLib.h>
#include <posterLib.h>

//----------------------------------------------------
// SPARK
//----------------------------------------------------
#define M3D_MAX_DOF 65
#define SPARK_MAX_AGENT_NB 7
#define SPARK_MAX_FREEFLYER_NB 30

typedef struct GEN_STRING64 {
  char name[64];
} GEN_STRING64;

typedef struct STRUCT_M3D_ROBOT {
  GEN_STRING64 name;
  double q[M3D_MAX_DOF];
  int length;
  int unused;
} M3D_ROBOT;

typedef struct STRUCT_M3D_FREEFLYER {
  GEN_STRING64 name;
  double q[6];
} M3D_FREEFLYER;

typedef struct STRUCT_SPARK_CURRENT_ENVIRONMENT {
  GEN_STRING64 envName;
  M3D_ROBOT robot[SPARK_MAX_AGENT_NB];
  M3D_FREEFLYER freeflyer[SPARK_MAX_FREEFLYER_NB];
  int robotNb;
  int freeflyerNb;
  int time;
  int unused;
} SPARK_CURRENT_ENVIRONMENT;

class FetchEnvironment : public QObject
{
  Q_OBJECT
  
public:
  //! Initializes a timer and 
  //! finds the emiting Environment Poster
  FetchEnvironment(QWidget* = NULL);
  
  //! Initializes the poster
  //! returns true if OK
  bool init(MainWindowRemote* win);

  //! Find the poster given a posterName
  //! returns true if OK and fill posterId
  bool findPoster(std::string str, POSTER_ID *posterId);
  bool getSparkStatus(){return _sparkStatus;}

public slots:
  void setSparkRefresh(bool checked);
  
protected slots:
  //! Reads a pocolib poster
  //! Sets every robot in Move3D to match the poster description
  //! @return true if the operation has succeded
  bool refresh();

private:
  //! Display functions
  MainWindowRemote* m_win;
  std::string _EnvPoster;
  POSTER_ID _EnvPosterID;
  bool _sparkRefrech;
  bool _sparkStatus;

};
