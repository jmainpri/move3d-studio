#ifndef ENVIRONMENTSTRUCT_H
#define ENVIRONMENTSTRUCT_H

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

#endif // ENVIRONMENTSTRUCT_H
