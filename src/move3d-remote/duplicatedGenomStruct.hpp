/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
#ifndef DUPLICATEDGENOMSTRUCT_HPP
#define DUPLICATEDGENOMSTRUCT_HPP

#include "genBasic/genBasicStruct.h"

#ifdef ATTENTIONAL_REMOTE
#include "attentional/attentionalStruct.h"
#endif
//----------------------------------------------------
// SPARK
//----------------------------------------------------
#define M3D_MAX_DOF 65
#define SPARK_MAX_AGENT_NB 7
#define SPARK_MAX_FREEFLYER_NB 30
#define NB_MAX_TRAJ 100
#define SPARK_NUM_SPHERES_MAX 10

//typedef struct GEN_STRING64 {
//  char name[64];
//} GEN_STRING64;

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

#ifndef _POMSTRUCT_H
#define _POMSTRUCT_H

/* --- Constants and enumerations ------------------------------------ */

/** Maximum length of strings found in PoM interface */
#define POM_MAX_STRLEN		512

typedef enum POM_FRAME {
  POM_FRAME_SENSOR,
  POM_FRAME_ROBOT,	/* also known as "main frame" */
  POM_FRAME_BASE,	/* also known as "odometry frame" */
  POM_FRAME_ORIGIN	/* global reference frame */
} POM_FRAME;

typedef enum POM_ACTION {
   POM_START, POM_STOP, POM_QUERY_MODE
} POM_ACTION;

typedef struct POM_STRING {
   char name[POM_MAX_STRLEN];
} POM_STRING;

typedef struct POM_STRING_2 {
   char name1[POM_MAX_STRLEN];
   char name2[POM_MAX_STRLEN];
} POM_STRING_2;


/* Motion estimator parameters --------------------------------------- */

typedef enum POM_FUSION_FLAG {
  POM_READ_ONLY, POM_FUSE
} POM_FUSION_FLAG;

typedef struct POM_ME_CREATE_DATA {
  char name[POM_MAX_STRLEN];
  POM_FUSION_FLAG fuseFlag;
  int unused;

#define POMDEF_ME_NAME		""
#define POMDEF_ME_FUSE		POM_FUSE

} POM_ME_CREATE_DATA;

/* Sensor estimator parameters --------------------------------------- */

typedef struct POM_SE_CREATE_DATA {
  char nameposter[POM_MAX_STRLEN];
  char namese[POM_MAX_STRLEN];
  char namefrom[POM_MAX_STRLEN];
  char nameto[POM_MAX_STRLEN];

#define POMDEF_SE_POSTER	""
#define POMDEF_SE_NAME		""
#define POMDEF_SE_FROM		""
#define POMDEF_SE_TO		""

} POM_SE_CREATE_DATA;

/* Reference position ------------------------------------------------ */

typedef struct POM_SET_POS {
   char frame[POM_MAX_STRLEN];	/* internal frame for position */
   double yaw;					/* position (euler) */
   double pitch;
   double roll;
   double x;
   double y;
   double z;

#define POMDEF_POS_FRAME	"Robot"
#define POMDEF_POS_YAW		0.0
#define POMDEF_POS_PITCH	0.0
#define POMDEF_POS_ROLL		0.0
#define POMDEF_POS_X		0.0
#define POMDEF_POS_Y		0.0
#define POMDEF_POS_Z		0.0

} POM_SET_POS;

/* --- Fusion Methods ------------------------------------------------ */

/* Class of fusion methods */
typedef enum POM_FUSION_METHOD {
  POM_FUSION_KALMAN, POM_FUSION_MAX_CONF, POM_FUSION_BAYES
} POM_FUSION_METHOD;

/* --- Euler Pos ------------------------------------------------------ */

typedef struct POM_EULER {
  double yaw, pitch, roll;
  double x, y, z;
} POM_EULER;

typedef struct POM_EULER_VARIANCES {
  double voo;                           /*  0 */
  double vop, vpp;                      /*  1,  2 */
  double vor, vpr, vrr;                 /*  3,  4,  5 */
  double vox, vpx, vrx, vxx;            /*  6,  7,  8,  9 */
  double voy, vpy, vry, vxy, vyy;       /* 10, 11, 12, 13, 14 */
  double voz, vpz, vrz, vxz, vyz, vzz;  /* 15, 16, 17, 18, 19, 20 */
} POM_EULER_VARIANCES;

typedef struct POM_EULER_V {
  POM_EULER euler;
  POM_EULER_VARIANCES var;
} POM_EULER_V;

typedef struct POM_UNC {
  double utheta,uphi,upsi,ux,uy,uz;
} POM_UNC;


/* --- Sensor Pos ------------------------------------------------ */

typedef struct POM_SENSOR_POS {
  int	date;		/* time of acquisition */
  int	pad;

  POM_EULER_V sensorToMain;	/* transformation from the sensor's frame to the
                         * main robot's frame */

  POM_EULER_V mainToBase;	/* transformation  from the robot's main frame to
                         * the current local frame */
  POM_EULER_V mainToOrigin;	/* transformation  from the robot's main frame to
                                 * the global absolute frame */

  POM_EULER_V VLocal;  /* Local velocities */

} POM_SENSOR_POS;


/** Poster exported by sensor estimators */
typedef struct POM_SE_POSTER {
  char seName[POM_MAX_STRLEN];
  POM_EULER_V seConfig;
} POM_SE_POSTER;


/* --- ME Pos ------------------------------------------------ */

typedef enum POM_ME_CLASS {
   POM_ME_ABSOLUTE, POM_ME_DELTA
} POM_ME_CLASS;

/* Structure that must be produced by a motion estimator */
typedef struct POM_ME_POS {
  POM_ME_CLASS kind;	/* kind of position estimation method */
  int unused;
  double confidence;	/* must be in [0-1] */
  int date1;
  int date2; /* for delta MEs  / xxx to disapear */
  POM_EULER_V main; /* mainToOrigin for absolute MEs, current to previous for delta Mes */
  POM_EULER_V VLocal;
} POM_ME_POS;


/* Similar to previous one but without the uncertainties */
typedef struct POM_EULER_POS {
  int date;
  int unused;
  double confidence;	/* must be in [0-1] */
  POM_EULER main;       /* mainToOrigin */
  POM_EULER VLocal;
} POM_EULER_POS;


/* --- Display params ------------------------------------------------ */

/* typedef struct PomDisplayParams { */
/*    char server[POM_MAX_STRLEN]; */

/* #define POMDEF_SERVER	"farabi" */

/* } PomDisplayParams; */


typedef struct POM_POS {
  int date;
  int pomTickDate; /* for now date = pomTickDate */
  POM_EULER_V mainToOrigin;
  POM_EULER_V mainToBase;
  POM_EULER_V VLocal;
} POM_POS;


typedef struct POM_OBJECT_POS {
  POM_FRAME frame;
  int confidence;      /* 0: not to be used  1: best */
  POM_EULER_V pos;
  POM_EULER_V speed;
  POM_SENSOR_POS sensorPos; /* for now sensor on platine */
} POM_OBJECT_POS;


#endif /* _POMSTRUCT_H */

/* --- monitoring spheres ------------------------------------------------ */

typedef struct STRUCT_SPARK_3D_COORD {
  double x;
  double y;
  double z;
} SPARK_3D_COORD;


typedef enum ENUM_SPARK_SPHERE_TYPE {
  SPARK_SIMPLE_ENTRY = 0, // simple sphere defined through radius and center. To monitor entrance in the sphere;
  SPARK_SIMPLE_EXIT = 1,  // simple sphere defined through radius and center. To monitor exit from the sphere.
  SPARK_THROW_IN_CONTAINER = 2, // simple sphere those radius and center is automatically defined as fitting above the container.
  SPARK_PICK_OBJECT =  3, // simple sphere those radius and center is automatically defined as around object to pick.
  SPARK_PERMANENT_STOP_MONITOR = 4  //
} SPARK_SPHERE_TYPE; // Monitor that hands are fixed at some points.

typedef struct STRUCT_SPARK_ACTION_MONITORING_SPHERE {
  int isSphereActive; // TRUE if monitor is active, FALSE otherwise.
  GEN_STRING64  agentName; // agent name whose action is monitored.
  int agentIndex; // agent index.
  GEN_STRING64  objectName; // Object name used to define sphere for certain sphereType
  int entityIndex; // entity index of the object
  int handIndexInput; // -1 if we don't want to precise hand index > -1 otherwise.
  SPARK_3D_COORD sphereCenter; //  sphere center.
  double sphereRadius; // sphere radius.
  double filteringTimeThreshold; // time delay to wait for monitor success condition to triger monitor success
  SPARK_SPHERE_TYPE sphereType; // what is the type of this sphere as far as creating it is concerned
  int monitorEnterInResult; // TRUE if monitor trigger for enter in spheres and FALSE otherwise.
  int monitorGetOutResult; // TRUE if monitor trigger for get out of spheres (if it was in) and FALSE otherwise.
  int handIndexResult; // Whose agent hands trigger monitor.
  int modifIndex; // each time there is something new on this sphere the index is incremented. This is used to update poster from move3d spheres.
  int dummy;
} SPARK_ACTION_MONITORING_SPHERE;

typedef struct STRUCT_SPARK_ALL_MONITORING_SPHERES {
  SPARK_ACTION_MONITORING_SPHERE spheres[SPARK_NUM_SPHERES_MAX];
  int modifIndex;  //each time there is something new in spheres the index is incremented. This is used to update poster from move3d spheres.
} SPARK_ALL_MONITORING_SPHERES;

//----------------------------------------------------
// MHP
//----------------------------------------------------

typedef struct POSE {
  double x;
  double y;
  double theta;
} POSE;

typedef struct TRAJ_POINTS {
  int nbPts;
  int dummy;
  POSE points[NB_MAX_TRAJ];
} TRAJ_POINTS;

typedef struct STRUCT_MHP_ROBOT_TO_DRAW {
  GEN_STRING64 robot_name;
  double q[M3D_MAX_DOF];
  TRAJ_POINTS traj;
} MHP_ROBOT_TO_DRAW;

typedef struct MHP_ROBOTFUTURPOS_POSTER_STR {
  MHP_ROBOT_TO_DRAW robot_to_draw;
} MHP_ROBOTFUTURPOS_POSTER_STR;


//----------------------------------------------------
// VIAM
//----------------------------------------------------
#define VIAM_ID_MAX	64
#define VIAM_ID_EMPTY	"-"

typedef struct ViamId {
  char id[VIAM_ID_MAX];
} ViamId;


typedef enum viam_caltype_t {
  VIAM_CAL_MONO,
  VIAM_CAL_STEREO
} viam_caltype_t;

typedef enum ViamIO {
  VIAM_LOAD,
  VIAM_SAVE
} ViamIO;

typedef enum ViamActivate {
  VIAM_DISABLE,
  VIAM_ENABLE
} ViamActivate;


typedef enum viam_distortionmodel_t {
  VIAM_DIST_R1T0,
  VIAM_DIST_R2T0,
  VIAM_DIST_R3T0,
  VIAM_DIST_R1T2,
  VIAM_DIST_R2T2,
  VIAM_DIST_R3T2
} viam_distortionmodel_t;


/* --- Image calibration --------------------------------------------- */

typedef struct ViamCalibrate {
  ViamId	bank;		/**< bank name */
  viam_caltype_t type;		/**< calibration type */
  viam_distortionmodel_t dmodel;/**< distortion model */
  int		gridrows;	/**< rows in the calibration grid */
  int		gridcols;	/**< columns in the calibration grid */
  double	gridsize;	/**< dimension of the grid squares */
  double	overlapth;	/**< minimal overlapping distance (pixels) */
  double	motionth;	/**< maximal chessboard motion (pixels) */
  double	precision;	/**< target average reprojection error */
  int		images;		/**< max number of images */
} ViamCalibrate;

typedef struct ViamCalibrationIO {
  ViamId	bank;		/**< bank name */
  ViamIO	op;		/**< I/O operation */
  char		file[1024];/**< file name of saved data */
} ViamCalibrationIO;

typedef struct viam_cameracalibration_t {
  double intrinsic[9];	/**< intrinsic matrix (homogeneous 3x3) */
  double intrirect[9];	/**< intrinsic matrix of rectified camera (homogeneous 3x3) */
  double distortion[5];	/**< distortion coeffs (1x5: r1, r2, r3, t1, t2) */
  double rectification[9];/**< rectification matrix (homogeneous 3x3) */
  double rotation[9];	/**< 3D rotation matrix from rectified to original (3x3) */

  int width;		/**< width of the calibration images */
  int height;		/**< height of the calibration images */
} viam_cameracalibration_t;

typedef struct viam_bankcalibration_t {
  viam_caltype_t type;	/**< MONO or STEREO */
  double baseline;	/**< baseline in meters */
  double pbaseline;	/**< baseline in pixels */
} viam_bankcalibration_t;

/* --- Image poster -------------------------------------------------- */

typedef struct ViamImageHeader {
  ViamId name;		/* camera name */

  unsigned long tacq_sec;	/** time of acquisition (seconds) */
  unsigned long tacq_usec;	/** time of acquisition (microseconds) */
  POM_SENSOR_POS pos;		/**< pom position */

  int nChannels;	/* 1 or 3 channels */
  int depth;		/* pixel depth in bits */
  int width;		/* image width in pixels */
  int height;		/* image height in pixels */
  int imageSize;	/* image size in bytes (image->height*image->widthStep) */
  int widthStep;	/* size of aligned image row in bytes */

  viam_cameracalibration_t calibration;

  unsigned long dataOffset;/* offset of first pixel in image data array below */
  unsigned char data[0];/* data: top-left origin, interleaved color channels */
} ViamImageHeader;

typedef struct ViamImageBank {
  ViamId name;				/* bank name */
  viam_bankcalibration_t calibration;	/* calibration data */
  int nImages;				/* number of images in bank */
  ViamImageHeader image[0];		/* actual size is nImages */
} ViamImageBank;

/* --- Niut poster -------------------------------------------------- */
//typedef struct GEN_POINT_3D {
//  double x;
//  double y;
//  double z;
//} GEN_POINT_3D;

///*
// * Joint position
// */
//typedef struct NIUT_JOINT_STR {
//	double confidence;
//	GEN_POINT_3D position;
//} NIUT_JOINT_STR;
//
///*
// * List of joints
// *
// * Ordered to be binary compatible with OpenNI.
// *
// * Only HEAD, NECK, TORSO, SHOULDERS, ELBOW, HAND, HIP, KNEE and FOOT
// * are used by the current version of the software.
// */
//typedef enum NIUT_JOINT {
//	NIUT_HEAD = 1,
//	NIUT_NECK = 2,
//	NIUT_TORSO = 3,
//	NIUT_WAIST = 4,
//
//	NIUT_LEFT_COLLAR = 5,
//	NIUT_LEFT_SHOULDER = 6,
//	NIUT_LEFT_ELBOW = 7,
//	NIUT_LEFT_WRIST = 8,
//	NIUT_LEFT_HAND = 9,
//	NIUT_LEFT_FINGERTIP = 10,
//
//	NIUT_RIGHT_COLLAR = 11,
//	NIUT_RIGHT_SHOULDER = 12,
//	NIUT_RIGHT_ELBOW = 13,
//	NIUT_RIGHT_WRIST = 14,
//	NIUT_RIGHT_HAND = 15,
//	NIUT_RIGHT_FINGERTIP = 16,
//
//	NIUT_LEFT_HIP = 17,
//	NIUT_LEFT_KNEE = 18,
//	NIUT_LEFT_ANKLE = 19,
//	NIUT_LEFT_FOOT = 20,
//
//	NIUT_RIGHT_HIP = 21,
//	NIUT_RIGHT_KNEE = 22,
//	NIUT_RIGHT_ANKLE = 23,
//	NIUT_RIGHT_FOOT = 24
//} NIUT_JOINT;
//
//#define NIUT_MAX_JOINTS 25
//
//typedef struct NIUT_SKELETON_STR {
//	NIUT_JOINT_STR joint[NIUT_MAX_JOINTS];
//} NIUT_SKELETON_STR;
//
//typedef enum NIUT_TRACK_STATE {
//	NIUT_NO_TRACKING,
//	NIUT_POSE_SEARCH,
//	NIUT_CALIBRATE,
//	NIUT_TRACKING
//} NIUT_TRACK_STATE;
//
//typedef struct NIUT_USER_STR {
//	int id;
//	NIUT_TRACK_STATE state;
//	NIUT_SKELETON_STR skeleton;
//} NIUT_USER_STR;
//
//#define NIUT_MAX_HUMANS 16
//
//typedef struct NIUT_HUMAN_LIST {
//  int watch_dog;
//	int num;
//	NIUT_USER_STR users[NIUT_MAX_HUMANS];
//} NIUT_HUMAN_LIST;

/*
 * time
 */
typedef struct NIUT_TIME_STR {
  unsigned int t_sec;
  unsigned int t_usec;
} NIUT_TIME_STR;


/*
 * Joint position
 */
typedef struct NIUT_JOINT_STR {
        double confidence;
        GEN_POINT_3D position;
} NIUT_JOINT_STR;

/*
 * List of joints
 *
 * Ordered to be binary compatible with OpenNI.
 *
 * Only HEAD, NECK, TORSO, SHOULDERS, ELBOW, HAND, HIP, KNEE and FOOT
 * are used by the current version of the software.
 */
typedef enum NIUT_JOINT {
        NIUT_HEAD = 1,
        NIUT_NECK = 2,
        NIUT_TORSO = 3,
        NIUT_WAIST = 4,

        NIUT_LEFT_COLLAR = 5,
        NIUT_LEFT_SHOULDER = 6,
        NIUT_LEFT_ELBOW = 7,
        NIUT_LEFT_WRIST = 8,
        NIUT_LEFT_HAND = 9,
        NIUT_LEFT_FINGERTIP = 10,

        NIUT_RIGHT_COLLAR = 11,
        NIUT_RIGHT_SHOULDER = 12,
        NIUT_RIGHT_ELBOW = 13,
        NIUT_RIGHT_WRIST = 14,
        NIUT_RIGHT_HAND = 15,
        NIUT_RIGHT_FINGERTIP = 16,

        NIUT_LEFT_HIP = 17,
        NIUT_LEFT_KNEE = 18,
        NIUT_LEFT_ANKLE = 19,
        NIUT_LEFT_FOOT = 20,

        NIUT_RIGHT_HIP = 21,
        NIUT_RIGHT_KNEE = 22,
        NIUT_RIGHT_ANKLE = 23,
        NIUT_RIGHT_FOOT = 24
} NIUT_JOINT;

#define NIUT_MAX_JOINTS 25

typedef struct NIUT_SKELETON_STR {
        NIUT_JOINT_STR joint[NIUT_MAX_JOINTS];
} NIUT_SKELETON_STR;

typedef enum NIUT_TRACK_STATE {
        NIUT_NO_TRACKING,
        NIUT_POSE_SEARCH,
        NIUT_CALIBRATE,
        NIUT_TRACKING
} NIUT_TRACK_STATE;

typedef struct NIUT_USER_STR {
        int id;
        NIUT_TIME_STR date_discovered;
        NIUT_TIME_STR date;
        NIUT_TRACK_STATE state;
        NIUT_SKELETON_STR skeleton;
} NIUT_USER_STR;

#define NIUT_MAX_HUMANS 16

typedef struct NIUT_HUMAN_LIST {
       int watch_dog;
       int num;
       NIUT_USER_STR users[NIUT_MAX_HUMANS];
       NIUT_USER_STR filtered_users[NIUT_MAX_HUMANS];
} NIUT_HUMAN_LIST;


#endif // DUPLICATEDGENOMSTRUCT_HPP
