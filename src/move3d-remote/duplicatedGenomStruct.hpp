#ifndef DUPLICATEDGENOMSTRUCT_HPP
#define DUPLICATEDGENOMSTRUCT_HPP

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

#endif // DUPLICATEDGENOMSTRUCT_HPP
