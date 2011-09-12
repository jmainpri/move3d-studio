#ifndef MYOPENCV_HPP
#define MYOPENCV_HPP

 #include <QPainter>

#define IPL_DEPTH_1U 1
#define IPL_DEPTH_8U 8
#define IPL_DEPTH_16U 16
#define IPL_DEPTH_32F 32

#define IPL_DEPTH_8S (IPL_DEPTH_SIGN| 8)
#define IPL_DEPTH_16S (IPL_DEPTH_SIGN|16)
#define IPL_DEPTH_32S (IPL_DEPTH_SIGN|32)
#define IPL_DEPTH_64F 64 

//#ifndef VIAM_OPENCV_SUPPORT

/* Provide our simple replacement for the missing opencv structures and
 * functions when configured without opencv support. Note that the IplImage
 * provided here is not compatible with opencv's one. */

typedef struct _IplImage {
  int  nChannels;		/**< 1 or 3 channels */
  int  depth;			/**< pixel depth in bits */
  int  width;			/**< image width in pixels */
  int  height;			/**< image height in pixels */
  int  imageSize;		/**< image data size in bytes */
  unsigned char *imageData;	/**< pointer to aligned image data */
  int  widthStep;		/**< size of aligned image row in bytes */
} IplImage;

typedef struct CvSize {
  int width;
  int height;
} CvSize;

#define IPL_DEPTH_8U	8
#define IPL_DEPTH_16U	16

#define IPL_ORIGIN_TL	0

#define CV_AUTOSTEP	0x7fffffff

#define CV_LOAD_IMAGE_UNCHANGED  -1

CvSize		cvSize(int width, int height);
IplImage *	cvCreateImage(CvSize size, int depth, int channels);
IplImage *	cvCreateImageHeader(CvSize size, int depth, int channels);
IplImage *	cvInitImageHeader(IplImage *image, CvSize size, int depth,
                        int channels, int origin, int align);
void		cvSetData(struct _IplImage *image, void *data, int step);
int		cvSaveImage(const char *filename, const struct _IplImage *image);
IplImage *	cvLoadImage(const char* filename, int iscolor);
void		cvReleaseImage(IplImage **image);

//#endif /* !VIAM_OPENCV_SUPPORT */

#endif // MYOPENCV_HPP
