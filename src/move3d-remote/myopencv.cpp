#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <err.h>
#include "myopencv.hpp"
/* ------------------------------------------------------------------- */

/* Follows several opencv replacement functions for when
 * VIAM_OPENCV_SUPPORT is disabled */

//#ifndef VIAM_OPENCV_SUPPORT

CvSize
cvSize(int width, int height)
{
  return (CvSize){width, height};
}

IplImage *
cvCreateImage(CvSize size, int depth, int channels)
{
  IplImage *i = cvCreateImageHeader(size, depth, channels);
  cvSetData(i, malloc(i->imageSize), CV_AUTOSTEP);

  if (!i->imageData) cvReleaseImage(&i);
  return i;
}

IplImage *
cvCreateImageHeader(CvSize size, int depth, int channels)
{
  IplImage *i = (IplImage*)malloc(sizeof(*i));
  if (!i) return NULL;

  return cvInitImageHeader(i, size, depth, channels, 0, 0);
}

IplImage *
cvInitImageHeader(IplImage *image, CvSize size, int depth, int channels,
                  int origin, int align)
{
  image->nChannels = channels;
  image->depth = depth;
  image->width = size.width;
  image->height = size.height;
  image->widthStep = ((depth & 255) >> 3)*channels * size.width;
  image->imageSize = image->widthStep * image->height;
  image->imageData = NULL;

  return image;
}

void
cvSetData(struct _IplImage *image, void *data, int step)
{
  int line;

  line = ((image->depth & 255) >> 3)*image->nChannels * image->width;
  if (step == CV_AUTOSTEP || step < line)
    image->widthStep = line;
  else
    image->widthStep = step;

  image->imageSize = image->widthStep * image->height;
  image->imageData = (unsigned char*)data;
}


/* --- cvReleaseImage ------------------------------------------------ */

/* Free image structure */

void
cvReleaseImage(IplImage **image)
{
  if (*image) {
    if ((*image)->imageData) free((*image)->imageData);
    free(*image);
    *image = NULL;
  }
}


/* --- cvSaveImage --------------------------------------------------- */

/* Write an image to filename, in PGM or PPM format */

int
cvSaveImage(const char *filename, const struct _IplImage *image)
{
  FILE *f;
  unsigned char *l, *p;
  int i, j, c;

  f = fopen(filename, "w");
  if (!f) return 1;

  fprintf(f, "P%c %d %d %d\n", image->nChannels==1?'5':'6',
          image->width, image->height, (1<<image->depth)-1);

  for(i=0, l=image->imageData; i<image->height; i++, l+=image->widthStep) {
    for(j=0, p=l; j<image->width; j++, p+=(image->depth>>3)*image->nChannels)
      for(c=image->nChannels-1; c>=0; c--)
        fwrite(&p[c*(image->depth >> 3)], image->depth >> 3, 1, f);
  }

  fclose(f);
  return 0;
}


/* --- cvLoadImage --------------------------------------------------- */

/* Load an image from filename, in PGM or PPM format */

IplImage *
cvLoadImage(const char *filename, int iscolor)
{
  FILE *f;
  IplImage *image;
  unsigned char *l, *p;
  int i, j, c, s;

  if (iscolor != CV_LOAD_IMAGE_UNCHANGED)
    return NULL;

  f = fopen(filename, "r");
  if (!f) return NULL;

  image = (IplImage *)malloc(sizeof(*image));
  if (!image) return NULL;

  s = fscanf(f, "P%1d %d %d %d\n",
             &image->nChannels, &image->width, &image->height, &image->depth);
  if (s != 4) { free(image); return NULL; }

  if (image->nChannels == 5)
    image->nChannels = 1;
  else
    image->nChannels = 3;

  image->depth++;
  image->depth>>=5;

  image->widthStep = (image->depth >> 3)*image->nChannels * image->width;
  image->imageSize = image->widthStep * image->height;

  image->imageData = (unsigned char*)malloc(image->imageSize);
  if (!image->imageData) { free(image); return NULL; }

  for(i=0, l=image->imageData; i<image->height; i++, l+=image->widthStep) {
    for(j=0, p=l; j<image->width; j++, p+=(image->depth>>3)*image->nChannels)
      for(c=image->nChannels-1; c>=0; c--)
        fread(&p[c*(image->depth >> 3)], image->depth >> 3, 1, f);
  }

  fclose(f);
  return image;
}



//#endif /* VIAM_OPENCV_SUPPORT */
