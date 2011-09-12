/*---------------------------------------------------------------*
 * Interface between Qt images and OpenCV images                 *
 * Filename : qtipl.h                                            *
 * Creation : 23 April 2003                                      *
 * Authors  : Rémi Ronfard, David Knossow and Matthieu Guilbert  *
 *---------------------------------------------------------------*/
 
#ifndef QTIPL_H
    #define QTIPL_H
 
    #include <qimage.h>
    #include <iostream>
    #include <cstring>
#include "myopencv.hpp"
 
    typedef unsigned short uint16_t;
 
    using std::string;
    using std::iostream;
 
    QImage *IplImageToQImage(
        const IplImage* iplImage,
        uchar**         data,
        double          mini=0.0,
        double          maxi=0.0);
    IplImage *QImageToIplImage(const QImage * qImage);
#endif
 
