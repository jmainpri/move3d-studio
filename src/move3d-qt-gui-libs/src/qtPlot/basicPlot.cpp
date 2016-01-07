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
#include <stdlib.h>
#include <qwt_painter.h>
#include <qwt_plot_canvas.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_curve.h>
#include <qwt_scale_widget.h>
#include <qwt_legend.h>
#include <qwt_scale_draw.h>
#include <qwt_math.h>
#include "basicPlot.hpp"
#include "../p3d/env.hpp"
#include <iostream>
#include <algorithm>
//
//  Initialize main window
//

using namespace std;

BasicPlot::BasicPlot( QWidget *parent):
QwtPlot(parent)
{
    // Disable polygon clipping
    // QwtPainter::setDeviceClipping(false);

    // We don't need the cache here
//    canvas()->setPaintAttribute(QwtPlotCanvas::PaintCached, false);
//    canvas()->setPaintAttribute(QwtPlotCanvas::PaintPacked, false);

#if QT_VERSION >= 0x040000
#ifdef Q_WS_X11
    /*
       Qt::WA_PaintOnScreen is only supported for X11, but leads
       to substantial bugs with Qt 4.2.x/Windows
     */
    canvas()->setAttribute(Qt::WA_PaintOnScreen, true);
#endif
#endif

//    alignScales();
    
    //  Initialize data
    QVector<QPointF> points;
    for (int i = 0; i< PLOT_SIZE; i++)
    {
        d_x[i] = i;     // time axis
        d_y[i] = i;
        points.push_back( QPointF(d_x[i],d_y[i]) );
    }

    // Assign a title
    setTitle("Cost along trajectory");
    insertLegend(new QwtLegend(), QwtPlot::BottomLegend);

    // Insert new curves
    cRight = new QwtPlotCurve("Cost");
    cRight->setPen(QPen(Qt::red));

    // cRight->setRawData(d_x, d_y, PLOT_SIZE);


    cRight->setSamples( points );

    cRight->attach(this);

    init = false;
    Max_y = 0.0;
    replot();
}

//
//  Set a plain canvas frame and align the scales to it
//
void BasicPlot::alignScales()
{
    // The code below shows how to align the scales to
    // the canvas frame, but is also a good example demonstrating
    // why the spreaded API needs polishing.

    canvas()->setFrameStyle(QFrame::Box | QFrame::Plain );
    canvas()->setLineWidth(1);

    for ( int i = 0; i < QwtPlot::axisCnt; i++ )
    {
        QwtScaleWidget *scaleWidget = (QwtScaleWidget *)axisWidget(i);

        if ( scaleWidget )
            scaleWidget->setMargin(0);

        QwtScaleDraw *scaleDraw = (QwtScaleDraw *)axisScaleDraw(i);

        if ( scaleDraw )
            scaleDraw->enableComponent(QwtAbstractScaleDraw::Backbone, false);
    }
}

//void BasicPlot::rescale()
//{
//    Max_y = *std::max_element(d_y,d_y+PLOT_SIZE);
//    setAxisScale(QwtPlot::yLeft, 0.0,Max_y*1.10);
//}

//  Generate new values 
void BasicPlot::setData(std::vector<double> y)
{
  if (int(y.size()) >= PLOT_SIZE) 
  {
    if(ENV.getBool(Env::initPlot) == false )
    {
      Max_y = *std::max_element(y.begin(),y.end());
      setAxisScale(QwtPlot::yLeft, 0.0,Max_y*1.10);
      ENV.setBool(Env::initPlot,true);
    }
    
    QVector<QPointF> points;
    points.clear();
    for ( int i = 0; i<PLOT_SIZE ; i++)
    {
      d_y[i] = y[i];
      //cout << y[i] << endl
      points.push_back( QPointF(d_x[i],d_y[i]) );;
    }

    cRight->setSamples( points );
  }
  else
  {
    cout << "cannot set data (vector too small)" << endl;
  }
  
  replot();
  //		rescale()
  //    updateLayout();
  //    alignScales();
}
