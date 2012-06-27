#include <stdlib.h>
#include <qwt_painter.h>
#include <qwt_plot_canvas.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_curve.h>
#include <qwt_scale_widget.h>
#include <qwt_legend.h>
#include <qwt_scale_draw.h>
#include <qwt_math.h>
#include "multiPlot.hpp"
#include "../p3d/env.hpp"
#include <iostream>
#include <algorithm>

using namespace std;

MultiPlot::MultiPlot( QWidget *parent):
QwtPlot(parent)

{
	// Disable polygon clipping
	QwtPainter::setDeviceClipping(false);
	
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
	
	d_y.clear();
	
	//  Initialize data
	for ( int i=0; i<PLOT_SIZE; i++)
	{
		d_x.push_back( i );     // time axis
	}
	
	// Assign a title
	setTitle("Cost along trajectory");
	insertLegend(new QwtLegend(), QwtPlot::BottomLegend);
	
	init = false;
	//replot();
}

//! Set a plain canvas frame and align the scales to it
void MultiPlot::alignScales()
{
	// The code below shows how to align the scales to
	// the canvas frame, but is also a good example demonstrating
	// why the spreaded API needs polishing.
	
	canvas()->setFrameStyle(QFrame::Box | QFrame::Plain );
	canvas()->setLineWidth(1);
	
	for ( int i=0; i<QwtPlot::axisCnt; i++ )
	{
		QwtScaleWidget *scaleWidget = (QwtScaleWidget *)axisWidget(i);
		
		if ( scaleWidget )
			scaleWidget->setMargin(0);
		
		QwtScaleDraw *scaleDraw = (QwtScaleDraw *)axisScaleDraw(i);
		
		if ( scaleDraw )
			scaleDraw->enableComponent(QwtAbstractScaleDraw::Backbone, false);
	}
}

void MultiPlot::rescale()
{
	//    Max_y = *std::max_element(d_y,d_y+PLOT_SIZE);
	//    setAxisScale(QwtPlot::yLeft, 0.0,Max_y*1.10);
}

//  Generate new values
void MultiPlot::setData(const std::vector< std::string >& names , 
												 const std::vector< std::vector <double> >& data )
{	
	if( ENV.getBool(Env::initPlot) == false )
	{
    vector< double > Max_y,Min_y;
    
		// Get the max element in y of all curves
		for (vector< vector <double> >::const_iterator it = data.begin(); 
				 it != data.end(); ++it) 
		{
			Max_y.push_back( *std::max_element( (*it).begin(), (*it).end()) );
      Min_y.push_back( *std::min_element( (*it).begin(), (*it).end()) );
		}
		
		double max = *std::max_element( Max_y.begin() , Max_y.end() );
    double min = *std::min_element( Min_y.begin() , Min_y.end() );
		
		//        cout << "Setting fixed Axis"<< endl;
		//setAxisScale(QwtPlot::yLeft, -1.0, max*1.10);
    setAxisScale(QwtPlot::yLeft, 1.10*min, max*1.10);
		
		ENV.setBool(Env::initPlot,true);
	}
	
	// Save data in plot member
	cData.clear();
	
	// See Qt::GlobalColor
	int QtColours[]= { 7,8,9,11,12, 10, 16, 11, 17, 12, 18, 5, 4, 6, 19, 0, 1 };
	
	unsigned int i=0;
	for ( vector< vector<double> >::const_iterator it = data.begin(); 
			 it != data.end() ; ++it)
	{		
		// Set name and color (be carfull less that 16 plots)
		QString CurveName;
		if ( i < names.size() ) {
			CurveName = names[i].c_str();
		}
		else {
			CurveName = QString("Cost %1").arg(i);
		}
		
		QColor Color((Qt::GlobalColor) QtColours[i] ); i++;
    
    // Compute new curves
    QwtArray<double> d_y = QVector<double>::fromStdVector( *it );
    
    d_x.clear();
    for ( int i=0; i<d_y.size(); i++)
      d_x.push_back( i ); // time axis
		
		// Insert new curves
		cData.push_back( new QwtPlotCurve( CurveName ) );
		cData.back()->setPen ( QPen( Color ) );
		cData.back()->setData( d_x, d_y );
		cData.back()->attach(this);
	}
	
	replot();
}
