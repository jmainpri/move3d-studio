#include <stdlib.h>
#include <qwt_painter.h>
#include <qwt_plot_canvas.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_curve.h>
#include <qwt_scale_widget.h>
#include <qwt_legend.h>
#include <qwt_scale_draw.h>
#include <qwt_math.h>
#include "replottingVectors.hpp"

#include "p3d/env.hpp"

#include <iostream>
#include <algorithm>
#include <limits>
//
//  Initialize main window
//

using namespace std;


ReplottingVectors::ReplottingVectors(QWidget *parent):
    QwtPlot(parent),
    d_interval(0),
    d_timerId(-1)
{
    // Disable polygon clipping
    QwtPainter::setDeviceClipping(false);

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
    for ( int i = 0; i< REPLOT_SIZE; i++)
    {
        d_x.push_back( i );     // time axis
    }

    // Assign a title
    setTitle("Cost along trajectory");
    insertLegend(new QwtLegend(), QwtPlot::BottomLegend);

    init = false;

    setTimerInterval(200.0);
    
    m_Max_y = -numeric_limits<double>::max();
    m_Min_y = numeric_limits<double>::max();

    replot();
}

//
//  Set a plain canvas frame and align the scales to it
//
void ReplottingVectors::alignScales()
{
    // The code below shows how to align the scales to
    // the canvas frame, but is also a good example demonstrating
    // why the spreaded API needs polishing.

    canvas()->setFrameStyle( QFrame::Box | QFrame::Plain );
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

void ReplottingVectors::setTimerInterval(double ms)
{
    d_interval = qRound(ms);

    if ( d_timerId >= 0 )
    {
        killTimer(d_timerId);
        d_timerId = -1;
    }
    if (d_interval >= 0 )
        d_timerId = startTimer(d_interval);
}

//  Generate new values 
void ReplottingVectors::timerEvent(QTimerEvent *)
{
    for(unsigned int i=0;i<cData.size();i++)
    {
        try
        {
            vector<double> cost;

            if( m_data[i] != NULL && m_data[i]->size() > 0 )
            {
                double step = double(m_data[i]->size()) / double(REPLOT_SIZE);

                cost.clear();

                for( double param=0;
                     param < double(m_data[i]->size());
                     param += step)
                {
                    int id = ceil(param);
                    if (id >= int(m_data[i]->size())) {
                        id = m_data[i]->size() - 1;
                    }
                    cost.push_back(m_data[i]->at(id));
                }

                cost.resize( REPLOT_SIZE , 0.0 );

                double Max_y = *std::max_element(cost.begin(),cost.end());
                double Min_y = *std::min_element(cost.begin(),cost.end());

                if( Max_y > m_Max_y )
                    m_Max_y = Max_y;

                if( Min_y < m_Min_y )
                    m_Min_y = Min_y;

                setAxisScale(QwtPlot::yLeft, m_Min_y*1.10 ,m_Max_y*1.10);

                cData[i]->setData( d_x, QVector<double>::fromStdVector( cost ) );
                cout << "Add data (" << i << ")" << endl;
            }
        }
        catch (...)
        {
            cout << "Exeption in replotting vector widget" << endl;
        }
    }

    replot();
    //		rescale()
    //    updateLayout();
    //    alignScales();
}

//  Generate new values 
void ReplottingVectors::addData(const std::vector<const std::vector<double>* >& data)
{
    // Save data in plot member
    // See Qt::GlobalColor
    int QtColours[]= { 7,8,9,11,12, 10, 16, 11, 17, 12, 18, 5, 4, 6, 19, 0, 1 };

    QwtArray< double > vect;

    for ( int j = 0; j< REPLOT_SIZE; j++)
        vect.push_back( j );     // time axis∫

    unsigned int i=0;
    for (  vector<const vector<double>* >::const_iterator it = data.begin();
           it != data.end() ; ++it)
    {
        // Set name and color (be carfull less that 16 plots)
        QString CurveName  = QString("Cost %1").arg(i);

        //		if ( i < names.size() )
        //			CurveName = names[i].c_str();
        //		else
        //      CurveName = QString("Cost %1").arg(i);

        QColor Color((Qt::GlobalColor) QtColours[i] ); i++;

        m_data.push_back( *it );

        // Insert new curves
        cData.push_back( new QwtPlotCurve( CurveName ) );
        cData.back()->setPen ( QPen( Color ) );
        cData.back()->setData( d_x, vect );
        cData.back()->attach(this);
    }
}

void ReplottingVectors::clearData()
{
    cData.clear();
    m_data.clear();
}

