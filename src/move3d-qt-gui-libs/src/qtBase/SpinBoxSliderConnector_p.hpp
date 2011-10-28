/*
 *  Copyright (c) 2008 Cyrille Berger <cberger@cberger.net>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation;
 * version 2 of the license.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 */

#ifndef _SPINBOX_SLIDER_CONNECTOR_HPP_
#define _SPINBOX_SLIDER_CONNECTOR_HPP_

#include "../p3d/env.hpp"
#include "planner/planEnvironment.hpp"

#if defined( MOVE3D_CORE )
#include "qtLibrary.hpp"
#endif

class QDoubleSpinBox;
class QSlider;

/**
  * @ingroup qtMainWindow
  * @brief Connects a slider to a double spin box
  The object is the result of the connection of both objects
  */
namespace QtShiva
{
    class SpinBoxSliderConnector : public QObject
    {
        Q_OBJECT
    public:
        SpinBoxSliderConnector( QObject* _parent,
                                QDoubleSpinBox* _spinBox,
                                QSlider* _slider);

        SpinBoxSliderConnector( QObject* _parent,
                                QDoubleSpinBox* _spinBox,
                                QSlider* _slider,
                                Env::doubleParameter p);

        SpinBoxSliderConnector( QObject* _parent,
                                QDoubleSpinBox* _spinBox,
                                QSlider* _slider,
                                Env::intParameter p);
			
				SpinBoxSliderConnector( QObject* _parent,
																QDoubleSpinBox* _spinBox,
																QSlider* _slider,
																PlanParam::doubleParameter p);
			
				SpinBoxSliderConnector( QObject* _parent,
																QDoubleSpinBox* _spinBox,
																QSlider* _slider,
																PlanParam::intParameter p);


        ~SpinBoxSliderConnector();

        /**
          @brief gets the value of the Connector object
          */
        double value() const;

        /**
          @brief sets the value of the Connector object
          **/
        void setValue( double _value );

    private slots:
        void spinBoxValueChanged( double _value );
        void sliderValueChanged( int _value );

    signals:
        void valueChanged( double _value );
        void valueChanged( int _valueInt );

    private:
        QDoubleSpinBox* m_spinBox;
        QSlider* m_slider;

        void computeScaling();
        bool _init;
        double _a,_b,_c,_d;
        double _Coeff, _Offset;
    };
  
  /**
   * @ingroup qtMainWindow
   * @brief Connects a SpinBox to Env Parameters
   */
  class SpinBoxConnector : public QObject
  {
    Q_OBJECT
  public:
    
    SpinBoxConnector( QObject* _parent, QDoubleSpinBox* _spinBox, Env::doubleParameter p);
    SpinBoxConnector( QObject* _parent, QDoubleSpinBox* _spinBox, Env::intParameter p);
    SpinBoxConnector( QObject* _parent, QDoubleSpinBox* _spinBox, PlanParam::doubleParameter p);
    SpinBoxConnector( QObject* _parent, QDoubleSpinBox* _spinBox, PlanParam::intParameter p);
    
    SpinBoxConnector( QObject* _parent, QSpinBox* _spinBox, Env::intParameter p);
    SpinBoxConnector( QObject* _parent, QSpinBox* _spinBox, PlanParam::intParameter p);
    
    ~SpinBoxConnector() { }
    
  private:
    QSpinBox*           m_spinBox;
    QDoubleSpinBox*     m_doubleSpinBox;
    bool _init;
  };
}

#endif
