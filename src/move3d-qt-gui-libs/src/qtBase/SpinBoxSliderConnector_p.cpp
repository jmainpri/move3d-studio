/*
 *  Copyright (c) 2008 Cyrille Berger <cberger@cberger.net>
 *  Copyright (c) 2010 Jim Mainprice LAAS/CNRS
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation;
 * either version 2, or (at your option) any later version of the License.
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

#include "SpinBoxSliderConnector_p.hpp"

#include "planner/planEnvironment.hpp"

#include <iostream>
#include <limits>

using namespace QtShiva;
using namespace std;

connectCheckBoxToEnv::connectCheckBoxToEnv( QCheckBox* box, QObject* o )
{
    connect( o, SIGNAL(valueChanged(bool)), box, SLOT(setChecked(bool)), Qt::DirectConnection);
    connect( box, SIGNAL(toggled(bool)), o, SLOT(set(bool)), Qt::DirectConnection);
    box->setChecked( dynamic_cast<boolContainer*>(o)->get());
}


connectComboBoxToEnv::connectComboBoxToEnv( QComboBox* box, QObject* o )
{
    connect( o, SIGNAL(valueChanged(int)), box, SLOT(setCurrentIndex(int)));
    connect( box, SIGNAL(currentIndexChanged(int)), o, SLOT(set(int)) );
    box->setCurrentIndex( dynamic_cast<intContainer*>(o)->get() );
}


SpinBoxSliderConnector::SpinBoxSliderConnector( QObject* _parent, QDoubleSpinBox* _spinBox, QSlider* _slider) :
    QObject( _parent ), m_spinBox( _spinBox ), m_slider( _slider )
{
    m_spinBox->setValue(numeric_limits<double>::max());

    connect( m_spinBox, SIGNAL(valueChanged( double )), SLOT(spinBoxValueChanged( double ) ) );
    connect( m_slider, SIGNAL(valueChanged( int )), SLOT(sliderValueChanged( int ) ) );

    emit( this->valueChanged( m_spinBox->value() ) );
    
    _init = false;
}


SpinBoxSliderConnector::SpinBoxSliderConnector( QObject* _parent, QDoubleSpinBox* _spinBox, QSlider* _slider, QObject* o ) :
    QObject( _parent ), m_spinBox( _spinBox ), m_slider( _slider )
{
    m_spinBox->setValue(numeric_limits<double>::max());
    
    connect( m_spinBox, SIGNAL(valueChanged( double )), SLOT(spinBoxValueChanged( double ) ) );
    connect( m_slider, SIGNAL(valueChanged( int )), SLOT(sliderValueChanged( int ) ) );

    if( dynamic_cast<intContainer*>(o) != NULL )
    {
        connect( this, SIGNAL(valueChanged( int )), o, SLOT(set(int)));
        connect( o, SIGNAL(valueChanged(int)), this, SLOT(spinBoxValueChanged(int)));
        m_spinBox->setValue(dynamic_cast<intContainer*>(o)->get());
    }
    else
    {
        connect( this, SIGNAL(valueChanged( double )), o,SLOT(set(double)));
        connect( o, SIGNAL(valueChanged(double)),m_spinBox, SLOT(setValue(double)));
        m_spinBox->setValue(dynamic_cast<doubleContainer*>(o)->get());
    }
    
    _init = false;
}


/*
SpinBoxSliderConnector::SpinBoxSliderConnector( QObject* _parent, QDoubleSpinBox* _spinBox, QSlider* _slider, PlanParam::doubleParameter p) :
QObject( _parent ), m_spinBox( _spinBox ), m_slider( _slider )
{
    m_spinBox->setValue(numeric_limits<double>::max());

    connect( m_spinBox, SIGNAL(valueChanged(double)), SLOT(spinBoxValueChanged(double)) );
    connect( m_slider, SIGNAL(valueChanged(int)), SLOT(sliderValueChanged(int)) );
    connect( this,SIGNAL(valueChanged(double)),PlanEnv->getObject(p),SLOT(set(double)));
    connect( PlanEnv->getObject(p),SIGNAL(valueChanged(double)),m_spinBox, SLOT(setValue(double)));

    m_spinBox->setValue(PlanEnv->getDouble(p));

    _init = false;
}

SpinBoxSliderConnector::SpinBoxSliderConnector( QObject* _parent,
                                               QDoubleSpinBox* _spinBox,
                                               QSlider* _slider,
                                               Env::intParameter p) :
QObject( _parent ), m_spinBox( _spinBox ), m_slider( _slider )
{
    m_spinBox->setValue(numeric_limits<double>::max());

    connect( m_spinBox, SIGNAL(valueChanged( double )), SLOT(spinBoxValueChanged( double ) ) );
    connect( m_slider, SIGNAL(valueChanged( int )), SLOT(sliderValueChanged( int ) ) );
    connect( this, SIGNAL(valueChanged( int )), ENV.getObject(p), SLOT(set(int)));
    connect( ENV.getObject(p), SIGNAL(valueChanged(int)), this, SLOT(spinBoxValueChanged(int)));

    m_spinBox->setValue(ENV.getInt(p));

    _init = false;
}

SpinBoxSliderConnector::SpinBoxSliderConnector( QObject* _parent,
                                               QDoubleSpinBox* _spinBox,
                                               QSlider* _slider,
                                               PlanParam::intParameter p) :
QObject( _parent ), m_spinBox( _spinBox ), m_slider( _slider )
{
    m_spinBox->setValue(numeric_limits<double>::max());

    connect( m_spinBox, SIGNAL(valueChanged( double )), SLOT(spinBoxValueChanged( double ) ) );
    connect( m_slider, SIGNAL(valueChanged( int )), SLOT(sliderValueChanged( int ) ) );
    connect( this, SIGNAL(valueChanged( int )), PlanEnv->getObject(p), SLOT(set(int)));
    connect( PlanEnv->getObject(p), SIGNAL(valueChanged(int)), this, SLOT(spinBoxValueChanged(int)));

    m_spinBox->setValue(PlanEnv->getInt(p));

    _init = false;
}
*/

SpinBoxSliderConnector::~SpinBoxSliderConnector()
{
}

void SpinBoxSliderConnector::computeScaling()
{
    _a =  (double)m_slider->maximum();
    _b =  (double)m_slider->minimum();
    _c =  m_spinBox->maximum();
    _d =  m_spinBox->minimum();
    
    //cout << "a : " << _a << " , b : " << _b << " , c : " << _c << " , d : " << _d << endl;
    //cout << "_Coeff : " << _Coeff << " , _Offset : " << _Offset << endl;

    _Coeff = ( _a - _b )/( _c - _d );
    _Offset = (_a + _b)/2 - (_a - _b)*(_c + _d)/(2*(_c - _d));
}

double SpinBoxSliderConnector::value() const
{
    return m_spinBox->value();
}

void SpinBoxSliderConnector::setValue( double _value )
{
    m_spinBox->setValue( _value );
}

void SpinBoxSliderConnector::spinBoxValueChanged( int _value )
{
    int newValue = (int) ( _value * _Coeff + _Offset );

    m_slider->setValue( newValue );

    emit( valueChanged( m_spinBox->value() ) );
    emit( valueChanged( (int)(m_spinBox->value()) ) );
    //    m_slider->setValue(_value);
}

void SpinBoxSliderConnector::spinBoxValueChanged( double _value )
{
    /*bool v =*/ m_slider->blockSignals(true);
    
    //    if(!_init)
    //    {
    this->computeScaling();
    _init = true;
    //    }
    
    int newValue = (int) ( _value * _Coeff + _Offset );
    
    m_slider->setValue( newValue );
    m_slider->blockSignals(false);
    emit( valueChanged( m_spinBox->value() ) );
    emit( valueChanged( (int)(m_spinBox->value()) ) );
}

void SpinBoxSliderConnector::sliderValueChanged( int _value )
{   
    /*bool v =*/ m_spinBox->blockSignals(true);
    
    //    if(!_init)
    //    {
    this->computeScaling();
    _init = true;
    //    }
    
    double newValue = (double) (( _value - _Offset) / _Coeff );
    
    m_spinBox->setValue( newValue );
    m_spinBox->blockSignals(false);

    //cout << "vauleChanged : " << m_spinBox->value() << endl;
    emit( this->valueChanged( m_spinBox->value() ) );
    emit( this->valueChanged( (int)(m_spinBox->value()) ));
}

//****************************************************************
// SpinBox to Env

//----------------------------------------------------------------
// Env connector
//----------------------------------------------------------------
SpinBoxConnector::SpinBoxConnector( QObject* _parent, QDoubleSpinBox* _spinBox, QObject* o) :
    QObject( _parent ), m_doubleSpinBox( _spinBox )
{
    m_doubleSpinBox->setValue(numeric_limits<double>::max());

    if( dynamic_cast<intContainer*>(o) != NULL )
    {
        connect(m_doubleSpinBox,SIGNAL(valueChanged(int)),o,SLOT(set(int)));
        connect(o,SIGNAL(valueChanged(int)),m_doubleSpinBox, SLOT(setValue(int)));
        m_doubleSpinBox->setValue(dynamic_cast<intContainer*>(o)->get());
    }
    else
    {
        connect(m_doubleSpinBox,SIGNAL(valueChanged(double)), o,SLOT(set(double)));
        connect(o,SIGNAL(valueChanged(double)),m_doubleSpinBox,SLOT(setValue(double)));
        m_doubleSpinBox->setValue(dynamic_cast<doubleContainer*>(o)->get());
    }
}

SpinBoxConnector::SpinBoxConnector( QObject* _parent, QSpinBox* _spinBox, QObject* o ) :
    QObject( _parent ), m_spinBox( _spinBox )
{
    if( dynamic_cast<intContainer*>(o) == NULL )
    {
        cout << "Error connecting spin box!!!" << endl;
        return;
    }

    m_spinBox->setValue(numeric_limits<int>::max());

    connect( m_spinBox, SIGNAL(valueChanged(int)), o, SLOT(set(int)) );
    connect( o, SIGNAL(valueChanged(int)), m_spinBox, SLOT(setValue(int)) );

    m_spinBox->setValue(dynamic_cast<intContainer*>(o)->get());
}

/*
//----------------------------------------------------------------
// PlanParam connector
//----------------------------------------------------------------
SpinBoxConnector::SpinBoxConnector( QObject* _parent,
                                    QDoubleSpinBox* _spinBox,
                                    PlanParam::doubleParameter p) :
QObject( _parent ), m_doubleSpinBox( _spinBox )
{
    m_doubleSpinBox->setValue(numeric_limits<double>::max());

    connect(m_doubleSpinBox,SIGNAL(valueChanged(double)),PlanEnv->getObject(p),SLOT(set(double)));
    connect(PlanEnv->getObject(p),SIGNAL(valueChanged(double)),m_doubleSpinBox,SLOT(setValue(double)));

    m_doubleSpinBox->setValue(PlanEnv->getDouble(p));
}

SpinBoxConnector::SpinBoxConnector( QObject* _parent,
                                    QDoubleSpinBox* _spinBox,
                                    PlanParam::intParameter p) :
QObject( _parent ), m_doubleSpinBox( _spinBox )
{
    m_doubleSpinBox->setValue(numeric_limits<double>::max());

    connect(m_doubleSpinBox, SIGNAL(valueChanged(int)), PlanEnv->getObject(p),SLOT(set(int)));
    connect(PlanEnv->getObject(p),SIGNAL(valueChanged(int)),m_doubleSpinBox,SLOT(setValue(int)));

    m_doubleSpinBox->setValue(PlanEnv->getInt(p));
}

SpinBoxConnector::SpinBoxConnector( QObject* _parent,
                                    QSpinBox* _spinBox,
                                    PlanParam::intParameter p) :
QObject( _parent ), m_spinBox( _spinBox )
{
    m_spinBox->setValue(numeric_limits<int>::max());

    connect(m_spinBox, SIGNAL(valueChanged(int)), PlanEnv->getObject(p),SLOT(set(int)));
    connect(PlanEnv->getObject(p),SIGNAL(valueChanged(int)),m_spinBox,SLOT(setValue(int)));

    m_spinBox->setValue(PlanEnv->getInt(p));
}
*/
