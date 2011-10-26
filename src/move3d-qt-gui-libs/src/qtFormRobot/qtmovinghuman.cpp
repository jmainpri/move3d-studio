#include "qtmovinghuman.hpp"
#include "ui_qtmovinghuman.h"


#include "qtMainInterface/mainwindow.hpp"
#include "qtMainInterface/mainwindowGenerated.hpp"
#include "planner_handler.hpp"
#include "planner/planEnvironment.hpp"

MovingHuman::MovingHuman(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MovingHuman)
{
    ui->setupUi(this);
    init(0,0,0);
}

MovingHuman::MovingHuman(double x, double y, double rz, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::MovingHuman)
{
    ui->setupUi(this);
    init(x,y,rz);
}

MovingHuman::~MovingHuman()
{
    delete ui;
}

void MovingHuman::init(double x, double y, double rz)
{

    m_k_x  = new QtShiva::SpinBoxSliderConnector(this, ui->doubleSpinBoxX,      ui->horizontalSliderX,    PlanParam::env_futurX);
    m_k_x->setValue(x);
    m_k_y  = new QtShiva::SpinBoxSliderConnector(this, ui->doubleSpinBoxY,      ui->horizontalSliderY,    PlanParam::env_futurY);
    m_k_y->setValue(y);
    m_k_rz = new QtShiva::SpinBoxSliderConnector(this, ui->doubleSpinBoxRZ,     ui->horizontalSliderRZ,   PlanParam::env_futurRZ);
    m_k_rz->setValue(rz);

    connect(m_k_x,SIGNAL(valueChanged(double)),this,SLOT(updateMainWindow(double)));
    connect(m_k_y,SIGNAL(valueChanged(double)),this,SLOT(updateMainWindow(double)));
    connect(m_k_rz,SIGNAL(valueChanged(double)),this,SLOT(updateMainWindow(double)));
}


void MovingHuman::changeEvent(QEvent *e)
{
    QWidget::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

void MovingHuman::on_MovingHuman_destroyed()
{
    PlanEnv->setBool(PlanParam::env_realTime,false);
}


void MovingHuman::updateMainWindow(double)
{
    m_mainWindow->drawAllWinActive();
}
