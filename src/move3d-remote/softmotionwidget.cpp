#include "softmotionwidget.hpp"
#include "ui_softmotionwidget.h"
#include <iostream>


#include "planner_handler.hpp"
#include "move3d-gui.h"
#include "API/scene.hpp"
#include "API/project.hpp"
#include "P3d-pkg.h"
#include "Graphic-pkg.h"
#include "Util-pkg.h"

#include "qtBase/SpinBoxSliderConnector_p.hpp"

using namespace std;

softmotionWidget::softmotionWidget(PosterReader *pr, Ui::MainWindowRemote *m_ui_parent, QWidget *parent) :
        QWidget(parent),
        m_pr(pr),
        m_ui_p(m_ui_parent),
        m_ui(new Ui::softmotionWidget)
{
    m_ui->setupUi(this);
   connect(m_ui->pushButtonSMPlot,SIGNAL(clicked()),pr,SLOT(softmotionPlotTraj()));


   connect(m_ui->checkBoxSoftMotionDrawTraj, SIGNAL(toggled(bool)), pr , SLOT(softmotionDrawTraj(bool)), Qt::DirectConnection);

   QtShiva::SpinBoxSliderConnector *connectordt= new QtShiva::SpinBoxSliderConnector(
           this, m_ui->doubleSpinBoxdt, m_ui->horizontalSliderdt);


   connect(this, SIGNAL(softmotiondt(double)), pr, SLOT(changesoftmotiondt(double)));
}


softmotionWidget::~softmotionWidget()
{
    delete m_ui;
}



void softmotionWidget::init(PosterReader *pr, Ui::MainWindowRemote *ui_parent)
{
  if( (!pr) || (!ui_parent))
  {
    cout << "niutWidget not well initialized!!!" << endl;
    return;
  }


}

void softmotionWidget::on_doubleSpinBoxdt_valueChanged(double dt)
{
    emit softmotiondt(dt);
}
