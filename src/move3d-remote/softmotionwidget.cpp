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

softmotionWidget::softmotionWidget(QWidget *parent) :
        QWidget(parent),
        m_ui(new Ui::softmotionWidget)
{
    m_ui->setupUi(this);

}


softmotionWidget::~softmotionWidget()
{
    delete m_ui;
}



void softmotionWidget::init(PosterReader *pr, Ui::ParamWidget *ui_param)
{
  if( (!pr) || (!ui_param))
  {
    cout << "niutWidget not well initialized!!!" << endl;
    return;
  }
  m_pr = pr;
  m_ui_p = ui_param;
  connect(m_ui->pushButtonSMPlot,SIGNAL(clicked()),pr,SLOT(softmotionPlotTraj()));
  connect(m_ui->checkBoxSoftMotionDrawTraj, SIGNAL(toggled(bool)), pr , SLOT(softmotionDrawTraj(bool)), Qt::DirectConnection);
  QtShiva::SpinBoxSliderConnector *connectordt= new QtShiva::SpinBoxSliderConnector(
          this, m_ui->doubleSpinBoxdt, m_ui->horizontalSliderdt);
  connect(connectordt, SIGNAL(valueChanged(double)), pr, SLOT(changesoftmotiondt(double)));
}


