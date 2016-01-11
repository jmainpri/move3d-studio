#include "qtTrajectorySampling.hpp"
#include "ui_qtTrajectorySampling.h"
#include "qtBase/SpinBoxSliderConnector_p.hpp"
#include "planner/planEnvironment.hpp"

using namespace QtShiva;

TrajectorySamplingWidget::TrajectorySamplingWidget(QWidget *parent)
    : QWidget(parent), m_ui(new Ui::TrajectorySamplingWidget) {
  m_ui->setupUi(this);
  init();
}

TrajectorySamplingWidget::~TrajectorySamplingWidget() { delete m_ui; }

void TrajectorySamplingWidget::init() {
  // new connectCheckBoxToEnv( m_ui->radioButtonRRTStart,
  // PlanEnv->getObject(PlanParam::starRRT));

  //    connect( m_ui->radioButtonRRTStart, SIGNAL(toggled(bool)),
  //    PlanEnv->getObject(PlanParam::starRRT), SLOT(set(bool)),
  //    Qt::DirectConnection);
  //    connect( PlanEnv->getObject(PlanParam::starRRT),
  //    SIGNAL(valueChanged(bool)), m_ui->radioButtonRRTStart,
  //    SLOT(setChecked(bool)), Qt::DirectConnection);
  //    m_ui->radioButtonRRTStart->setChecked(
  //    PlanEnv->getBool(PlanParam::starRRT) );

  //    connect( m_ui->radioButtonRRG, SIGNAL(toggled(bool)),
  //    PlanEnv->getObject(PlanParam::rrg), SLOT(set(bool)),
  //    Qt::DirectConnection);
  //    connect( PlanEnv->getObject(PlanParam::rrg), SIGNAL(valueChanged(bool)),
  //    m_ui->radioButtonRRG, SLOT(setChecked(bool)), Qt::DirectConnection);
  //    m_ui->radioButtonRRG->setChecked( PlanEnv->getBool(PlanParam::rrg) );

  //    new connectCheckBoxToEnv( m_ui->checkBoxWithRewire,
  //    PlanEnv->getObject(PlanParam::starRewire));

  //    // Radius
  //    new SpinBoxSliderConnector( this, m_ui->doubleSpinBoxRadius,
  //    m_ui->horizontalSliderRadius,
  //    PlanEnv->getObject(PlanParam::starRadius));
  new SpinBoxConnector(this,
                       m_ui->doubleSpinBoxHessianFactor,
                       PlanEnv->getObject(PlanParam::lamp_hessian_factor));
  new SpinBoxConnector(this,
                       m_ui->doubleSpinBoxStdDev,
                       PlanEnv->getObject(PlanParam::trajOptimStdDev));
  new SpinBoxConnector(this,
                       m_ui->doubleSpinBoxControlCost,
                       PlanEnv->getObject(PlanParam::lamp_control_cost));
  new SpinBoxConnector(
      this, m_ui->doubleSpinBoxEta, PlanEnv->getObject(PlanParam::lamp_eta));

  // n
  new SpinBoxConnector(this,
                       m_ui->spinBoxNbSamples,
                       PlanEnv->getObject(PlanParam::lamp_nb_samples));
}
