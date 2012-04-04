#include "paramwidget.hpp"
#include "ui_paramwidget.h"
#include "posterreader.hpp"
#include "planner_handler.hpp"
#include "planner/planEnvironment.hpp"
#include "HRI_costspace/HRICS_Natural.hpp"
#include "HRI_costspace/HRICS_costspace.hpp"

ParamWidget::ParamWidget(QWidget *parent) :
  QWidget(parent),
  m_ui(new Ui::ParamWidget)
{
  m_ui->setupUi(this);
}

ParamWidget::~ParamWidget()
{
  delete m_ui;
}

void ParamWidget::setAndConnectPosterReader(PosterReader *pr)
{
  m_pr = pr;
  /* spark page */
  //m_pr->getSparkPoster()->setRefreshStatus(m_ui->sparkCheckBox->isChecked());
  connect(m_ui->sparkCheckBox, SIGNAL(clicked()), this, SLOT(setSparkRefresh()));
  connect(m_ui->sparkSaveSceBut,SIGNAL(clicked()),this,SLOT(sparkSaveScenario()));
  connect(m_ui->checkBoxDrawGoto, SIGNAL(toggled(bool)), pr , SLOT(setDrawGoTo(bool)), Qt::DirectConnection);
//  connect(m_ui->checkBoxColorCost,SIGNAL(toggled(bool)), pr,SLOT(changeHumanColor(bool)));

  connect(PlanEnv->getObject(PlanParam::drawColorConfig), SIGNAL(valueChanged(bool)), m_ui->checkBoxColorCost, SLOT(setChecked(bool)), Qt::DirectConnection);
  connect(m_ui->checkBoxColorCost, SIGNAL(toggled(bool)), PlanEnv->getObject(PlanParam::drawColorConfig), SLOT(set(bool)), Qt::DirectConnection);
  m_ui->checkBoxColorCost->setChecked(PlanEnv->getBool(PlanParam::drawColorConfig));

  //connect(m_pr, SIGNAL(sparkStatus(bool)),this, SLOT(setSparkStatusText(bool)));
}


void ParamWidget::setSparkRefresh()
{
  m_pr->getSparkPoster()->setRefreshStatus(m_ui->sparkCheckBox->isChecked());
}

void ParamWidget::sparkSaveScenario()
{
  QString fileName = QFileDialog::getSaveFileName(this);
  if (!fileName.isEmpty())
  {
    qt_fileName = fileName.toStdString().c_str();
    qt_saveScenario();
    emit drawAllWinActive();
    //this->drawAllWinActive();
  }
}

void ParamWidget::setSparkStatusText(bool updating)
{
  if(updating)
  {
    m_ui->sparkStatusLineEdit->setText(QString("updating"));
  }
  else
  {
    m_ui->sparkStatusLineEdit->setText(QString("not updating"));
  }
}

void ParamWidget::on_checkBoxColorCost_toggled(bool checked)
{

}

void ParamWidget::on_pushButtonNiutUpdate_toggled(bool checked)
{
    m_pr->getNiutPoster()->setRefreshStatus(checked);
}
