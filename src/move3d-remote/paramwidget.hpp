#ifndef PARAMWIDGET_HPP
#define PARAMWIDGET_HPP

#include <QWidget>

class PosterReader;

namespace Ui {
    class ParamWidget;
}

class ParamWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ParamWidget(QWidget *parent = 0);
    ~ParamWidget();

  void setAndConnectPosterReader(PosterReader *pr);

signals:
    void drawAllWinActive();

public slots:
    void setSparkStatusText(bool updating);

private slots:
        void on_pushButtonNiutUpdate_toggled(bool checked);
        void on_checkBoxColorCost_toggled(bool checked);
        void setSparkRefresh();
        void sparkSaveScenario();

public:
    Ui::ParamWidget *m_ui;
    PosterReader *m_pr;
};

#endif // PARAMWIDGET_HPP
