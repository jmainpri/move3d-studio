#ifndef ATTENTIONALWIDGET_HPP
#define ATTENTIONALWIDGET_HPP

#include <QWidget>

class PosterReader;

namespace Ui {
    class AttentionalWidget;
    class ParamWidget;
}

class AttentionalWidget : public QWidget
{
    Q_OBJECT

public:
    explicit AttentionalWidget(QWidget *parent = 0);
    ~AttentionalWidget();
    void init(PosterReader *pr, Ui::ParamWidget *ui_param);

private slots:
  void update();

private:
    Ui::AttentionalWidget *m_ui;
    PosterReader *m_pr;
    Ui::ParamWidget *m_ui_p;

};

#endif // ATTENTIONALWIDGET_HPP
