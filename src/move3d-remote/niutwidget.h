#ifndef niutWIDGET_H
#define niutWIDGET_H

#include <QWidget>
#include "qtLibrary.hpp"

#include "posterreader.hpp"

namespace Ui {
    class niutWidget;
    class ParamWidget;
}

class niutWidget : public QWidget 
{
    Q_OBJECT
public:
    niutWidget(QWidget *parent = 0);
    ~niutWidget();
    void init(PosterReader *pr, Ui::ParamWidget *m_ui_parent);
    bool isToHideBar();

public slots:
    void setNiutIsAlive(bool state);
    void setNiutColorLabel(int idLabel, int color);

protected:
    void changeEvent(QEvent *e);

private:
    PosterReader *m_pr;
    Ui::ParamWidget *m_ui_p;
    Ui::niutWidget *m_ui;
    
    std::vector<QLabel*> _niutLabels;
    QPixmap _niutPmAlive;
    QPixmap _niutPmDead;
    QPixmap _niutPmRed;
    QPixmap _niutPmOrange;
    QPixmap _niutPmYellow;
    QPixmap _niutPmGreen;

    std::vector<bool> _showedPics;
    std::vector<int> _idDict;

};

#endif // niutWIDGET_H
