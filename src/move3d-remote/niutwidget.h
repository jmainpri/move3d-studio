#ifndef niutWIDGET_H
#define niutWIDGET_H

#include <QWidget>
#include "qtLibrary.hpp"

#include "images/green-man-xpm.h"
#include "images/orange-man-xpm.h"
#include "images/red-man-xpm.h"
#include "images/yellow-death-xpm.h"
#include "images/yellow-man-xpm.h"
#include "images/im_still_alive-xpm.h"

namespace Ui {
    class niutWidget;
}

class niutWidget : public QWidget {
    Q_OBJECT
public:
    niutWidget(QWidget *parent = 0);
    ~niutWidget();

public slots:
    void setNiutIsAlive(bool state);
    void setNiutColorLabel(int idLabel, int color);

protected:
    void changeEvent(QEvent *e);

private:
    Ui::niutWidget *m_ui;
    std::vector<QLabel*> _niutLabels;
    QPixmap _niutPmAlive;
    QPixmap _niutPmDead;
    QPixmap _niutPmRed;
    QPixmap _niutPmOrange;
    QPixmap _niutPmYellow;
    QPixmap _niutPmGreen;

    void initNiut();
};

#endif // niutWIDGET_H
