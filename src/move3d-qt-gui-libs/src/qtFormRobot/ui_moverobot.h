/*
 * Copyright (c) 2010-2014 LAAS/CNRS, WPI
 * All rights reserved.
 *
 * Redistribution  and  use  in  source  and binary  forms,  with  or  without
 * modification, are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of  source  code must retain the  above copyright
 *      notice and this list of conditions.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice and  this list of  conditions in the  documentation and/or
 *      other materials provided with the distribution.
 *
 * THE SOFTWARE  IS PROVIDED "AS IS"  AND THE AUTHOR  DISCLAIMS ALL WARRANTIES
 * WITH  REGARD   TO  THIS  SOFTWARE  INCLUDING  ALL   IMPLIED  WARRANTIES  OF
 * MERCHANTABILITY AND  FITNESS.  IN NO EVENT  SHALL THE AUTHOR  BE LIABLE FOR
 * ANY  SPECIAL, DIRECT,  INDIRECT, OR  CONSEQUENTIAL DAMAGES  OR  ANY DAMAGES
 * WHATSOEVER  RESULTING FROM  LOSS OF  USE, DATA  OR PROFITS,  WHETHER  IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR  OTHER TORTIOUS ACTION, ARISING OUT OF OR
 * IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.                                  
 *
 * Siméon, T., Laumond, J. P., & Lamiraux, F. (2001). 
 * Move3d: A generic platform for path planning. In in 4th Int. Symp.
 * on Assembly and Task Planning.
 *
 *                                               Jim Mainprice Tue 27 May 2014 
 */
/********************************************************************************
** Form generated from reading UI file 'moverobot.ui'
**
** Created: Tue Dec 7 13:21:10 2010
**      by: Qt User Interface Compiler version 4.7.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MOVEROBOT_H
#define UI_MOVEROBOT_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MoveRobot
{
public:
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *MainLayout;

    void setupUi(QWidget *MoveRobot)
    {
        if (MoveRobot->objectName().isEmpty())
            MoveRobot->setObjectName(QString::fromUtf8("MoveRobot"));
        MoveRobot->resize(660, 440);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MoveRobot->sizePolicy().hasHeightForWidth());
        MoveRobot->setSizePolicy(sizePolicy);
        horizontalLayout = new QHBoxLayout(MoveRobot);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        MainLayout = new QVBoxLayout();
        MainLayout->setObjectName(QString::fromUtf8("MainLayout"));

        horizontalLayout->addLayout(MainLayout);


        retranslateUi(MoveRobot);

        QMetaObject::connectSlotsByName(MoveRobot);
    } // setupUi

    void retranslateUi(QWidget *MoveRobot)
    {
        MoveRobot->setWindowTitle(QApplication::translate("MoveRobot", "Form", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MoveRobot: public Ui_MoveRobot {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MOVEROBOT_H
