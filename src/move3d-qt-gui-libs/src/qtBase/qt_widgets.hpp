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
#ifndef QT_WIDGETS_HH
#define QT_WIDGETS_HH

#include "qtLibrary.hpp"

#include <vector>

/**
 * @ingroup qtOldWidget
 * @brief Slider for double
 */
class QDoubleSlider: public QSlider
{
	Q_OBJECT

	double doubleValue;
	double doubleMin;
	double doubleMax;

signals:
void valueChanged(double value);

private:
	void setValue(double new_value, bool sync, bool init = false);private slots:
	void synchronizeDouble(int new_value);

public slots:
	void setValue(double new_value);

public:
	QDoubleSlider(double min, double max, double init_value, QWidget *parent =
			0);
	void setRange(double min, double max);
	double value() const;
	double min() const;
	double max() const;
};

/**
 * @ingroup qtOldWidget
 * @brief Lableled slider for double
 */
class LabeledDoubleSlider: public QWidget
{
	Q_OBJECT

	QDoubleSlider* slider;

	signals:
	void sliderValue(QString);
	void sliderValue(double);
	void valueChanged(double);public slots:
	void doubleToQString(double value);
	void qStringToDouble(QString value);
	void setValue(double new_value);

public:
	LabeledDoubleSlider(double min, double max, double init_value,
			QString text, int row = 0, QGridLayout* layout = 0,
			QWidget *parent = 0);
	double value() const;
};

/**
 * @ingroup qtOldWidget
 * @brief Lableled slider forinteger
 */
class LabeledSlider: public QWidget
{
	Q_OBJECT

	QSlider* slider;

	signals:
	void sliderValue(QString);
	void sliderValue(int);
	void valueChanged(int);public slots:
	void intToQString(int value);
	void qStringToInt(QString value);
	void setValue(int new_value);

public:
	LabeledSlider(int min, int max, int init_value, QString text, int row = 0,
			QGridLayout* layout = 0, QWidget *parent = 0);
	int value() const;
};

/**
 * @ingroup qtOldWidget
 * @brief GroupBox class
 */
class QVGroupBox: public QGroupBox
{
Q_OBJECT

public:
	QVGroupBox(const QString& title, QWidget* parent = 0);
	void addWidget(QWidget* w, Qt::Alignment alignment = 0);
};

/**
 * @ingroup qtOldWidget
 * @brief Open a File
 */
class OpenFile: public QWidget
{
	Q_OBJECT
	Q_PROPERTY(QString filename READ filename)
	QLabel _label;
	QString p_filename;signals:
	void fileChanged(QString text);

protected slots:
	void setTextFromList(QStringList list);

public:
	OpenFile(QString button_label, QStringList filters = QStringList(
			"Any files (*)"), QBoxLayout::Direction dir =
			QBoxLayout::LeftToRight);
	QString filename() const;
	QFileDialog dialog;
};

#endif
