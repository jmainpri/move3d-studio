#ifndef QT_BASE_WIN_HH
#define QT_BASE_WIN_HH

#include "../p3d/env.hpp"

#if defined( MOVE3D_CORE )
#include "qtLibrary.hpp"
#include "qtBase/qt_widgets.hpp"
#endif

/**
 * @ingroup qtOldWidget
 * @brief Qt Window base container
 */
class qtBaseWindow : public QObject
{
	Q_OBJECT;

protected:
	QString string;
	QGroupBox* box;
	QGridLayout* Layout;

public:
	// Constructor
	qtBaseWindow();

	// Getters
	QString 		getString();
	QGroupBox* 		getBox();
	QGridLayout* 	getLayout();

	// Initialization function
	//virtual void  init() = 0;
	//void init();

	// Create sliders and check boxes
	LabeledSlider* createSlider(QString s, Env::intParameter p, int lower, int upper);
	LabeledDoubleSlider* createDoubleSlider(QString s, Env::doubleParameter p, double lower, double upper);
	QCheckBox* createCheckBox(QString s, Env::boolParameter p);

public:
	// Destructor
	~qtBaseWindow();

};
#endif
