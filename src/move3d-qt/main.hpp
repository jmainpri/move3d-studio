#ifndef MAIN_HPP
#define MAIN_HPP

#if defined( QT_LIBRARY ) && defined( MOVE3D_CORE )
#include "qtLibrary.hpp"
#endif

#if defined( QT_GL ) && defined( MOVE3D_CORE )
#include "qtOpenGL/qtGLWindow.hpp"
#endif

#include <QtCore/QString>

#ifdef QT_GL
/**
 * @ingroup qtWindow
 * @brief Main application with the QT_WidgetMain double thread class (X-Forms Thread)
 */
class Main_threads: public QObject
{
  Q_OBJECT
  
  qtGLWindow* 	g3dWin;
  QApplication*       app;
  QCoreApplication* 	coreApp;
  
public:
  Main_threads();
  ~Main_threads();
  
  int run(int argc, char** argv);
  
public slots:
  void selectPlanner();
  void initInterface();
  void loadSettings();
  
signals:
  void selectedPlanner(QString);
  
  private slots :
  void exit();  
};

/**
 * @ingroup qtWindow
 * @brief Simple OpenGl display with Qt
 */
class Simple_threads : public QObject
{
	Q_OBJECT

	GLWidget* 	m_simpleOpenGlWidget;

	QApplication* 	app;
	
public:
	Simple_threads();
	~Simple_threads();
	
  int run(int argc, char** argv);

	
	private slots :
};

#endif

#endif // MAIN_HPP
