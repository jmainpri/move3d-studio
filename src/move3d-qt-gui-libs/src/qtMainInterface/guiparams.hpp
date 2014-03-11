#ifndef GUI_PARAMETERS_HPP
#define GUI_PARAMETERS_HPP

#include <libmove3d/p3d/ParametersEnv.hpp>

#include "mainwindow.hpp"

#ifdef QT_LIBRARY
class GuiParam : public QObject
{

  Q_OBJECT;
  Q_ENUMS(boolParameter);
  Q_ENUMS(intParameter);
  Q_ENUMS(doubleParameter);
  Q_ENUMS(stringParameter);
  Q_ENUMS(vectorParameter);

public:

  GuiParam();
  ~GuiParam();

#else
namespace GuiParam
{
#endif
        enum boolParameter
        {
            tete
        };

        enum intParameter
        {
            mainwin_x,
            mainwin_y,
            mainwin_w,
            mainwin_h,
            tab_index_main,
            tab_index_cost
        };

        enum doubleParameter
        {
            toto
        };

        enum stringParameter
        {
             titi
        };

        enum vectorParameter
        {
             tutu
        };
};

// Object that holds all parameters
// Of the planner Environment
extern Parameters<GuiParam::boolParameter,GuiParam::intParameter,GuiParam::doubleParameter,
GuiParam::stringParameter,GuiParam::vectorParameter>* GuiEnv;

// Functions that initializes the planner
// Parameters
void initGuiParameters();

#ifdef QT_LIBRARY
extern GuiParam* EnumGuiParameterObject;
#endif

// Parameters of individual (computer, account) settings
void qt_saveGuiParameters(bool print, std::string fileName, MainWindow* mw);
void qt_loadGuiParameters(bool print, std::string fileName, MainWindow* mw);

#endif // GUI_PARAMETERS_HPP
