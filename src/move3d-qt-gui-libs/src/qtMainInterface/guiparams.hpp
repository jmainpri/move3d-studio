#ifndef GUI_PARAMETERS_HPP
#define GUI_PARAMETERS_HPP

#include <libmove3d/p3d/ParametersEnv.hpp>

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
            mainwin_w,
            mainwin_h
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

#endif // GUI_PARAMETERS_HPP
