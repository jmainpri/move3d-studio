#include "guiparams.hpp"

#include "move3d-headless.h"
#include <iostream>
//#include "../p3d/env.hpp"

// A new container is created for each module
// First declaire the maps of praramters
// Then fill in the maps that associate the enum to the Qt container
// When Qt is disabled this just acts as a normal container

// Definition of the parameter container
Parameters<
GuiParam::boolParameter,
GuiParam::intParameter,
GuiParam::doubleParameter,
GuiParam::stringParameter,
GuiParam::vectorParameter>* GuiEnv = NULL;

#ifdef QT_LIBRARY
GuiParam* EnumGuiParameterObject = NULL;

GuiParam::GuiParam()
{

}

GuiParam::~GuiParam()
{

}
#endif

// @brief Function that inizializes the
// Parameter container
void initGuiParameters()
{
#ifdef QT_LIBRARY
    EnumGuiParameterObject = new GuiParam;
#endif

    // Create 5 maps for all types and fill the 5 maps
    // ------------------------------------------------------------------
    std::map<GuiParam::boolParameter,      boolContainer*>                  myBoolMap;
    std::map<GuiParam::intParameter,       intContainer*>                   myIntMap;
    std::map<GuiParam::doubleParameter,    doubleContainer*>                myDoubleMap;
    std::map<GuiParam::stringParameter,    stringContainer*>                myStringMap;
    std::map<GuiParam::vectorParameter,    vectorContainer*>                myVectorMap;

    // Bool
    // ------------------------------------------------------------------
    myBoolMap.insert( std::make_pair( GuiParam::tete,   new boolContainer(false) ));

    // Int
    // ------------------------------------------------------------------
    myIntMap.insert( std::make_pair( GuiParam::mainwin_x,    new intContainer(0) ));
    myIntMap.insert( std::make_pair( GuiParam::mainwin_y,    new intContainer(0) ));
    myIntMap.insert( std::make_pair( GuiParam::mainwin_w,    new intContainer(1000) ));
    myIntMap.insert( std::make_pair( GuiParam::mainwin_h,    new intContainer(600) ));

    // Double
    // ------------------------------------------------------------------
    myDoubleMap.insert( std::make_pair( GuiParam::toto, new doubleContainer(10.0) ));

    //cout << "PlanEnv->getDouble(p) = " << PlanEnv->getDouble( PlanParam::env_objectNessecity ) << endl;

    // String
    // ------------------------------------------------------------------
#ifdef QT_LIBRARY
    myStringMap.insert(std::make_pair(GuiParam::titi, new stringContainer("titi")));
#endif

    // Vector
    // ------------------------------------------------------------------
    std::vector<double> tutu;
    tutu.push_back( 1 ); tutu.push_back( 8 );

    myVectorMap.insert(std::make_pair(GuiParam::tutu,  new vectorContainer(tutu)));

    // Make the new parameter container
    GuiEnv =  new Parameters<
            GuiParam::boolParameter,
            GuiParam::intParameter,
            GuiParam::doubleParameter,
            GuiParam::stringParameter,
            GuiParam::vectorParameter>(
                myBoolMap,
                myIntMap,
                myDoubleMap,
                myStringMap,
                myVectorMap);
}

