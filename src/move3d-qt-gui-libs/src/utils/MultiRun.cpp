#include "MultiRun.hpp"
#include "SaveContext.hpp"

#include "API/project.hpp"
#include "API/Roadmap/graph.hpp"
#include "API/Roadmap/graphConverter.hpp"
#include "planner/planEnvironment.hpp"
#include "planner/plannerFunctions.hpp"
#include "planner/TrajectoryOptim/Classic/smoothing.hpp"
#include "planner/TrajectoryOptim/Classic/costOptimization.hpp"
#include "planner/TrajectoryOptim/trajectoryOptim.hpp"

//#include "Greedy/GreedyCost.hpp"

//#include "PlanningThread.hpp"
#include "move3d-headless.h"

#include "Util-pkg.h"
#include "Planner-pkg.h"
#include "Graphic-pkg.h"

#include <string>
#include <iostream>
#include <fstream>
#include <sys/time.h>
#include <algorithm>

using namespace std;
MOVE3D_USING_SHARED_PTR_NAMESPACE

MultiRun::MultiRun()
{

}

/**
 * Saves Cost and Time in a file under the names
 */
void MultiRun::saveVectorToFile(int Context)
{
    std::ostringstream oss;
    oss << getenv("HOME_MOVE3D") << "/statFiles/"<<ENV.getString(Env::nameOfFile).toStdString();

    std::ofstream s;
    s.open(oss.str().c_str());

    cout << "Opening save file : " << oss.str() << endl;

    for (char i = 0; i < mNames.size(); i++)
    {
        s << mNames.at(i) << ";";
    }

    s << endl;


    for (char i=0; i<ENV.getInt(Env::nbMultiRun); i++)
    {
        for (char j=0; j<int(mVectDoubles.size()); j++)
        {
            s << mVectDoubles[j][Context*ENV.getInt(Env::nbMultiRun)+i] << ";";
        }
        s << endl;
    }

    s << endl;

    cout << "Closing save file" << endl;

    s.close();
}

void MultiRun::saveGraph(int i)
{
    std::ostringstream oss;
    oss << getenv("HOME_MOVE3D") << "/statFiles/Graph_"<<i<<".graph";

    cout << "Saving graph to : "<<oss.str() << endl;
    p3d_writeGraph(XYZ_GRAPH, oss.str().c_str(), DEFAULTGRAPH);//Mokhtar Using XML Format.
}

/**
 * Run multiple RRT runs
 */
void MultiRun::runMultiRRT()
{
    cout << "Running Multi RRT" << endl;
    if (storedContext.getNumberStored() == 0)
    {
        cout << "WARNING: No context on the context stack, using current one" << endl;

        storedContext.saveCurrentEnvToStack();

    }
    else
    {
        cout << " Running " << storedContext.getNumberStored() << " RRTs from stored context" << endl;
    }

    mNames.clear();
    mVectDoubles.clear();
    mTime.clear();

    mNames.push_back("Time (RRT)");
    mNames.push_back("Time (Optim)");
    mNames.push_back("Time (Tot)");
    mNames.push_back("NbQRand");
    mNames.push_back("NbNodes");
    mNames.push_back("Cost1");
    mNames.push_back("Cost2");
    mNames.push_back("Integral");
    mNames.push_back("Meca-Work");

  Robot* rob = global_Project->getActiveScene()->getActiveRobot();

    mVectDoubles.resize(9);

    unsigned int context = 0;

    for (unsigned int j = 0; j < storedContext.getNumberStored(); j++)
    {
        storedContext.switchCurrentEnvTo( j );
//		storedPlannerContext->switchCurrentEnvTo( j );
        // storedContext.getTime(j).clear();
        // vector<double> time = storedContext.getTime(j);

        for (int i = 0; i < ENV.getInt(Env::nbMultiRun); i++)
        {
            unsigned int rounds = j == 0 ? 0 : std::pow( (double)10 , (int)j );
            context =  rounds + i;

            double tu(0.0);
            double ts(0.0);

            ChronoOn();

            //p3d_SetStopValue(FALSE);
            PlanEnv->setBool(PlanParam::stopPlanner,false);

            int nbNodes;

            if ( XYZ_GRAPH ) XYZ_GRAPH=NULL;//p3d_del_graph(XYZ_GRAPH);

            try
            {
                p3d_planner_functions_set_run_id( context );
                nbNodes = p3d_run_rrt(rob->getRobotStruct());
            }
            catch (string str)
            {
                cerr << "Exeption in run p3d_run_rrt : " << endl;
                cerr << str << endl;
            }
            catch (...)
            {
                cerr << "Exeption in run p3d_run_rrt" << endl;
            }

            if( nbNodes > 0 )
            {
                ChronoPrint("");
                ChronoTimes(&tu, &ts);
                ChronoOff();

                mTime.push_back(tu);

                //                ENV.setBool(Env::isCostSpace,true);

                Robot currRobot((p3d_rob *) p3d_get_desc_curid(P3D_ROBOT));
                API::Trajectory Traj(
                                                         &currRobot,
                                                         currRobot.getTrajStruct());
                if(XYZ_GRAPH==NULL)
                {
                    GraphConverter gc;
                    XYZ_GRAPH= gc.convert(*API_activeGraph,true);
                }
                mVectDoubles[0].push_back(XYZ_GRAPH->rrtTime);
                mVectDoubles[1].push_back(XYZ_GRAPH->optTime);
                mVectDoubles[2].push_back(XYZ_GRAPH->totTime);

                mVectDoubles[3].push_back( ENV.getInt(Env::nbQRand) );
                mVectDoubles[4].push_back( nbNodes );

                bool tmpUnSetCostSpace =false;
                bool tmpUnSetDistancePath =false;

                if(ENV.getBool(Env::HRIPlannerWS) || ENV.getBool(Env::HRIPlannerCS) )
                {
                    if(!ENV.getBool(Env::isCostSpace))
                    {
                        tmpUnSetCostSpace = true;
                        ENV.setBool(Env::isCostSpace,true);
                    }
                    if(ENV.getBool(Env::HRIPathDistance))
                    {
                        tmpUnSetDistancePath = true;
                        ENV.setBool(Env::HRIPathDistance,false);
                    }
                }

                int tmp = ENV.getInt(Env::costDeltaMethod);
                cout << "Tmp : " << tmp << endl;

                // Before and after Smoothing
                mVectDoubles[5].push_back( XYZ_GRAPH->rrtCost1 );
                mVectDoubles[6].push_back( XYZ_GRAPH->rrtCost2 );

                Traj.resetCostComputed();

                cout << "Saving Integral cost" << endl;
                ENV.setInt( Env::costDeltaMethod , INTEGRAL );
                mVectDoubles[7].push_back( Traj.cost() );

                Traj.resetCostComputed();

                cout << "Saving Minimal-W cost" << endl;
                ENV.setInt( Env::costDeltaMethod, MECHANICAL_WORK );
                mVectDoubles[8].push_back( Traj.cost() );
                cout << "Traj W Cost : " << Traj.cost() << endl;

                ENV.setInt(Env::costDeltaMethod,tmp);

                if( ENV.getBool(Env::HRIPlannerWS)  || ENV.getBool(Env::HRIPlannerCS)  )
                {
                    if(tmpUnSetCostSpace)
                    {
                        tmpUnSetCostSpace = false;
                        ENV.setBool(Env::isCostSpace,false);
                    }
                    if(tmpUnSetDistancePath)
                    {
                        tmpUnSetDistancePath = false;
                        ENV.setBool(Env::HRIPathDistance,true);
                    }
                }

                //                ENV.setBool(Env::isCostSpace,false);

                cout << " Mean Collision test : "  << Traj.meanCollTest() << endl;
                //g3d_draw_allwin_active();
                if(ENV.getBool(Env::StopMultiRun))
                    break;

                saveGraph(i);
            }
            else
            {
                cout << "--------------------------------------------------------------"  << endl;
                cout << "Warning : No traj Found : Problem during trajectory extraction"  << endl;
                cout << "--------------------------------------------------------------"  << endl;
            }
        }

        storedContext.addTime( mTime );
//		storedPlannerContext->addTime( mTime );
        saveVectorToFile( j );
        ENV.setBool(Env::StopMultiRun,false);
        cout << "Save to file" << endl;
    }
    cout << " End of Tests ----------------------" << endl;
    return;
}



void MultiRun::runMultiGreedy()
{

    if (storedContext.getNumberStored() == 0)
    {
        cout << "WARNING: No context on the context stack" << endl;
        return;
    }
    else
    {
        cout << " Running " << storedContext.getNumberStored() << " RRTs from stored context" << endl;
    }

    mNames.clear();
    mVectDoubles.clear();
    mTime.clear();

    mNames.push_back("Time");
    mNames.push_back("NbQRand");
    mNames.push_back("NbNodes");
    mNames.push_back("Integral");
    mNames.push_back("Meca-Work");

    mVectDoubles.resize(5);

    for (unsigned int j = 0; j < storedContext.getNumberStored(); j++)
    {
        storedContext.switchCurrentEnvTo(j);
//		storedPlannerContext->switchCurrentEnvTo(j);
        //			storedContext.getTime(j).clear();
        //			vector<double> time = storedContext.getTime(j);
        for (int i = 0; i < ENV.getInt(Env::nbMultiRun); i++)
        {

            double tu(0.0);
            double ts(0.0);
            ChronoOn();

            //p3d_SetStopValue(FALSE);
            PlanEnv->setBool(PlanParam::stopPlanner,false);

            /*int res = */
            //p3d_RunGreedyCost(XYZ_GRAPH, fct_stop, fct_draw);
            cout << "Attention p3d_RunGreedyCost pas implémenté!!!!" << endl;
            ChronoPrint("");
            ChronoTimes(&tu, &ts);
            ChronoOff();

            p3d_rob *robotPt = (p3d_rob *) p3d_get_desc_curid(P3D_ROBOT);
            p3d_traj* CurrentTrajPt = robotPt->tcur;
            if (CurrentTrajPt == NULL)
            {
                PrintInfo(("Warning: no current trajectory to optimize\n"));
                continue;
            }

            Robot currRobot((p3d_rob *) p3d_get_desc_curid(P3D_ROBOT));

            API::Smoothing optimTrj(
                                                            &currRobot,
                                                            currRobot.getTrajStruct());

            mTime.push_back(tu);
            mVectDoubles[0].push_back(tu);

            mVectDoubles[1].push_back( ENV.getInt(Env::nbQRand) );
            mVectDoubles[2].push_back( XYZ_GRAPH->nnode );


            ENV.setInt(Env::costDeltaMethod,INTEGRAL);
            mVectDoubles[3].push_back( optimTrj.cost() );

            ENV.setInt(Env::costDeltaMethod,MECHANICAL_WORK);
            mVectDoubles[4].push_back( optimTrj.costDeltaAlongTraj() );

            cout << " Mean Collision test : "  << optimTrj.meanCollTest() << endl;
        }
        storedContext.addTime(mTime);
//		storedPlannerContext->addTime(mTime);
        saveVectorToFile(j);
    }

    cout << " End of Tests ----------------------" << endl;
    return;
}

/**
 * Compute Mean
 */
void MultiRun::computeConvergence( int run_id, double time )
{
  if (traj_convergence_with_time.empty()) {
    return;
  }

//  cout << endl;
//  cout << "traj_convergence_with_time.size() : " << traj_convergence_with_time.size() << endl;

  int nb_samples = 100;
  double step = (time/(nb_samples-1));

  mConvergence[run_id].clear();

  // Resamples the vector to build
  // the convergence rate vector
  for ( double t=0.0; t<=(time+0.0001); t += step )
  {
    int ith = 0;
    double min = numeric_limits<double>::max();

    // Find the step in vector with closest time step
    for(int i=0;i<int(traj_convergence_with_time.size()); i++)
    {
      if ( min > abs(traj_convergence_with_time[i].first-t) )
      {
        min = abs(traj_convergence_with_time[i].first-t);
        ith = i;
        continue;
      }
    }

    mConvergence[run_id].push_back( traj_convergence_with_time[ith].second );
  }


  double max = *std::max_element( mConvergence[run_id].begin(), mConvergence[run_id].end() );

  for(int i=0;i<int(mConvergence[run_id].size()); i++)
  {
    if(  mConvergence[run_id][i] > max )
    {
      mConvergence[run_id][i] = max;
    }
  }

//  for(int i=0;i<int(traj_convergence_with_time.size()); i++)
//  {
//    cout << "traj_convergence_with_time[" << i << "].first = " << traj_convergence_with_time[i].first << endl;
//    cout << "traj_convergence_with_time[" << i << "].secon = " << traj_convergence_with_time[i].second << endl;
//  }
//
//  for(int i=0;i<int(mConvergence[run_id].size()); i++)
//  {
//    cout << "mConvergence[" << run_id << "][" << i <<"]= " << mConvergence[run_id][i] << endl;
//  }

  if( int(mConvergence[run_id].size()) != (nb_samples) )
  {
    mConvergence[run_id].resize( nb_samples );
  }
}

void MultiRun::computeAverageConvergenceAndSave()
{
  if( mConvergence.empty() ) {
    return;
  }

  // Compute Average
  int nb_samples = mConvergence[0].size();
  vector<double> average;

  for(int j=0;j<nb_samples; j++)
  {
    double value = 0.0;

    for(int i=0;i<int(mConvergence.size()); i++) {
      value += mConvergence[i][j];
    }

    average.push_back( value/double(mConvergence.size()) );
  }

  // Save to file
  std::string home( getenv("HOME_MOVE3D") );
  std::ostringstream oss;
    oss << "/statFiles/Average.csv";

    const char *path = (home+oss.str()).c_str();

    std::ofstream s;
    s.open(path);

    cout << "Opening save file : " << path << endl;

    for (int i=0; i<int(average.size()); i++)
    {
    s << average[i] << ";";
        s << endl;
    }

  s << endl;
  s.close();
}

//void MultiRun::loadTraj()
//{
//    char file_name[256];
//    sprintf(file_name,"/Users/jmainpri/Desktop/STomp/ManipTraj.traj");

//    p3d_rob *robotPt = (p3d_rob*) p3d_get_desc_curid(P3D_ROBOT);
//    int ir;
//    configPt qi, qf;
//    pp3d_traj trajPt;

//  cout << "Read for robot : " << robotPt->name << endl;

//    if (file_name!=NULL)
//    {
//        if (p3d_read_traj(file_name))
//        {
//            trajPt = (p3d_traj *) p3d_get_desc_curid(P3D_TRAJ);
//            ir = p3d_get_desc_curnum(P3D_ROBOT);

//            qi = p3d_alloc_config(robotPt);
//            qf = p3d_alloc_config(robotPt);
//            p3d_ends_and_length_traj(trajPt, &qi, &qf);
//            p3d_copy_config_into(robotPt, qf, &(robotPt->ROBOT_GOTO));
//            p3d_copy_config_into(robotPt, qi, &(robotPt->ROBOT_POS));

//      g3d_draw_allwin_active();
//            p3d_destroy_config(robotPt, qi);
//            p3d_destroy_config(robotPt, qf);
//        }
//    }
//}

bool MultiRun::runSingleRRT()
{
  bool result = false;

  timeval tim;
  gettimeofday(&tim, NULL);
  double t_init = tim.tv_sec+(tim.tv_usec/1000000.0);

  result = p3d_run_rrt(mRobot->getRobotStruct());

  gettimeofday(&tim, NULL);
  double dt = tim.tv_sec+(tim.tv_usec/1000000.0) - t_init;
  cout << "RRT computed in : " << dt << " sec" << endl;

  if( !ENV.getBool(Env::drawDisabled) ) {
    g3d_draw_allwin_active();
  }

  return result;
}


/**
 * Run multiple Smooth runs
 */
void MultiRun::runMultiSmooth()
{
  int nb_runs = ENV.getInt(Env::nbMultiSmooth);
  double time_limit = 10.0;

  mTime.clear();

    mNames.clear();
    mNames.push_back("Time");
  mNames.push_back("Iteration");
    mNames.push_back("Cost");

  mVectDoubles.clear();
    mVectDoubles.resize(3);

  mConvergence.resize( nb_runs );

  mRobot = global_Project->getActiveScene()->getRobotByNameContaining("PR2_ROBOT");

  if( mRobot == NULL ) {
    cout << "Change the robot name int " << __func__ << endl;
    return;
  }

  ENV.setBool(Env::isRunning,true);

    for(int i =0;i<nb_runs;i++)
    {
        //loadTraj();
    ENV.setBool(Env::isCostSpace,false);

    if( !runSingleRRT() ) {
      continue;
    }

        API::CostOptimization optimTrj( mRobot, mRobot->getTrajStruct() );

        cout << "Run Nb"  << i << " = " << endl;


    ENV.setBool(Env::isCostSpace,true);

    timeval tim;
    gettimeofday(&tim, NULL);
    double t_init = tim.tv_sec+(tim.tv_usec/1000000.0);
    double time = 0.0;

        if( PlanEnv->getBool(PlanParam::withDeformation) )
        {
      PlanEnv->setBool(PlanParam::trajStompComputeColl, false );
      PlanEnv->setBool( PlanParam::trajWithTimeLimit, true );
      PlanEnv->setDouble( PlanParam::timeLimitSmoothing, time_limit );

      optimTrj.cutTrajInSmallLP( PlanEnv->getInt( PlanParam::nb_pointsOnTraj ) );
            optimTrj.runDeformation( PlanEnv->getInt(PlanParam::smoothMaxIterations) , i );
      optimTrj.replaceP3dTraj();
        }
    else if( PlanEnv->getBool(PlanParam::withShortCut) )
        {
      PlanEnv->setBool(PlanParam::trajStompComputeColl, false );
      PlanEnv->setBool( PlanParam::trajWithTimeLimit, true );
      PlanEnv->setDouble( PlanParam::timeLimitSmoothing, time_limit );

      optimTrj.cutTrajInSmallLP( PlanEnv->getInt( PlanParam::nb_pointsOnTraj ) );
            optimTrj.runShortCut( PlanEnv->getInt(PlanParam::smoothMaxIterations) , i );
      optimTrj.replaceP3dTraj();
        }
        else if( PlanEnv->getBool(PlanParam::withStomp) )
    {
      PlanEnv->setBool(PlanParam::trajStompComputeColl, true );
      PlanEnv->setBool( PlanParam::withCurrentTraj, true );
      PlanEnv->setBool( PlanParam::trajStompWithTimeLimit, true );
      PlanEnv->setDouble( PlanParam::trajStompTimeLimit , time_limit );

      traj_optim_runStomp(i);
    }

        gettimeofday(&tim, NULL);
    time = tim.tv_sec+(tim.tv_usec/1000000.0) - t_init;
    cout << "Time optim : " << time << endl;

    computeConvergence( i, time_limit );

        mTime.push_back( time );
        mVectDoubles[0].push_back( time );
    mVectDoubles[1].push_back( optimTrj.getIteration() );
        mVectDoubles[2].push_back( optimTrj.cost() );

        //ENV.setBool(Env::drawTraj,true);
        g3d_draw_allwin_active();

        if(ENV.getBool(Env::StopMultiRun))
        {
            break;
        }
    }

  computeAverageConvergenceAndSave();
    saveVectorToFile(0);

  ENV.setBool(Env::isRunning,false);

    cout << " End of Tests ----------------------" << endl;
    return;
}
