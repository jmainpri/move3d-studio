#ifndef MULTIRUNS_HPP
#define MULTIRUNS_HPP

#include <vector>
#include <string>

#include "API/Device/robot.hpp"

/**
 * Enables mutliple runs
 */
class MultiRun
{
public:
  MultiRun();

  void runMultiRRT();
  void runMultiGreedy();
  void runMultiSmooth();
  
private:
  void saveVectorToFile(int Context);
  void saveGraph(int i);
  void loadTrajectory();
  bool runSingleRRT();
  
  void computeConvergence(int run_id, double time);
  void computeAverageConvergenceAndSave();
  
  std::vector< std::string>                mNames;
  std::vector< std::vector<double> >       mVectDoubles;
  std::vector< double >                    mTime;
  std::vector< std::vector<double> >       mConvergence;
  
  Move3D::Robot* mRobot;
};

#endif // MULTIRUNS_HPP
