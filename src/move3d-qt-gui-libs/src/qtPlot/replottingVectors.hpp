#ifndef _REPLOTTING_PLOT_H
#define _REPLOTTING_PLOT_H

#include <qwt_plot.h>
#include <qwt_plot_curve.h>
#include <vector>

const int REPLOT_SIZE = 100;      // 0 to 200

/**
 * @ingroup qtPlot
 * @brief Qt simple plot relies on qwt
 */
class ReplottingVectors : public QwtPlot
{
  Q_OBJECT
  
public:
  ReplottingVectors(QWidget* = NULL);
  int getPlotSize() { return REPLOT_SIZE; }
  
  void addData(const std::vector<const std::vector<double>* >& data);
  void clearData();
  
  void setTimerInterval(double interval);
  
protected:
  virtual void timerEvent(QTimerEvent *e);
  
private:
  void alignScales();
	
	std::vector< QwtPlotCurve* > cData;
  std::vector<double> Max_y;
  QwtArray< double >								d_x;
	std::vector< QwtArray< double > > d_y;
  std::vector< const std::vector<double>* > m_data;
  
  bool init;
  double m_Max_y;
  double m_Min_y;
  
  int d_interval; // timer in ms
  int d_timerId;
};

#endif
