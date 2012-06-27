#ifndef MULTIPLOT_HPP
#define MULTIPLOT_HPP

#include "qtPlot/basicPlot.hpp"

#include<vector>

/**
 * @ingroup qtPlot
 * @brief Qt simple plot relies on qwt
 */
class MultiPlot : public QwtPlot
{
	Q_OBJECT
	
public:
	MultiPlot(QWidget* = NULL);
	
	int getPlotSize() { return PLOT_SIZE; }
	void setData(const std::vector< std::string >& names , 
							 const std::vector< std::vector <double> >& data );
	void rescale();
	
private:
	void alignScales();
	
	QwtArray< double >								d_x;
	std::vector< QwtArray< double > > d_y;
	
	bool init;
	
	std::vector<double> Max_y;
	std::vector< QwtPlotCurve* > cData;
};

#endif // MultiPlot_HPP
