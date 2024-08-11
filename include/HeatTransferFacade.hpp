#ifndef HEATTRANSFERFACADE_HPP
#define HEATTRANSFERFACADE_HPP

#include "Graph.hpp"


class HeatTransferFacade 
//Singleton
{
public:
  static HeatTransferFacade& getInstance();

  bool getJsonData() const;

  ///Analytical Solution///
  bool createLaplaceMatrix() const;

  bool computeEigenvaluesAndEigenvectors(const Eigen::MatrixXd& matrix) const;

  //saves temperature on time dependence in Analytical_output.json file
  bool solveAnalyticly(double measurement_time, double dt) const;
  /////////////////////////

  ///Numerical solution///
  //saves temperature on time dependence in Numerical_output.json file
  bool solveNumerically(double measurement_time, double dt) const;
  ////////////////////////

  ///Comparation between analytical and numerical methods///
  //compares total temperature in both methods (let it be energy ~ temperature)
  double computeTotalEnergyDifference() const;
  //compares max temperature difference in each step
  double computeMaxTemperatureDifference() const;
  ///////////////////////////////////////////////////////

private:
  HeatTransferFacade(Graph graph);
  HeatTransferFacade(const HeatTransferFacade&) = delete;
  HeatTransferFacade& operator=(const HeatTransferFacade&) = delete;
};


#endif
