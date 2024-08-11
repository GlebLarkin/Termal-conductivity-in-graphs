#ifndef ANALYTICALSOLVER_HPP
#define ANALYTICALSOLVER_HPP

#include "Solver.hpp"

class AnalyticalSolver : public Solver
{
public:
  static std::shared_ptr<AnalyticalSolver> getInstance();

  //saves temperature on time dependence data in analytic_output.json
  bool solve(const double_t measurment_time, const double_t dt) const override;

private:
  AnalyticalSolver(std::shared_ptr<Graph> graph);

  Eigen::MatrixXd laplace_matrix; //= std::move<createLaplaceMatrix()>

  Eigen::MatrixXd createLaplaceMatrix() const;
  bool computeEigenvaluesAndEigenvectors(const Eigen::MatrixXd& laplace_matrix) const;
  bool saveJsonData(const std::vector<double_t> & current_time);
  std::vector<double_t> computeTemperatures (const double_t measurment_time, const double_t dt) const;

  static std::shared_ptr<AnalyticalSolver> instance_;
  static std::mutex mutex_;
};


#endif
