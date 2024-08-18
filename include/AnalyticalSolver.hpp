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
  AnalyticalSolver(std::shared_ptr<Graph> & graph);

  const Eigen::MatrixXd laplace_matrix_;
  Eigen::VectorXd eigenvalues; 
  Eigen::MatrixXd eigenvectors;

  Eigen::MatrixXd createLaplaceMatrix();

  //returns solver, not eigen-s
  //use solver.eigenvalues() and solver.eigenvectors to get eigen-s
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> computeEigenvaluesAndEigenvectors(const Eigen::MatrixXd& laplace_matrix) const;
  std::vector<double_t> getTemperatures() const;
  bool saveJsonData(std::vector<double_t> & current_temperature, double_t current_time) const;
  std::vector<double_t> computeTemperatures (const double_t current_time) const;

  static std::shared_ptr<AnalyticalSolver> instance_;
  static std::mutex mutex_;
};


#endif
