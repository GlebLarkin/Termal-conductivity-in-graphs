#ifndef NUMERICALSOLVER_HPP
#define NUMERICALSOLVER_HPP

#include "Solver.hpp"

class NumericalSolver : public Solver
{
public:
  static std::shared_ptr<NumericalSolver> getInstance();

  //saves temperature on time dependence data in numerical_output.json
  bool solve(const double_t measurment_time, const double_t dt) const override;

private:
  NumericalSolver(std::shared_ptr<Graph> graph);

  static std::shared_ptr<NumericalSolver> instance_;
  static std::mutex mutex_;
};


#endif
