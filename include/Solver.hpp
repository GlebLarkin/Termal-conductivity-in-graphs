#ifndef SOLVER_HPP
#define SOLVER_HPP

#include "Graph.hpp"


class Solver
{
public:
  virtual ~Solver() = default;

  virtual bool solve(const double_t measurment_time, const double_t dt) const = 0;

protected:
  Solver(const std::shared_ptr<Graph>& graph_ptr);

  const std::shared_ptr<Graph> graph_ptr_;

private:
  Solver() = delete;
  Solver(const Solver&) = delete;
  Solver& operator=(const Solver&) = delete;
};


#endif
