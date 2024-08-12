#include "Solver.hpp"

Solver::Solver(const std::shared_ptr<Graph> & graph_ptr) : graph_ptr_(graph_ptr) {}

std::shared_ptr<Graph> Solver::getGraph() const 
{
  return graph_ptr_;
}