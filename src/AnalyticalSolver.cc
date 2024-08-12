#include "AnalyticalSolver.hpp"

std::shared_ptr<AnalyticalSolver> AnalyticalSolver::instance_ = nullptr;
std::mutex AnalyticalSolver::mutex_;

std::shared_ptr<AnalyticalSolver> AnalyticalSolver::getInstance()
{
  std::lock_guard<std::mutex> lock(mutex_); 
  if (!instance_)
  {
    instance_ = std::shared_ptr<AnalyticalSolver>(new AnalyticalSolver(Graph::getInstance()));
  }
  return instance_;
}

AnalyticalSolver::AnalyticalSolver(std::shared_ptr<Graph> & graph_ptr)
  : Solver(graph_ptr), laplace_matrix(std::move(createLaplaceMatrix())) {}

bool AnalyticalSolver::solve(const double_t measurment_time, const double_t dt) const
{

}