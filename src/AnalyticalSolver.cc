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
  : Solver(graph_ptr), laplace_matrix_(std::move(createLaplaceMatrix())) {}


bool AnalyticalSolver::solve(const double_t measurment_time, const double_t dt) const
{
  AnalyticalSolver::computeEigenvaluesAndEigenvectors(laplace_matrix_);
  for (auto current_time = 0.0; current_time < measurment_time; current_time += dt)
  {
    auto current_temp = AnalyticalSolver::computeTemperatures(current_time);
    AnalyticalSolver::saveJsonData(current_temp, current_time);
  }
}

//this func turns keys into indexes for laplace matrix being correct (indexes < matrix size)
std::unordered_map<uint32_t, uint32_t> keyToIndex(const std::shared_ptr<const std::unordered_map<uint32_t, Node>> & map)
{
  std::unordered_map<uint32_t, uint32_t> index_map;
  auto index = 0;

  for (const auto & [key, _] : *map) index_map.emplace(key, index++);

  return index_map;
}

Eigen::MatrixXd AnalyticalSolver::createLaplaceMatrix() 
{
  auto adjacency_list_ptr_ = graph_ptr_->getAdjacencyListPtr();
  auto n = adjacency_list_ptr_->size();
  Eigen::MatrixXd laplace_matrix = Eigen::MatrixXd::Zero(n, n);

  auto index_map = keyToIndex(adjacency_list_ptr_);

  for (const auto& [key, node] : *adjacency_list_ptr_) 
  {
    auto i = index_map[key];

    laplace_matrix(i, i) = static_cast<double>(node.connected_edges.size());

    for (auto neighbor_key : node.connected_edges)
    {
      auto neighbor = index_map[neighbor_key]; 

      if (neighbor == i) continue;
      laplace_matrix(i, neighbor) = -1;
    }
  }

  return laplace_matrix;
}
